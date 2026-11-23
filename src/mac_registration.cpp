// Copyright 2026 Dun Yan Ong. Licensed under the Apache License, Version 2.0.
#include "mac_registration/mac_registration.hpp"

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>

#include <geometry_msgs/msg/transform_stamped.hpp>

namespace mac_registration
{
namespace
{

geometry_msgs::msg::Pose to_pose_msg(const Eigen::Matrix4d & P)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = P(0, 3);
  pose.position.y = P(1, 3);
  pose.position.z = P(2, 3);
  Eigen::Quaterniond q(Eigen::Matrix3d(P.topLeftCorner<3, 3>()));
  q.normalize();
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();
  return pose;
}

}  // namespace

MacRegistrationNode::MacRegistrationNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("mac_registration", options)
{
  declare_parameters();

  store_ = mac::ReferenceStore(params_.reference_dir, params_.label);
  capture_gate_.set_rule(params_.capture);

  pose_pub_ = create_publisher<vision_msgs::msg::Detection3D>("~/pose", 10);
  aligned_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("~/aligned_cloud", 5);
  diag_pub_ = create_publisher<msg::RegistrationStats>("~/diagnostics", 10);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  capture_srv_ = create_service<std_srvs::srv::Trigger>(
    "~/capture_reference",
    [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
      force_capture_ = true;
      res->success = true;
      res->message = "a new reference will be captured from the next frame with enough points";
    });
  clear_srv_ = create_service<std_srvs::srv::Trigger>(
    "~/clear_reference",
    [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
      clear_requested_ = true;
      res->success = true;
      res->message = "reference '" + params_.label + "' will be deleted; " +
      (params_.auto_capture ? "the next stable frame becomes the new one" :
      "call ~/capture_reference to take a new one");
    });

  if (!load_cached_reference()) {
    RCLCPP_INFO(get_logger(), "No cached reference for '%s' (%s). Waiting for %d stable frames.",
      params_.label.c_str(), store_.pcd_path().c_str(), params_.capture.stability_frames);
  }

  worker_ = std::thread(&MacRegistrationNode::worker_loop, this);
  // Best effort, newest only: matches every cloud publisher and never queues stale frames.
  sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "~/input_cloud", rclcpp::SensorDataQoS().keep_last(1),
    std::bind(&MacRegistrationNode::on_cloud, this, std::placeholders::_1));
  RCLCPP_INFO(get_logger(),
    "mac_registration ready: input '%s', label '%s', voxel %.4f m, max %d correspondences, "
    "%d threads, gate {inliers >= %d, mae <= %.3f m, fitness >= %.2f}",
    sub_->get_topic_name(), params_.label.c_str(), params_.pipeline.frontend.voxel_size,
    params_.pipeline.frontend.max_correspondences, params_.pipeline.mac.num_threads,
    params_.pipeline.gate.min_inliers, params_.pipeline.gate.max_mae, params_.pipeline.gate.min_fitness);
}

MacRegistrationNode::~MacRegistrationNode()
{
  {
    std::lock_guard<std::mutex> lock(mailbox_mutex_);
    stop_ = true;
  }
  mailbox_cv_.notify_all();
  if (worker_.joinable()) {
    worker_.join();
  }
}

void MacRegistrationNode::declare_parameters()
{
  Parameters & p = params_;
  mac::PipelineParams & pp = p.pipeline;
  p.label = declare_parameter<std::string>("label", "default");
  p.intensity_mode = declare_parameter<std::string>("intensity_mode", "target");
  p.reference_dir = declare_parameter<std::string>("reference_dir", "~/.ros/mac_registration");
  p.auto_capture = declare_parameter<bool>("auto_capture", true);
  p.capture.min_points = declare_parameter<int>("capture_min_points", 200);
  p.capture.min_extent = declare_parameter<double>("capture_min_extent", 0.03);
  p.capture.max_extent = declare_parameter<double>("capture_max_extent", 1.5);
  p.capture.stability_frames = declare_parameter<int>("capture_stability_frames", 3);
  p.capture.centroid_tolerance = declare_parameter<double>("capture_centroid_tolerance", 0.005);
  p.capture.warmup_frames = declare_parameter<int>("capture_warmup_frames", 15);
  p.publish_tf = declare_parameter<bool>("publish_tf", false);
  p.tf_child_frame = declare_parameter<std::string>("tf_child_frame", "");
  p.publish_aligned_cloud = declare_parameter<bool>("publish_aligned_cloud", true);
  p.publish_diagnostics = declare_parameter<bool>("publish_diagnostics", true);
  p.warn_throttle_sec = declare_parameter<double>("warn_throttle_sec", 2.0);

  int threads = declare_parameter<int>("num_threads", 0);
  if (threads <= 0) {
    threads = std::max(1, std::min(4, static_cast<int>(std::thread::hardware_concurrency())));
  }

  mac::FrontendParams & fe = pp.frontend;
  fe.voxel_size = declare_parameter<double>("voxel_size", 0.005);
  fe.normal_radius = declare_parameter<double>("normal_radius", 0.0);
  fe.fpfh_radius = declare_parameter<double>("fpfh_radius", 0.0);
  fe.keypoints = declare_parameter<std::string>("keypoints", "none");
  fe.max_keypoints = declare_parameter<int>("max_keypoints", 2000);
  fe.mutual_matching = declare_parameter<bool>("mutual_matching", true);
  fe.max_correspondences = declare_parameter<int>("max_correspondences", 1000);
  fe.random_seed = static_cast<unsigned int>(declare_parameter<int>("random_seed", 42));
  fe.num_threads = threads;
  pp.min_correspondences = declare_parameter<int>("min_correspondences", 10);
  pp.point_resolution = declare_parameter<double>("point_resolution", 0.0);

  mac::Params & m = pp.mac;
  m.use_sog = declare_parameter<bool>("mac.use_sog", true);
  const std::string metric = declare_parameter<std::string>("mac.evaluation_metric", "MAE");
  if (!mac::parse_metric(metric, m.metric)) {
    RCLCPP_WARN(get_logger(), "Unknown mac.evaluation_metric '%s'; using MAE", metric.c_str());
    m.metric = mac::Metric::MAE;
  }
  m.t_cmp = declare_parameter<double>("mac.t_cmp", 0.99);
  m.d_cmp_pr = declare_parameter<double>("mac.d_cmp_pr", 10.0);
  m.inlier_thresh_pr = declare_parameter<double>("mac.inlier_thresh_pr", 10.0);
  m.min_clique_size = declare_parameter<int>("mac.min_clique_size", 3);
  m.clique_ranking = declare_parameter<bool>("mac.clique_ranking", false);
  m.top_k = declare_parameter<int>("mac.top_k", 100);
  m.refinement_iterations = declare_parameter<int>("mac.refinement_iterations", 20);
  m.max_graph_edges = declare_parameter<int64_t>("mac.max_graph_edges", 150000);
  m.max_cliques = declare_parameter<int64_t>("mac.max_cliques", 20000);
  m.num_threads = threads;

  pp.icp_enable = declare_parameter<bool>("icp.enable", true);
  pp.icp_max_corr_dist = declare_parameter<double>("icp.max_corr_dist", 0.0);
  pp.icp_max_iterations = declare_parameter<int>("icp.max_iterations", 30);
  pp.fitness_radius = declare_parameter<double>("fitness_radius", 0.0);
  pp.gate.max_mae = declare_parameter<double>("gate.max_mae", 0.010);
  pp.gate.min_inliers = declare_parameter<int>("gate.min_inliers", 20);
  pp.gate.min_fitness = declare_parameter<double>("gate.min_fitness", 0.4);

  if (p.intensity_mode != "target" && p.intensity_mode != "obstacle" && p.intensity_mode != "all") {
    RCLCPP_WARN(get_logger(), "intensity_mode '%s' unknown; using 'all'", p.intensity_mode.c_str());
    p.intensity_mode = "all";
  }
}

void MacRegistrationNode::on_cloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(mailbox_mutex_);
    mailbox_ = std::move(msg);  // an unprocessed older cloud is simply replaced
  }
  mailbox_cv_.notify_one();
}

void MacRegistrationNode::worker_loop()
{
  while (true) {
    sensor_msgs::msg::PointCloud2::ConstSharedPtr msg;
    {
      std::unique_lock<std::mutex> lock(mailbox_mutex_);
      mailbox_cv_.wait(lock, [this] {return stop_ || mailbox_;});
      if (stop_) {
        return;
      }
      msg = std::move(mailbox_);
      mailbox_.reset();
    }
    try {
      process(msg);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "processing failed: %s", e.what());
    }
  }
}

mac::Cloud::Ptr MacRegistrationNode::extract_points(const sensor_msgs::msg::PointCloud2 & msg)
{
  bool has_intensity = false;
  for (const auto & f : msg.fields) {
    if (f.name == "intensity") {
      has_intensity = true;
      break;
    }
  }
  mac::Cloud::Ptr out(new mac::Cloud);
  if (params_.intensity_mode != "all" && has_intensity) {
    pcl::PointCloud<pcl::PointXYZI> in;
    pcl::fromROSMsg(msg, in);
    const bool want_target = params_.intensity_mode == "target";
    out->reserve(in.size());
    for (const auto & p : in.points) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {continue;}
      if ((p.intensity > 0.5f) == want_target) {
        out->push_back(pcl::PointXYZ(p.x, p.y, p.z));
      }
    }
  } else {
    if (params_.intensity_mode != "all" && !has_intensity && !warned_no_intensity_) {
      RCLCPP_WARN(get_logger(),
        "intensity_mode is '%s' but the input cloud has no intensity field; using every point.",
        params_.intensity_mode.c_str());
      warned_no_intensity_ = true;
    }
    pcl::PointCloud<pcl::PointXYZ> in;
    pcl::fromROSMsg(msg, in);
    out = mac::finite_points(in);
  }
  return out;
}

bool MacRegistrationNode::load_cached_reference()
{
  mac::Cloud::Ptr raw;
  mac::ReferenceMeta meta;
  std::string err;
  if (!store_.load(raw, meta, &err)) {
    return false;
  }
  set_reference(raw);
  if (!reference_->valid()) {
    RCLCPP_WARN(get_logger(), "Cached reference %s is unusable (too few features); ignoring it.",
      store_.pcd_path().c_str());
    reference_.reset();
    return false;
  }
  RCLCPP_INFO(get_logger(), "Loaded cached reference '%s' (%zu points, in '%s'): %s",
    params_.label.c_str(), raw->size(), meta.frame_id.c_str(), store_.pcd_path().c_str());
  return true;
}

void MacRegistrationNode::set_reference(const mac::Cloud::ConstPtr & raw)
{
  reference_ = std::make_shared<mac::Reference>(mac::make_reference(raw, params_.pipeline));
}

void MacRegistrationNode::handle_capture(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg, const mac::Cloud::Ptr & raw,
  bool forced, msg::RegistrationStats & stats)
{
  if (!params_.auto_capture && !forced) {
    stats.reject_reason = "no_reference";
    return;
  }
  const std::string why = capture_gate_.check_frame(*raw);
  if (!why.empty()) {
    capture_gate_.reset();
    stats.reject_reason = why;
    return;
  }
  const bool stable = capture_gate_.update(*raw);
  if (!stable && !forced) {
    stats.reject_reason = capture_gate_.warming_up() ? "capture_warming_up" :
      "capturing_" + std::to_string(capture_gate_.streak()) + "_of_" +
      std::to_string(params_.capture.stability_frames);
    return;
  }
  auto previous = reference_;
  set_reference(raw);
  if (!reference_->valid()) {
    reference_ = previous;
    stats.reject_reason = "capture_failed_no_features";
    return;
  }
  force_capture_ = false;
  capture_gate_.reset();
  announced_waiting_ = false;

  mac::ReferenceMeta meta;
  meta.label = params_.label;
  meta.frame_id = msg->header.frame_id;
  meta.stamp_sec = rclcpp::Time(msg->header.stamp).seconds();
  meta.points = raw->size();
  meta.voxel_size = params_.pipeline.frontend.voxel_size;
  meta.point_resolution = reference_->pr;
  meta.centroid = reference_->centroid;
  std::string err;
  if (!store_.save(*raw, meta, &err)) {
    RCLCPP_WARN(get_logger(), "Reference captured but not cached: %s", err.c_str());
  }
  RCLCPP_INFO(get_logger(),
    "Captured reference '%s' in '%s': %zu points, %zu voxels, %zu described, pr %.4f m -> %s",
    params_.label.c_str(), msg->header.frame_id.c_str(), raw->size(), reference_->voxelized->size(),
    reference_->described.size(), reference_->pr, store_.pcd_path().c_str());

  // By definition the object has not moved relative to itself: publish T = I right away.
  stats.accepted = true;
  stats.reject_reason = "";
  stats.has_reference = true;
  stats.reference_points = static_cast<uint32_t>(reference_->voxelized->size());
  stats.fitness = 1.0;
  stats.point_resolution = reference_->pr;
  const Eigen::Matrix4d I = Eigen::Matrix4d::Identity();
  stats.pose = to_pose_msg(mac::object_pose(I, reference_->centroid));
  publish_pose(msg->header, I, 1.0);
  publish_aligned(msg->header, I);
  publish_tf(msg->header, mac::object_pose(I, reference_->centroid));
}

void MacRegistrationNode::process(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)
{
  const auto t0 = std::chrono::steady_clock::now();
  msg::RegistrationStats stats;
  stats.header = msg->header;
  stats.label = params_.label;

  if (clear_requested_.exchange(false)) {
    store_.remove();
    reference_.reset();
    capture_gate_.reset();
    RCLCPP_INFO(get_logger(), "Reference '%s' cleared; waiting for a stable frame.", params_.label.c_str());
  }

  const mac::Cloud::Ptr raw = extract_points(*msg);
  stats.input_points = static_cast<uint32_t>(raw->size());

  const bool forced = force_capture_.load();
  if (!reference_ || forced) {
    handle_capture(msg, raw, forced, stats);
    if (!reference_ && !announced_waiting_) {
      RCLCPP_INFO(get_logger(), "No reference yet for '%s': %s", params_.label.c_str(),
        stats.reject_reason.c_str());
      announced_waiting_ = true;
    }
    stats.total_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    if (params_.publish_diagnostics) {
      diag_pub_->publish(stats);
    }
    return;
  }

  const std::shared_ptr<mac::Reference> ref = reference_;
  const mac::FrameResult f = mac::register_frame(*ref, raw, params_.pipeline);

  stats.has_reference = true;
  stats.accepted = f.accepted;
  stats.reject_reason = f.reason;
  stats.live_points = static_cast<uint32_t>(f.live_points);
  stats.reference_points = static_cast<uint32_t>(ref->voxelized->size());
  stats.correspondences = static_cast<uint32_t>(f.correspondences);
  stats.graph_edges = static_cast<uint32_t>(f.mac.graph_edges);
  stats.cliques = static_cast<uint32_t>(f.mac.clique_count);
  stats.hypotheses = static_cast<uint32_t>(f.mac.selected_cliques);
  stats.t_cmp_used = f.mac.t_cmp_used;
  stats.inlier_count = static_cast<uint32_t>(f.inliers);
  stats.mae = f.mae;
  stats.fitness = f.fit.fitness;
  stats.cloud_rmse = f.fit.rmse;
  stats.mac_score = f.mac.mac_score;
  stats.point_resolution = f.pr;
  stats.preprocess_ms = f.preprocess_ms;
  stats.features_ms = f.features_ms;
  stats.match_ms = f.match_ms;
  stats.graph_ms = f.mac.stage_ms[0];
  stats.cliques_ms = f.mac.stage_ms[1];
  stats.selection_ms = f.mac.stage_ms[2];
  stats.hypotheses_ms = f.mac.stage_ms[3];
  stats.icp_ms = f.icp_ms;
  const Eigen::Matrix4d object_pose = mac::object_pose(f.T, ref->centroid);
  stats.pose = to_pose_msg(object_pose);

  if (f.accepted) {
    publish_pose(msg->header, f.T, f.fit.fitness);
    publish_aligned(msg->header, f.T);
    publish_tf(msg->header, object_pose);
  } else {
    const auto period = static_cast<int64_t>(params_.warn_throttle_sec * 1000.0);
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), period,
      "No pose for '%s': %s (correspondences %zu, inliers %d, mae %.4f m, fitness %.2f)",
      params_.label.c_str(), f.reason.c_str(), f.correspondences, f.inliers, f.mae, f.fit.fitness);
  }

  stats.total_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
  if (params_.publish_diagnostics) {
    diag_pub_->publish(stats);
  }
}

void MacRegistrationNode::publish_pose(
  const std_msgs::msg::Header & header, const Eigen::Matrix4d & T, double score)
{
  if (!reference_) {
    return;
  }
  const Eigen::Matrix4d P = mac::object_pose(T, reference_->centroid);
  vision_msgs::msg::Detection3D det;
  det.header = header;  // stamp and frame of the INPUT cloud, never base_link
  det.id = params_.label;
  vision_msgs::msg::ObjectHypothesisWithPose hyp;
  hyp.hypothesis.class_id = params_.label;
  hyp.hypothesis.score = score;
  hyp.pose.pose = to_pose_msg(P);
  det.results.push_back(hyp);

  // Box around the reference moved by T (axis-aligned in the input frame).
  pcl::PointCloud<pcl::PointXYZ> moved;
  pcl::transformPointCloud(*reference_->voxelized, moved, T.cast<float>());
  if (!moved.empty()) {
    pcl::PointXYZ lo, hi;
    pcl::getMinMax3D(moved, lo, hi);
    det.bbox.center.position.x = 0.5 * (lo.x + hi.x);
    det.bbox.center.position.y = 0.5 * (lo.y + hi.y);
    det.bbox.center.position.z = 0.5 * (lo.z + hi.z);
    det.bbox.center.orientation.w = 1.0;
    det.bbox.size.x = hi.x - lo.x;
    det.bbox.size.y = hi.y - lo.y;
    det.bbox.size.z = hi.z - lo.z;
  }
  pose_pub_->publish(det);
}

void MacRegistrationNode::publish_aligned(const std_msgs::msg::Header & header, const Eigen::Matrix4d & T)
{
  if (!params_.publish_aligned_cloud || !reference_) {
    return;
  }
  pcl::PointCloud<pcl::PointXYZ> moved;
  pcl::transformPointCloud(*reference_->voxelized, moved, T.cast<float>());
  sensor_msgs::msg::PointCloud2 out;
  pcl::toROSMsg(moved, out);
  out.header = header;
  aligned_pub_->publish(out);
}

void MacRegistrationNode::publish_tf(const std_msgs::msg::Header & header, const Eigen::Matrix4d & P)
{
  if (!params_.publish_tf) {
    return;
  }
  geometry_msgs::msg::TransformStamped tf;
  tf.header = header;
  tf.child_frame_id = params_.tf_child_frame.empty() ? params_.label + "_frame" : params_.tf_child_frame;
  const geometry_msgs::msg::Pose pose = to_pose_msg(P);
  tf.transform.translation.x = pose.position.x;
  tf.transform.translation.y = pose.position.y;
  tf.transform.translation.z = pose.position.z;
  tf.transform.rotation = pose.orientation;
  tf_broadcaster_->sendTransform(tf);
}

}  // namespace mac_registration
