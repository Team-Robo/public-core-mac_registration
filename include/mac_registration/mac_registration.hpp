// Copyright 2026 Dun Yan Ong. Licensed under the Apache License, Version 2.0.
//
// mac_registration node: live cloud -> pose of the object (or scene) relative to an
// auto-captured reference, with MAC as the global registration step.
//
// Threads: the ROS executor only drops each cloud into a one-slot mailbox (older clouds are
// overwritten, so the node never falls behind); a worker thread processes the latest cloud.
#pragma once

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <vision_msgs/msg/detection3_d.hpp>

#include "mac_registration/msg/registration_stats.hpp"
#include "mac_registration/pipeline.hpp"
#include "mac_registration/reference_store.hpp"

namespace mac_registration
{

class MacRegistrationNode : public rclcpp::Node
{
public:
  explicit MacRegistrationNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~MacRegistrationNode() override;

private:
  struct Parameters
  {
    std::string label;
    std::string intensity_mode;  // target | obstacle | all
    std::string reference_dir;
    bool auto_capture{true};
    mac::CaptureRule capture;
    bool publish_tf{false};
    std::string tf_child_frame;
    bool publish_aligned_cloud{true};
    bool publish_diagnostics{true};
    double warn_throttle_sec{2.0};
    mac::PipelineParams pipeline;
  };

  void declare_parameters();
  void on_cloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  void worker_loop();
  void process(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg);
  void handle_capture(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg, const mac::Cloud::Ptr & raw,
    bool forced, msg::RegistrationStats & stats);
  bool load_cached_reference();
  void set_reference(const mac::Cloud::ConstPtr & raw);
  mac::Cloud::Ptr extract_points(const sensor_msgs::msg::PointCloud2 & msg);

  void publish_pose(const std_msgs::msg::Header & header, const Eigen::Matrix4d & T, double score);
  void publish_aligned(const std_msgs::msg::Header & header, const Eigen::Matrix4d & T);
  void publish_tf(const std_msgs::msg::Header & header, const Eigen::Matrix4d & object_pose);

  Parameters params_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<vision_msgs::msg::Detection3D>::SharedPtr pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_pub_;
  rclcpp::Publisher<msg::RegistrationStats>::SharedPtr diag_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr capture_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_srv_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Mailbox shared by the executor (writer) and the worker (reader).
  std::mutex mailbox_mutex_;
  std::condition_variable mailbox_cv_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr mailbox_;
  bool stop_{false};
  std::thread worker_;
  std::atomic<bool> force_capture_{false};
  std::atomic<bool> clear_requested_{false};

  // Owned by the worker thread.
  mac::ReferenceStore store_;
  mac::CaptureGate capture_gate_;
  std::shared_ptr<mac::Reference> reference_;
  bool warned_no_intensity_{false};
  bool announced_waiting_{false};
};

}  // namespace mac_registration
