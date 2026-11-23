// Copyright 2026 Dun Yan Ong. Licensed under the Apache License, Version 2.0.
//
// tabletop_object_cloud_node: a GPU-free stand-in for the team's YOLO/SAM -> object_pointcloud
// chain. Aligned depth + colour camera_info -> points -> table plane removed -> the cluster
// nearest the image centre is "the object".
//
// Output has the same format as /yolo_sam/cleaned_points: PointXYZI, intensity 1.0 = object,
// 0.0 = everything else in the crop. The header is the depth image's (frame
// camera_color_optical_frame for aligned depth, stamp = capture time).
#include <pcl_conversions/pcl_conversions.h>

#include <chrono>
#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "mac_registration/depth_cloud.hpp"
#include "mac_registration/tabletop.hpp"

namespace mac_registration
{

class TabletopObjectCloudNode : public rclcpp::Node
{
public:
  explicit TabletopObjectCloudNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node("tabletop_object_cloud", options)
  {
    const auto depth_topic = declare_parameter<std::string>(
      "depth_topic", "/camera/camera/aligned_depth_to_color/image_raw");
    const auto info_topic = declare_parameter<std::string>(
      "camera_info_topic", "/camera/camera/color/camera_info");
    const auto out_topic = declare_parameter<std::string>(
      "output_topic", "/mac_registration/tabletop_points");
    dp_.stride = declare_parameter<int>("pixel_stride", 2);
    dp_.z_min = declare_parameter<double>("z_min", 0.3);
    dp_.z_max = declare_parameter<double>("z_max", 1.2);
    dp_.depth_scale = declare_parameter<double>("depth_scale", 0.001);
    tp_.roi_min_x = declare_parameter<double>("roi_min_x", -0.4);
    tp_.roi_max_x = declare_parameter<double>("roi_max_x", 0.4);
    tp_.roi_min_y = declare_parameter<double>("roi_min_y", -0.4);
    tp_.roi_max_y = declare_parameter<double>("roi_max_y", 0.4);
    tp_.plane_distance = declare_parameter<double>("plane_distance", 0.01);
    tp_.plane_min_share = declare_parameter<double>("plane_min_share", 0.15);
    tp_.max_planes = declare_parameter<int>("max_planes", 3);
    tp_.cluster_tolerance = declare_parameter<double>("cluster_tolerance", 0.01);
    tp_.min_cluster_points = declare_parameter<int>("min_cluster_points", 200);
    tp_.max_cluster_points = declare_parameter<int>("max_cluster_points", 50000);
    tp_.work_voxel = declare_parameter<double>("work_voxel", 0.003);
    rest_stride_ = std::max<int>(1, declare_parameter<int>("rest_stride", 4));
    const double max_rate = declare_parameter<double>("max_rate_hz", 10.0);
    min_period_ = max_rate > 0 ? 1.0 / max_rate : 0.0;

    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(out_topic, rclcpp::SensorDataQoS());
    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      info_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr m) {
        std::lock_guard<std::mutex> lock(mutex_);
        K_.fx = m->k[0];
        K_.fy = m->k[4];
        K_.cx = m->k[2];
        K_.cy = m->k[5];
        K_.width = static_cast<int>(m->width);
        K_.height = static_cast<int>(m->height);
      });
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      depth_topic, rclcpp::SensorDataQoS().keep_last(1),
      std::bind(&TabletopObjectCloudNode::on_depth, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "tabletop_object_cloud: %s + %s -> %s (stride %d, z %.2f-%.2f m)",
      depth_sub_->get_topic_name(), info_sub_->get_topic_name(), pub_->get_topic_name(),
      dp_.stride, dp_.z_min, dp_.z_max);
  }

private:
  void on_depth(sensor_msgs::msg::Image::ConstSharedPtr img)
  {
    const double now = this->now().seconds();
    if (min_period_ > 0 && now - last_time_ < min_period_) {
      return;
    }
    last_time_ = now;
    mac::Intrinsics K;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      K = K_;
    }
    if (!K.valid()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "waiting for camera_info");
      return;
    }
    const auto t0 = std::chrono::steady_clock::now();
    const mac::Cloud::Ptr cloud = mac::depth_to_cloud(
      img->data.data(), static_cast<int>(img->width), static_cast<int>(img->height),
      static_cast<int>(img->step), img->encoding, K, dp_);
    const mac::TabletopResult seg = mac::segment_tabletop(cloud, tp_);

    pcl::PointCloud<pcl::PointXYZI> out;
    out.reserve(seg.object->size() + seg.rest->size() / rest_stride_ + 1);
    for (const auto & p : seg.object->points) {
      pcl::PointXYZI q;
      q.x = p.x; q.y = p.y; q.z = p.z; q.intensity = 1.0f;
      out.push_back(q);
    }
    for (size_t i = 0; i < seg.rest->size(); i += rest_stride_) {
      const auto & p = seg.rest->points[i];
      pcl::PointXYZI q;
      q.x = p.x; q.y = p.y; q.z = p.z; q.intensity = 0.0f;
      out.push_back(q);
    }
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(out, msg);
    msg.header = img->header;
    pub_->publish(msg);
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    if (!seg.reason.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "no object: %s (%zu points in view)",
        seg.reason.c_str(), cloud->size());
    } else {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000,
        "object %zu points, %d clusters, table %s, %.1f ms", seg.object->size(), seg.clusters,
        seg.plane_found ? "found" : "not found", ms);
    }
  }

  mac::DepthCloudParams dp_;
  mac::TabletopParams tp_;
  int rest_stride_{4};
  double min_period_{0.1};
  double last_time_{0.0};
  std::mutex mutex_;
  mac::Intrinsics K_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
};

}  // namespace mac_registration

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mac_registration::TabletopObjectCloudNode>());
  rclcpp::shutdown();
  return 0;
}
