// Copyright 2026 Dun Yan Ong. Licensed under the Apache License, Version 2.0.
//
// Aligned depth image + pinhole intrinsics -> 3-D points in the image's optical frame.
// Uses only K (fx, fy, cx, cy), exactly like object_pointcloud and depth_image_proc, so clouds
// built here match the team's clouds. ROS-free.
#pragma once

#include <cstdint>
#include <string>

#include "mac_registration/frontend.hpp"

namespace mac
{

struct Intrinsics
{
  double fx{0}, fy{0}, cx{0}, cy{0};
  int width{0}, height{0};
  bool valid() const {return fx > 0 && fy > 0 && width > 0 && height > 0;}
};

struct DepthCloudParams
{
  int stride{2};          ///< use every n-th pixel in both directions
  double z_min{0.3};      ///< [m]
  double z_max{1.2};      ///< [m]
  double depth_scale{0.001};  ///< metres per unit for 16UC1 (millimetres)
};

/// Back-projects a depth image. `encoding` is "16UC1"/"mono16" (units of depth_scale) or "32FC1"
/// (metres). Invalid (zero / NaN / out-of-range) pixels are skipped. `step` is bytes per row.
Cloud::Ptr depth_to_cloud(
  const uint8_t * data, int width, int height, int step, const std::string & encoding,
  const Intrinsics & K, const DepthCloudParams & p);

}  // namespace mac
