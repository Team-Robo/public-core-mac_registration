// Copyright 2026 Dun Yan Ong. Licensed under the Apache License, Version 2.0.
#include "mac_registration/depth_cloud.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace mac
{

Cloud::Ptr depth_to_cloud(
  const uint8_t * data, int width, int height, int step, const std::string & encoding,
  const Intrinsics & K, const DepthCloudParams & p)
{
  Cloud::Ptr cloud(new Cloud);
  if (!data || !K.valid() || width <= 0 || height <= 0) {
    return cloud;
  }
  const bool is16 = (encoding == "16UC1" || encoding == "mono16");
  const bool is32f = (encoding == "32FC1");
  if (!is16 && !is32f) {
    return cloud;
  }
  const int s = std::max(1, p.stride);
  cloud->reserve(static_cast<size_t>(width / s + 1) * static_cast<size_t>(height / s + 1));
  const double inv_fx = 1.0 / K.fx;
  const double inv_fy = 1.0 / K.fy;
  for (int v = 0; v < height; v += s) {
    const uint8_t * row = data + static_cast<size_t>(v) * static_cast<size_t>(step);
    for (int u = 0; u < width; u += s) {
      double z;
      if (is16) {
        uint16_t raw;
        std::memcpy(&raw, row + 2 * u, sizeof(raw));
        if (raw == 0) {
          continue;
        }
        z = raw * p.depth_scale;
      } else {
        float raw;
        std::memcpy(&raw, row + 4 * u, sizeof(raw));
        if (!std::isfinite(raw) || raw <= 0.0f) {
          continue;
        }
        z = raw;
      }
      if (z < p.z_min || z > p.z_max) {
        continue;
      }
      PointT pt;
      pt.x = static_cast<float>((u - K.cx) * z * inv_fx);
      pt.y = static_cast<float>((v - K.cy) * z * inv_fy);
      pt.z = static_cast<float>(z);
      cloud->push_back(pt);
    }
  }
  cloud->width = static_cast<uint32_t>(cloud->size());
  cloud->height = 1;
  cloud->is_dense = true;
  return cloud;
}

}  // namespace mac
