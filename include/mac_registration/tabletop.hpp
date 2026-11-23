// Copyright 2026 Dun Yan Ong. Licensed under the Apache License, Version 2.0.
//
// Tabletop object segmentation: the GPU-free stand-in for the team's YOLO/SAM chain.
//
//   crop box  ->  remove the large planes (table, and the wall behind it)  ->  points on the
//   camera's side of them  ->  Euclidean clusters  ->  keep the cluster nearest the optical axis
//
// Why more than one plane: a camera looking along a shelf sees more wall than table, so "the
// largest plane" flips between the two from frame to frame, and whenever the wall wins, the
// table stays in and merges with the object.
//
// Deterministic (seeded RANSAC): the same frame always gives the same object. The depth range
// (z) is already applied by depth_to_cloud.
#pragma once

#include <string>

#include "mac_registration/frontend.hpp"

namespace mac
{

struct TabletopParams
{
  double roi_min_x{-0.4}, roi_max_x{0.4};  ///< [m] camera frame: x right
  double roi_min_y{-0.4}, roi_max_y{0.4};  ///< [m] camera frame: y down
  double plane_distance{0.01};             ///< points this close to the plane are table
  int plane_iterations{300};
  double plane_min_share{0.15};            ///< each plane must hold this share of the crop
  int max_planes{3};                       ///< remove up to this many planes (1 = table only)
  double cluster_tolerance{0.01};
  int min_cluster_points{200};
  int max_cluster_points{50000};
  double work_voxel{0.003};                ///< thin the crop before segmentation; 0 = off
  unsigned int seed{42};
};

struct TabletopResult
{
  Cloud::Ptr object;   ///< chosen cluster (finite points)
  Cloud::Ptr rest;     ///< everything else inside the crop (table, other objects)
  bool plane_found{false};  ///< at least one large plane (table or wall) was removed
  int clusters{0};
  std::string reason;  ///< empty when an object was found
};

TabletopResult segment_tabletop(const Cloud::ConstPtr & cloud, const TabletopParams & p);

}  // namespace mac
