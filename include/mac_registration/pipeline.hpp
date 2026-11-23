// Copyright 2026 Dun Yan Ong. Licensed under the Apache License, Version 2.0.
//
// One registration of a live cloud against a captured reference, end to end:
//   preprocess (pr, voxel) -> FPFH -> mutual matching -> MAC -> ICP polish -> gate.
#pragma once

#include <Eigen/Core>

#include <string>

#include "mac_registration/frontend.hpp"
#include "mac_registration/mac_solver.hpp"

namespace mac
{

/// The quality gate (docs/details.md section 2.4). Every check must pass.
struct GateParams
{
  int min_inliers{20};       ///< correspondences that agree with the pose
  double max_mae{0.010};     ///< [m] mean residual of those correspondences; also caps cloud RMSE
  double min_fitness{0.4};   ///< share of the reference that lands on the live surface
};

/// Returns "" when accepted, otherwise the first failed check.
inline std::string gate_reason(
  int inliers, double mae, double fitness, double cloud_rmse, const GateParams & g)
{
  if (inliers < g.min_inliers) {return "few_inliers";}
  if (mae > g.max_mae) {return "high_mae";}
  if (fitness < g.min_fitness) {return "low_fitness";}
  if (cloud_rmse > g.max_mae) {return "high_cloud_rmse";}
  return "";
}

struct PipelineParams
{
  FrontendParams frontend;
  Params mac;
  double point_resolution{0.0};  ///< [m]; 0 = measure on the raw input (mean NN distance)
  int pr_samples{5000};
  int min_correspondences{10};
  bool icp_enable{true};
  double icp_max_corr_dist{0.0};  ///< [m]; 0 -> 2 x voxel
  int icp_max_iterations{30};
  double fitness_radius{0.0};     ///< [m]; 0 -> voxel
  GateParams gate;

  double icp_dist() const {return icp_max_corr_dist > 0 ? icp_max_corr_dist : 2.0 * frontend.voxel_size;}
  double fit_radius() const {return fitness_radius > 0 ? fitness_radius : frontend.voxel_size;}
};

/// A captured reference with its cached features.
struct Reference
{
  Cloud::Ptr voxelized;
  Described described;
  Eigen::Vector3d centroid{Eigen::Vector3d::Zero()};  ///< origin of the object frame at capture
  double pr{0.0};
  bool valid() const {return voxelized && !voxelized->empty() && described.size() >= 3;}
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/// Builds a reference from a finite raw cloud (already filtered to the object points).
Reference make_reference(const Cloud::ConstPtr & raw, const PipelineParams & p);

struct FrameResult
{
  bool accepted{false};     ///< the gate passed
  std::string reason;       ///< "" when accepted
  Eigen::Matrix4d T{Eigen::Matrix4d::Identity()};  ///< final reference -> live
  Result mac;               ///< solver statistics
  FitStats fit;             ///< of the final pose
  int inliers{0};           ///< correspondences agreeing with the final pose
  double mae{0.0};          ///< [m] their mean residual
  double pr{0.0};           ///< point resolution used for MAC [m]
  size_t live_points{0};    ///< after voxel downsampling
  size_t correspondences{0};  ///< after mutual matching and the cap
  double preprocess_ms{0}, features_ms{0}, match_ms{0}, icp_ms{0};
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

FrameResult register_frame(const Reference & ref, const Cloud::ConstPtr & live_raw, const PipelineParams & p);

/// The published object pose: object frame at capture = (R = I, origin = c_ref), moved by T.
inline Eigen::Matrix4d object_pose(const Eigen::Matrix4d & T, const Eigen::Vector3d & c_ref)
{
  Eigen::Matrix4d P = Eigen::Matrix4d::Identity();
  P.topLeftCorner<3, 3>() = T.topLeftCorner<3, 3>();
  P.topRightCorner<3, 1>() = T.topLeftCorner<3, 3>() * c_ref + T.topRightCorner<3, 1>();
  return P;
}

}  // namespace mac
