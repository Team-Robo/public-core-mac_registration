// Copyright 2026 Dun Yan Ong. Licensed under the Apache License, Version 2.0.
//
// The front-end: from two clouds to candidate correspondences, plus the cloud-level checks
// used after MAC (ICP polish, fitness).
//
//   raw cloud --point_resolution--> pr (mean nearest-neighbour spacing, as upstream)
//             --voxel_downsample--> evenly spaced points
//             --describe--------> normals + FPFH (33-bin shape histograms)
//   ref + live --match_features--> mutual nearest neighbours in FPFH space, capped
#pragma once

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>

#include <string>
#include <vector>

#include "mac_registration/mac_solver.hpp"

namespace mac
{

using PointT = pcl::PointXYZ;
using Cloud = pcl::PointCloud<PointT>;
using FeatureT = pcl::FPFHSignature33;
using FeatureCloud = pcl::PointCloud<FeatureT>;

struct FrontendParams
{
  double voxel_size{0.005};        ///< [m] the master scale knob
  double normal_radius{0.0};       ///< [m]; 0 -> 2 x voxel
  double fpfh_radius{0.0};         ///< [m]; 0 -> 5 x voxel
  std::string keypoints{"none"};   ///< "none": all voxel points; "uniform": one per 2 x voxel cell
  int max_keypoints{2000};         ///< random cap on described points
  bool mutual_matching{true};
  int max_correspondences{1000};   ///< random cap (MAC time grows fast with this, see docs/details.md section 1.5)
  unsigned int random_seed{42};
  int num_threads{4};

  double normal_r() const {return normal_radius > 0.0 ? normal_radius : 2.0 * voxel_size;}
  double fpfh_r() const {return fpfh_radius > 0.0 ? fpfh_radius : 5.0 * voxel_size;}
};

/// Mean distance from a point to its nearest neighbour: the same quantity as upstream
/// MeshResolution_mr_compute, estimated on at most `max_samples` random query points.
double point_resolution(const Cloud::ConstPtr & cloud, int max_samples = 5000, unsigned int seed = 42);

Cloud::Ptr voxel_downsample(const Cloud::ConstPtr & in, double leaf);

/// Removes NaN/Inf points.
Cloud::Ptr finite_points(const Cloud & in);

/// Keypoints with FPFH descriptors: `points->points[i]` is described by `features->points[i]`.
struct Described
{
  Cloud::Ptr points;
  FeatureCloud::Ptr features;
  pcl::KdTreeFLANN<FeatureT>::Ptr feature_tree;  ///< nearest-neighbour search in FPFH space
  size_t size() const {return points ? points->size() : 0;}
};

/// Normals face the sensor at the camera-frame origin.
Described describe(const Cloud::ConstPtr & voxelized, const FrontendParams & p);

/// Correspondences reference -> live (src = reference point, des = live point).
std::vector<Correspondence> match_features(
  const Described & ref, const Described & live, const FrontendParams & p);

struct FitStats
{
  double fitness{0.0};  ///< share of transformed reference points with a live point within radius
  double rmse{0.0};     ///< RMS distance of those matched points [m]
  int matched{0};
};

FitStats evaluate_fit(
  const Cloud & reference, const pcl::KdTreeFLANN<PointT> & live_tree, const Eigen::Matrix4d & T,
  double radius);

struct IcpResult
{
  bool converged{false};
  Eigen::Matrix4d T{Eigen::Matrix4d::Identity()};
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/// Point-to-point ICP of `reference` onto `live`, starting from `init`.
IcpResult refine_icp(
  const Cloud::ConstPtr & reference, const Cloud::ConstPtr & live, const Eigen::Matrix4d & init,
  double max_corr_dist, int max_iterations);

/// Counts correspondences whose residual |T src - des| is below `tau`, and their mean residual.
void correspondence_residuals(
  const std::vector<Correspondence> & corr, const Eigen::Matrix4d & T, double tau, int & inliers,
  double & mean_residual);

}  // namespace mac
