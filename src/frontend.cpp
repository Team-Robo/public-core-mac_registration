// Copyright 2026 Dun Yan Ong. Licensed under the Apache License, Version 2.0.
#include "mac_registration/frontend.hpp"

#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/search/kdtree.h>

#include <algorithm>
#include <cmath>
#include <numeric>
#include <random>

namespace mac
{

double point_resolution(const Cloud::ConstPtr & cloud, int max_samples, unsigned int seed)
{
  if (!cloud || cloud->size() < 2) {
    return 0.0;
  }
  pcl::KdTreeFLANN<PointT> tree;
  tree.setInputCloud(cloud);
  std::vector<int> queries(cloud->size());
  std::iota(queries.begin(), queries.end(), 0);
  if (max_samples > 0 && static_cast<int>(queries.size()) > max_samples) {
    std::mt19937 rng(seed);
    std::shuffle(queries.begin(), queries.end(), rng);
    queries.resize(static_cast<size_t>(max_samples));
  }
  std::vector<int> idx(2);
  std::vector<float> d2(2);
  double sum = 0.0;
  int count = 0;
  for (int q : queries) {
    if (tree.nearestKSearch(cloud->points[q], 2, idx, d2) == 2) {
      sum += std::sqrt(static_cast<double>(d2[1]));
      ++count;
    }
  }
  return count > 0 ? sum / count : 0.0;
}

Cloud::Ptr voxel_downsample(const Cloud::ConstPtr & in, double leaf)
{
  Cloud::Ptr out(new Cloud);
  if (!in || in->empty()) {
    return out;
  }
  pcl::VoxelGrid<PointT> grid;
  grid.setInputCloud(in);
  const float l = static_cast<float>(leaf);
  grid.setLeafSize(l, l, l);
  grid.filter(*out);
  return out;
}

Cloud::Ptr finite_points(const Cloud & in)
{
  Cloud::Ptr out(new Cloud);
  out->reserve(in.size());
  for (const PointT & p : in.points) {
    if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)) {
      out->push_back(p);
    }
  }
  out->header = in.header;
  return out;
}

Described describe(const Cloud::ConstPtr & voxelized, const FrontendParams & p)
{
  Described d;
  d.points.reset(new Cloud);
  d.features.reset(new FeatureCloud);
  d.feature_tree.reset(new pcl::KdTreeFLANN<FeatureT>);
  if (!voxelized || voxelized->size() < 10) {
    return d;
  }
  const int threads = std::max(1, p.num_threads);
  pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
  tree->setInputCloud(voxelized);

  // Normals on the whole voxelized surface; flipped towards the sensor (the origin of the
  // camera frame unless the caller moved it).
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  pcl::NormalEstimationOMP<PointT, pcl::Normal> ne;
  ne.setNumberOfThreads(threads);
  ne.setInputCloud(voxelized);
  ne.setSearchMethod(tree);
  ne.setRadiusSearch(p.normal_r());
  ne.setViewPoint(0.0f, 0.0f, 0.0f);
  ne.compute(*normals);

  // Which points get a descriptor.
  std::vector<int> keys;
  if (p.keypoints == "uniform") {
    pcl::UniformSampling<PointT> us;
    us.setInputCloud(voxelized);
    us.setRadiusSearch(2.0 * p.voxel_size);
    Cloud sampled;
    us.filter(sampled);
    // UniformSampling returns points, not indices: map each back to its nearest voxel point.
    std::vector<int> nn(1);
    std::vector<float> d2(1);
    for (const PointT & q : sampled.points) {
      if (tree->nearestKSearch(q, 1, nn, d2) == 1) {
        keys.push_back(nn[0]);
      }
    }
    std::sort(keys.begin(), keys.end());
    keys.erase(std::unique(keys.begin(), keys.end()), keys.end());
  } else {
    keys.resize(voxelized->size());
    std::iota(keys.begin(), keys.end(), 0);
  }
  // Drop points whose normal could not be estimated.
  keys.erase(
    std::remove_if(
      keys.begin(), keys.end(),
      [&](int i) {
        const pcl::Normal & n = normals->points[i];
        return !std::isfinite(n.normal_x) || !std::isfinite(n.normal_y) || !std::isfinite(n.normal_z);
      }),
    keys.end());
  if (p.max_keypoints > 0 && static_cast<int>(keys.size()) > p.max_keypoints) {
    std::mt19937 rng(p.random_seed);
    std::shuffle(keys.begin(), keys.end(), rng);
    keys.resize(static_cast<size_t>(p.max_keypoints));
    std::sort(keys.begin(), keys.end());
  }
  if (keys.size() < 3) {
    return d;
  }

  // FPFH of the keypoints, with the whole voxelized cloud as search surface.
  Cloud::Ptr key_cloud(new Cloud);
  key_cloud->reserve(keys.size());
  for (int i : keys) {
    key_cloud->push_back(voxelized->points[i]);
  }
  // FPFHEstimation needs normals for the search surface, not for the keypoints.
  FeatureCloud::Ptr feats(new FeatureCloud);
  pcl::FPFHEstimationOMP<PointT, pcl::Normal, FeatureT> fpfh;
  fpfh.setNumberOfThreads(threads);
  fpfh.setInputCloud(key_cloud);
  fpfh.setSearchSurface(voxelized);
  fpfh.setInputNormals(normals);
  fpfh.setSearchMethod(tree);
  fpfh.setRadiusSearch(p.fpfh_r());
  fpfh.compute(*feats);

  // Keep keypoints with a finite, non-empty histogram.
  for (size_t k = 0; k < feats->size(); ++k) {
    const FeatureT & f = feats->points[k];
    bool finite = true;
    float sum = 0.0f;
    for (float v : f.histogram) {
      if (!std::isfinite(v)) {
        finite = false;
        break;
      }
      sum += v;
    }
    if (finite && sum > 0.0f) {
      d.points->push_back(key_cloud->points[k]);
      d.features->push_back(f);
    }
  }
  if (!d.features->empty()) {
    d.feature_tree->setInputCloud(d.features);
  }
  return d;
}

std::vector<Correspondence> match_features(
  const Described & ref, const Described & live, const FrontendParams & p)
{
  std::vector<Correspondence> out;
  if (ref.size() < 3 || live.size() < 3) {
    return out;
  }
  // Reference -> live.
  const int nr = static_cast<int>(ref.size());
  const int nl = static_cast<int>(live.size());
  std::vector<int> fwd(nr, -1);
  std::vector<float> fwd_d2(nr, 0.0f);
  std::vector<int> nn(1);
  std::vector<float> d2(1);
  for (int i = 0; i < nr; ++i) {
    if (live.feature_tree->nearestKSearch(ref.features->points[i], 1, nn, d2) == 1) {
      fwd[i] = nn[0];
      fwd_d2[i] = d2[0];
    }
  }
  // Live -> reference, only needed for the mutual check.
  std::vector<int> bwd;
  if (p.mutual_matching) {
    bwd.assign(nl, -1);
    for (int j = 0; j < nl; ++j) {
      if (ref.feature_tree->nearestKSearch(live.features->points[j], 1, nn, d2) == 1) {
        bwd[j] = nn[0];
      }
    }
  }
  for (int i = 0; i < nr; ++i) {
    const int j = fwd[i];
    if (j < 0) {
      continue;
    }
    if (p.mutual_matching && bwd[j] != i) {
      continue;
    }
    Correspondence c;
    c.src = ref.points->points[i].getVector3fMap();
    c.des = live.points->points[j].getVector3fMap();
    c.src_index = i;
    c.des_index = j;
    c.score = 1.0f - std::sqrt(fwd_d2[i]);  // same convention as upstream feature_matching
    out.push_back(c);
  }
  if (p.max_correspondences > 0 && static_cast<int>(out.size()) > p.max_correspondences) {
    std::mt19937 rng(p.random_seed);
    std::shuffle(out.begin(), out.end(), rng);
    out.resize(static_cast<size_t>(p.max_correspondences));
    std::sort(
      out.begin(), out.end(),
      [](const Correspondence & a, const Correspondence & b) {return a.src_index < b.src_index;});
  }
  return out;
}

FitStats evaluate_fit(
  const Cloud & reference, const pcl::KdTreeFLANN<PointT> & live_tree, const Eigen::Matrix4d & T,
  double radius)
{
  FitStats f;
  if (reference.empty()) {
    return f;
  }
  const Eigen::Matrix3f R = T.topLeftCorner<3, 3>().cast<float>();
  const Eigen::Vector3f t = T.topRightCorner<3, 1>().cast<float>();
  const float r2 = static_cast<float>(radius * radius);
  std::vector<int> nn(1);
  std::vector<float> d2(1);
  double sum = 0.0;
  for (const PointT & p : reference.points) {
    PointT q;
    q.getVector3fMap() = R * p.getVector3fMap() + t;
    if (live_tree.nearestKSearch(q, 1, nn, d2) == 1 && d2[0] <= r2) {
      ++f.matched;
      sum += d2[0];
    }
  }
  f.fitness = static_cast<double>(f.matched) / static_cast<double>(reference.size());
  f.rmse = f.matched > 0 ? std::sqrt(sum / f.matched) : 0.0;
  return f;
}

IcpResult refine_icp(
  const Cloud::ConstPtr & reference, const Cloud::ConstPtr & live, const Eigen::Matrix4d & init,
  double max_corr_dist, int max_iterations)
{
  IcpResult r;
  r.T = init;
  if (!reference || !live || reference->size() < 3 || live->size() < 3) {
    return r;
  }
  pcl::IterativeClosestPoint<PointT, PointT> icp;
  icp.setInputSource(reference);
  icp.setInputTarget(live);
  icp.setMaxCorrespondenceDistance(max_corr_dist);
  icp.setMaximumIterations(max_iterations);
  icp.setTransformationEpsilon(1e-9);
  icp.setEuclideanFitnessEpsilon(1e-10);
  Cloud aligned;
  icp.align(aligned, init.cast<float>());
  r.converged = icp.hasConverged();
  if (r.converged) {
    r.T = icp.getFinalTransformation().cast<double>();
  }
  return r;
}

void correspondence_residuals(
  const std::vector<Correspondence> & corr, const Eigen::Matrix4d & T, double tau, int & inliers,
  double & mean_residual)
{
  inliers = 0;
  double sum = 0.0;
  const Eigen::Matrix3d R = T.topLeftCorner<3, 3>();
  const Eigen::Vector3d t = T.topRightCorner<3, 1>();
  for (const Correspondence & c : corr) {
    const double r = (R * c.src.cast<double>() + t - c.des.cast<double>()).norm();
    if (r < tau) {
      ++inliers;
      sum += r;
    }
  }
  mean_residual = inliers > 0 ? sum / inliers : 0.0;
}

}  // namespace mac
