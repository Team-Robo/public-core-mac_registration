// Copyright 2026 Dun Yan Ong. Licensed under the Apache License, Version 2.0.
#include "mac_registration/tabletop.hpp"

#include <pcl/ModelCoefficients.h>
#include <pcl/common/centroid.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <cmath>
#include <cstdlib>
#include <limits>

namespace mac
{

TabletopResult segment_tabletop(const Cloud::ConstPtr & cloud, const TabletopParams & p)
{
  TabletopResult r;
  r.object.reset(new Cloud);
  r.rest.reset(new Cloud);
  if (!cloud || cloud->empty()) {
    r.reason = "empty_input";
    return r;
  }

  // 1. Crop box in the camera frame.
  Cloud::Ptr crop(new Cloud);
  crop->reserve(cloud->size());
  for (const PointT & q : cloud->points) {
    if (!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z)) {continue;}
    if (q.x < p.roi_min_x || q.x > p.roi_max_x || q.y < p.roi_min_y || q.y > p.roi_max_y) {continue;}
    crop->push_back(q);
  }
  if (crop->size() < static_cast<size_t>(p.min_cluster_points)) {
    r.reason = "too_few_points_in_crop";
    *r.rest = *crop;
    return r;
  }
  Cloud::Ptr work = crop;
  if (p.work_voxel > 0.0) {
    work = voxel_downsample(crop, p.work_voxel);
  }

  // 2. Large planes (RANSAC, seeded so runs repeat): the table, then a wall if one is in view.
  //    Each must hold plane_min_share of the whole crop, so an object's flat face never counts.
  std::srand(p.seed);
  pcl::SACSegmentation<PointT> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(p.plane_distance);
  seg.setMaxIterations(p.plane_iterations);
  Cloud::Ptr above(new Cloud(*work));  // no plane in view: cluster everything
  Cloud::Ptr table(new Cloud);
  const double min_plane_points = p.plane_min_share * static_cast<double>(work->size());
  for (int k = 0; k < p.max_planes && above->size() >= min_plane_points; ++k) {
    seg.setInputCloud(above);
    pcl::PointIndices plane_idx;
    pcl::ModelCoefficients coeff;
    seg.segment(plane_idx, coeff);
    if (coeff.values.size() != 4 || plane_idx.indices.size() < min_plane_points) {break;}
    Eigen::Vector4f pl(coeff.values[0], coeff.values[1], coeff.values[2], coeff.values[3]);
    pl /= pl.head<3>().norm();
    // Orient the normal towards the camera (the origin): then "in front of it" = positive side.
    if (pl[3] < 0) {pl = -pl;}
    r.plane_found = true;
    Cloud::Ptr front(new Cloud);
    front->reserve(above->size());
    for (const PointT & q : above->points) {
      const float d = pl[0] * q.x + pl[1] * q.y + pl[2] * q.z + pl[3];
      (d > static_cast<float>(p.plane_distance) ? front : table)->push_back(q);
    }
    above = front;
  }

  // 3. Clusters above the table.
  std::vector<pcl::PointIndices> clusters;
  if (above->size() >= static_cast<size_t>(p.min_cluster_points)) {
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(above);
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(p.cluster_tolerance);
    ec.setMinClusterSize(p.min_cluster_points);
    ec.setMaxClusterSize(p.max_cluster_points);
    ec.setSearchMethod(tree);
    ec.setInputCloud(above);
    ec.extract(clusters);
  }
  r.clusters = static_cast<int>(clusters.size());

  // 4. Keep the cluster whose centroid is closest to the optical axis (angle from +z).
  int best = -1;
  double best_angle = std::numeric_limits<double>::max();
  for (size_t k = 0; k < clusters.size(); ++k) {
    Eigen::Vector4f c;
    pcl::compute3DCentroid(*above, clusters[k].indices, c);
    const double angle = std::atan2(std::hypot(c.x(), c.y()), c.z());
    if (angle < best_angle) {
      best_angle = angle;
      best = static_cast<int>(k);
    }
  }
  std::vector<char> is_object(above->size(), 0);
  if (best >= 0) {
    for (int i : clusters[best].indices) {
      is_object[i] = 1;
    }
  }
  r.object->reserve(best >= 0 ? clusters[best].indices.size() : 0);
  for (size_t i = 0; i < above->size(); ++i) {
    (is_object[i] ? r.object : r.rest)->push_back(above->points[i]);
  }
  for (const PointT & q : table->points) {
    r.rest->push_back(q);
  }
  if (best < 0) {
    r.reason = "no_object_cluster";
  }
  return r;
}

}  // namespace mac
