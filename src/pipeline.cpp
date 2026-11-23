// Copyright 2026 Dun Yan Ong. Licensed under the Apache License, Version 2.0.
#include "mac_registration/pipeline.hpp"

#include <pcl/common/centroid.h>

#include <algorithm>
#include <chrono>

namespace mac
{
namespace
{
using Clock = std::chrono::steady_clock;
double ms_since(Clock::time_point t0)
{
  return std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
}
}  // namespace

Reference make_reference(const Cloud::ConstPtr & raw, const PipelineParams & p)
{
  Reference r;
  r.pr = p.point_resolution > 0.0 ?
    p.point_resolution : point_resolution(raw, p.pr_samples, p.frontend.random_seed);
  r.voxelized = voxel_downsample(raw, p.frontend.voxel_size);
  r.described = describe(r.voxelized, p.frontend);
  if (!r.voxelized->empty()) {
    Eigen::Vector4d c;
    pcl::compute3DCentroid(*r.voxelized, c);
    r.centroid = c.head<3>();
  }
  return r;
}

FrameResult register_frame(const Reference & ref, const Cloud::ConstPtr & live_raw, const PipelineParams & p)
{
  FrameResult f;
  if (!ref.valid()) {
    f.reason = "no_reference";
    return f;
  }
  if (!live_raw || live_raw->size() < 10) {
    f.reason = "empty_input";
    return f;
  }

  // 1. Preprocess: point resolution on the raw input, then the voxel grid.
  auto t = Clock::now();
  const double live_pr = p.point_resolution > 0.0 ?
    p.point_resolution : point_resolution(live_raw, p.pr_samples, p.frontend.random_seed);
  f.pr = (ref.pr > 0.0 && live_pr > 0.0) ? 0.5 * (ref.pr + live_pr) : std::max(ref.pr, live_pr);
  const Cloud::Ptr live = voxel_downsample(live_raw, p.frontend.voxel_size);
  f.live_points = live->size();
  f.preprocess_ms = ms_since(t);

  // 2. Features.
  t = Clock::now();
  const Described live_described = describe(live, p.frontend);
  f.features_ms = ms_since(t);

  // 3. Matching.
  t = Clock::now();
  const std::vector<Correspondence> corr = match_features(ref.described, live_described, p.frontend);
  f.correspondences = corr.size();
  f.match_ms = ms_since(t);
  if (static_cast<int>(corr.size()) < std::max(3, p.min_correspondences)) {
    f.reason = "too_few_correspondences";
    return f;
  }

  // 4. MAC.
  Solver solver;
  f.mac = solver.solve(corr, f.pr, p.mac);
  if (!f.mac.ok) {
    f.reason = f.mac.reason.empty() ? "mac_failed" : f.mac.reason;
    return f;
  }
  f.T = f.mac.T;

  // 5. Cloud-level check and ICP polish (kept only if it does not lower the fitness).
  t = Clock::now();
  pcl::KdTreeFLANN<PointT> live_tree;
  live_tree.setInputCloud(live);
  f.fit = evaluate_fit(*ref.voxelized, live_tree, f.T, p.fit_radius());
  if (p.icp_enable) {
    const IcpResult icp = refine_icp(ref.voxelized, live, f.T, p.icp_dist(), p.icp_max_iterations);
    if (icp.converged && icp.T.allFinite()) {
      const FitStats fit_icp = evaluate_fit(*ref.voxelized, live_tree, icp.T, p.fit_radius());
      if (fit_icp.fitness >= f.fit.fitness) {
        f.T = icp.T;
        f.fit = fit_icp;
      }
    }
  }
  f.icp_ms = ms_since(t);

  // 6. Gate, on the final pose.
  correspondence_residuals(corr, f.T, f.mac.inlier_thresh_m, f.inliers, f.mae);
  f.reason = gate_reason(f.inliers, f.mae, f.fit.fitness, f.fit.rmse, p.gate);
  f.accepted = f.reason.empty();
  return f;
}

}  // namespace mac
