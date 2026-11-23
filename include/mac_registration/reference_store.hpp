// Copyright 2026 Dun Yan Ong. Licensed under the Apache License, Version 2.0.
//
// Reference cache and capture rule ("competition quick mode": nobody prepares a .pcd).
//
//   <reference_dir>/<label>.pcd       the raw object points of the captured frame (binary PCD)
//   <reference_dir>/<label>.meta.txt  frame_id, stamp, point count, voxel, pr and centroid at capture
//
// The raw snapshot is stored (not the voxelized one) so a later change of voxel_size simply
// rebuilds the features from it.
#pragma once

#include <Eigen/Core>

#include <string>

#include "mac_registration/frontend.hpp"

namespace mac
{

struct ReferenceMeta
{
  std::string label;
  std::string frame_id;
  double stamp_sec{0.0};
  size_t points{0};
  double voxel_size{0.0};
  double point_resolution{0.0};
  Eigen::Vector3d centroid{Eigen::Vector3d::Zero()};
};

class ReferenceStore
{
public:
  ReferenceStore() = default;
  ReferenceStore(const std::string & directory, const std::string & label);

  std::string pcd_path() const;
  std::string meta_path() const;
  bool exists() const;
  bool load(Cloud::Ptr & raw, ReferenceMeta & meta, std::string * error = nullptr) const;
  bool save(const Cloud & raw, const ReferenceMeta & meta, std::string * error = nullptr) const;
  bool remove() const;

private:
  std::string dir_;
  std::string label_;
};

/// Expands a leading "~/" with $HOME.
std::string expand_user(const std::string & path);

struct CaptureRule
{
  int min_points{200};
  double min_extent{0.03};       ///< [m] largest side of the bounding box must reach this
  double max_extent{1.5};        ///< [m] and stay below this
  int stability_frames{3};       ///< consecutive frames that must agree
  double centroid_tolerance{0.005};  ///< [m] centroid motion between those frames
  double count_tolerance{0.2};   ///< relative change of the point count between frames
  /// Acceptable frames ignored after start-up: a D455 that has just started streams depth that
  /// is still filling in (about 20 % fewer points), and those frames are "stable" too.
  int warmup_frames{15};
};

/// Tracks whether the incoming object cloud has been still for `stability_frames` frames.
class CaptureGate
{
public:
  explicit CaptureGate(const CaptureRule & rule = CaptureRule()) : rule_(rule) {}
  void set_rule(const CaptureRule & rule) {rule_ = rule; fed_ = 0; reset();}
  /// Returns "" when the frame is acceptable; otherwise why it is not (and the streak resets).
  std::string check_frame(const Cloud & raw) const;
  /// Feeds an acceptable frame; returns true once the streak reaches stability_frames and the
  /// warm-up is over.
  bool update(const Cloud & raw);
  int streak() const {return streak_;}
  bool warming_up() const {return fed_ <= rule_.warmup_frames;}
  /// Restarts the streak; the warm-up stays done (it counts frames since start-up).
  void reset() {streak_ = 0; has_last_ = false;}

private:
  CaptureRule rule_;
  int fed_{0};
  int streak_{0};
  bool has_last_{false};
  Eigen::Vector3d last_centroid_{Eigen::Vector3d::Zero()};
  size_t last_count_{0};
};

}  // namespace mac
