// Copyright 2026 Dun Yan Ong. Licensed under the Apache License, Version 2.0.
#include "mac_registration/reference_store.hpp"

#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>

namespace mac
{

std::string expand_user(const std::string & path)
{
  if (path.size() >= 2 && path[0] == '~' && path[1] == '/') {
    const char * home = std::getenv("HOME");
    if (home) {
      return std::string(home) + path.substr(1);
    }
  }
  return path;
}

ReferenceStore::ReferenceStore(const std::string & directory, const std::string & label)
: dir_(expand_user(directory)), label_(label)
{
}

std::string ReferenceStore::pcd_path() const {return dir_ + "/" + label_ + ".pcd";}
std::string ReferenceStore::meta_path() const {return dir_ + "/" + label_ + ".meta.txt";}

bool ReferenceStore::exists() const
{
  std::error_code ec;
  return std::filesystem::exists(pcd_path(), ec);
}

bool ReferenceStore::save(const Cloud & raw, const ReferenceMeta & meta, std::string * error) const
{
  std::error_code ec;
  std::filesystem::create_directories(dir_, ec);
  if (ec) {
    if (error) {*error = "cannot create " + dir_ + ": " + ec.message();}
    return false;
  }
  Cloud copy = raw;
  copy.width = static_cast<uint32_t>(copy.size());
  copy.height = 1;
  if (pcl::io::savePCDFileBinary(pcd_path(), copy) != 0) {
    if (error) {*error = "cannot write " + pcd_path();}
    return false;
  }
  std::ofstream out(meta_path());
  out << std::setprecision(10);
  out << "label: " << meta.label << "\n";
  out << "frame_id: " << meta.frame_id << "\n";
  out << "stamp_sec: " << meta.stamp_sec << "\n";
  out << "points: " << meta.points << "\n";
  out << "voxel_size: " << meta.voxel_size << "\n";
  out << "point_resolution: " << meta.point_resolution << "\n";
  out << "centroid: " << meta.centroid.x() << " " << meta.centroid.y() << " " << meta.centroid.z() << "\n";
  if (!out) {
    if (error) {*error = "cannot write " + meta_path();}
    return false;
  }
  return true;
}

bool ReferenceStore::load(Cloud::Ptr & raw, ReferenceMeta & meta, std::string * error) const
{
  raw.reset(new Cloud);
  if (!exists()) {
    if (error) {*error = "no file " + pcd_path();}
    return false;
  }
  if (pcl::io::loadPCDFile(pcd_path(), *raw) != 0 || raw->empty()) {
    if (error) {*error = "cannot read " + pcd_path();}
    return false;
  }
  meta = ReferenceMeta();
  meta.label = label_;
  std::ifstream in(meta_path());
  std::string line;
  while (std::getline(in, line)) {  // unknown keys are skipped
    const auto colon = line.find(':');
    if (colon == std::string::npos) {continue;}
    const std::string key = line.substr(0, colon);
    std::istringstream val(line.substr(colon + 1));
    if (key == "frame_id") {val >> meta.frame_id;}
    else if (key == "stamp_sec") {val >> meta.stamp_sec;}
    else if (key == "points") {val >> meta.points;}
    else if (key == "voxel_size") {val >> meta.voxel_size;}
    else if (key == "point_resolution") {val >> meta.point_resolution;}
    else if (key == "centroid") {val >> meta.centroid.x() >> meta.centroid.y() >> meta.centroid.z();}
  }
  return true;
}

bool ReferenceStore::remove() const
{
  std::error_code ec;
  const bool a = std::filesystem::remove(pcd_path(), ec);
  std::filesystem::remove(meta_path(), ec);
  return a;
}

std::string CaptureGate::check_frame(const Cloud & raw) const
{
  if (static_cast<int>(raw.size()) < rule_.min_points) {
    return "capture_too_few_points";
  }
  PointT lo, hi;
  pcl::getMinMax3D(raw, lo, hi);
  const double extent = std::max({hi.x - lo.x, hi.y - lo.y, hi.z - lo.z});
  if (extent < rule_.min_extent) {
    return "capture_too_small";
  }
  if (extent > rule_.max_extent) {
    return "capture_too_large";
  }
  return "";
}

bool CaptureGate::update(const Cloud & raw)
{
  Eigen::Vector4d c4 = Eigen::Vector4d::Zero();
  pcl::compute3DCentroid(raw, c4);
  const Eigen::Vector3d c = c4.head<3>();
  const size_t n = raw.size();
  bool consistent = false;
  if (has_last_) {
    const double dc = (c - last_centroid_).norm();
    const double dn = std::abs(static_cast<double>(n) - static_cast<double>(last_count_)) /
      std::max<double>(1.0, static_cast<double>(last_count_));
    consistent = dc <= rule_.centroid_tolerance && dn <= rule_.count_tolerance;
  }
  streak_ = consistent ? streak_ + 1 : 1;
  has_last_ = true;
  last_centroid_ = c;
  last_count_ = n;
  ++fed_;
  return !warming_up() && streak_ >= std::max(1, rule_.stability_frames);
}

}  // namespace mac
