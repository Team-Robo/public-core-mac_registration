// Copyright 2026 Dun Yan Ong. Licensed under the Apache License, Version 2.0.
//
// mac::Solver: a thin, side-effect-free host for the upstream MAC algorithm.
//
// It runs the same steps, in the same order, as upstream `registration()` (overload 2,
// third_party/MAC/Linux/registration.cpp) and calls the upstream functions unchanged:
//   Graph_construction -> clustering coefficient + OTSU -> optional graph reduction ->
//   igraph maximal cliques -> find_largest_clique_of_node -> evaluation_trans (MAE) per
//   clique -> post_refinement.
// What it removes: console output, est.txt / corr.txt writes, viewer windows, mkdir/exit,
// the hard-coded 12 OpenMP threads. What it adds: the transform is returned, every knob is a
// parameter, explosion guards, deterministic tie-breaking, timing per stage.
//
// This header deliberately does not include the upstream Eva.h (global `using namespace std`
// and macros); only src/mac_solver.cpp does. Upstream also implements node-guided selection,
// instance-equal SVD and no normal-consistency filter only, so those paper knobs are fixed.
#pragma once

#include <Eigen/Core>

#include <array>
#include <string>
#include <vector>

namespace mac
{

/// One candidate pairing: point `src` of the REFERENCE cloud <-> point `des` of the LIVE cloud.
/// The solver estimates T such that des ~= T * src.
struct Correspondence
{
  Eigen::Vector3f src{Eigen::Vector3f::Zero()};
  Eigen::Vector3f des{Eigen::Vector3f::Zero()};
  int src_index{-1};
  int des_index{-1};
  float score{0.0f};  ///< descriptor similarity; informational, the solver does not use it
};

enum class Metric { MAE, MSE, Inlier };

const char * metric_name(Metric m);
bool parse_metric(const std::string & text, Metric & out);

struct Params
{
  // Paper knobs (defaults = the paper's best configuration, Table 5 row 6).
  bool use_sog{true};                ///< second-order compatibility graph (FOG if false)
  bool clique_ranking{false};        ///< keep only the top_k heaviest selected cliques
  int top_k{100};
  Metric metric{Metric::MAE};
  double t_cmp{0.99};                ///< edge kept if exp(-d^2 / 2 d_cmp^2) >= t_cmp
  double d_cmp_pr{10.0};             ///< d_cmp in units of point resolution pr
  double inlier_thresh_pr{10.0};     ///< scoring threshold tau in units of pr
  int min_clique_size{3};
  int refinement_iterations{20};

  // Engineering knobs.
  int num_threads{4};                ///< OpenMP + Eigen threads used inside solve()
  long max_graph_edges{150000};      ///< above this, t_cmp is raised, then pairs are halved once
  long max_cliques{20000};           ///< stop igraph's enumeration after this many cliques
};

struct Result
{
  bool ok{false};
  std::string reason;                ///< empty when ok
  Eigen::Matrix4d T{Eigen::Matrix4d::Identity()};  ///< maps reference points onto live points
  double mac_score{0.0};             ///< upstream score of the refined pose (higher = better)
  double inlier_thresh_m{0.0};       ///< tau [m]
  long graph_edges{0};
  long clique_count{0};
  int selected_cliques{0};           ///< hypotheses evaluated (after node-guided selection)
  double t_cmp_used{0.99};
  /// [0] graph + clustering coefficient, [1] clique enumeration, [2] node-guided selection,
  /// [3] hypothesis evaluation + refinement. Milliseconds.
  std::array<double, 4> stage_ms{{0.0, 0.0, 0.0, 0.0}};
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class Solver
{
public:
  /// @param corr  candidate pairs (reference -> live)
  /// @param pr    point resolution [m]: mean nearest-neighbour spacing of the clouds
  /// Thread-safe: calls are serialised internally (upstream keeps global state).
  Result solve(const std::vector<Correspondence> & corr, double pr, const Params & params) const;
};

}  // namespace mac
