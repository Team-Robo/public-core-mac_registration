// Copyright 2026 Dun Yan Ong. Licensed under the Apache License, Version 2.0.
//
// Host for the upstream MAC algorithm. Read this file side by side with upstream
// third_party/MAC/Linux/registration.cpp, function `registration(PointCloudPtr&, ...)`
// (overload 2, lines 1426-1936, igraph >= 0.10 branch): every numbered step below names the
// upstream lines it mirrors. Algorithm functions are called unchanged; only the glue is here.
#include "mac_registration/mac_solver.hpp"

#include <omp.h>

#include <algorithm>
#include <chrono>
#include <climits>
#include <cmath>
#include <exception>
#include <memory>
#include <mutex>
#include <numeric>

#include "mac_registration/igraph_raii.hpp"

// Upstream header: global `using namespace std;`, macros, PCL typedefs. Keep it last and
// keep it out of every header of this package.
#include "Eva.h"

// Defined in upstream registration.cpp (node-guided clique selection) but not declared in Eva.h.
void find_largest_clique_of_node(
  Eigen::MatrixXf & Graph, igraph_vector_int_list_t * cliques,
  vector<Corre_3DMatch> & correspondence, node_cliques * result, vector<int> & remain,
  int num_node, int est_num, string descriptor);

namespace mac
{

const char * metric_name(Metric m)
{
  switch (m) {
    case Metric::MAE: return "MAE";
    case Metric::MSE: return "MSE";
    case Metric::Inlier: return "inlier";
  }
  return "MAE";
}

bool parse_metric(const std::string & text, Metric & out)
{
  if (text == "MAE" || text == "mae") {out = Metric::MAE; return true;}
  if (text == "MSE" || text == "mse") {out = Metric::MSE; return true;}
  if (text == "inlier" || text == "inliers" || text == "count") {out = Metric::Inlier; return true;}
  return false;
}

namespace
{

std::mutex g_solve_mutex;  // upstream reads globals and Eigen's thread count is global
using Clock = std::chrono::steady_clock;

double ms_between(Clock::time_point a, Clock::time_point b)
{
  return std::chrono::duration<double, std::milli>(b - a).count();
}

struct CliqueSink
{
  igraph_vector_int_list_t * list{nullptr};
  long max{LONG_MAX};
  long count{0};
};

// Called by igraph for every maximal clique. The clique vector belongs to igraph, so we copy it.
igraph_error_t on_clique(const igraph_vector_int_t * clique, void * arg)
{
  auto * sink = static_cast<CliqueSink *>(arg);
  if (sink->count >= sink->max) {
    return IGRAPH_STOP;  // stop the enumeration without an error
  }
  const igraph_error_t rc = igraph_vector_int_list_push_back_copy(sink->list, clique);
  if (rc != IGRAPH_SUCCESS) {
    return rc;
  }
  ++sink->count;
  return IGRAPH_SUCCESS;
}

struct Clustering
{
  std::vector<double> factor;       // per node, original order (upstream cluster_factor_bac)
  std::vector<double> sorted_desc;  // upstream cluster_factor after sort(compare_vote_score)
  double average{0.0};
  double total{0.0};
  double otsu{0.0};
  double threshold{0.0};
};

// Steps 2-3: degree list + weighted clustering coefficient + OTSU (upstream lines 1455-1555).
// Same formula. Upstream parallelises the inner pair loop with an `omp critical` per triangle,
// which makes the sum order (and so the last bits) depend on thread timing; here each node is
// summed by one thread, so the result is deterministic.
Clustering clustering_coefficients(const Eigen::MatrixXf & G)
{
  const int n = static_cast<int>(G.rows());
  std::vector<std::vector<int>> nbrs(n);
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < n; ++j) {
      if (i != j && G(i, j)) {
        nbrs[i].push_back(j);
      }
    }
  }
  std::vector<double> wijk(n, 0.0);
#pragma omp parallel for schedule(dynamic, 8)
  for (int i = 0; i < n; ++i) {
    const std::vector<int> & nb = nbrs[i];
    const int m = static_cast<int>(nb.size());
    double w = 0.0;
    for (int j = 0; j < m; ++j) {
      const int a = nb[j];
      for (int k = j + 1; k < m; ++k) {
        const int b = nb[k];
        if (G(a, b)) {
          w += pow(G(i, a) * G(i, b) * G(a, b), 1.0 / 3);
        }
      }
    }
    wijk[i] = w;
  }

  Clustering c;
  c.factor.assign(n, 0.0);
  double sum_fenzi = 0.0;
  double sum_fenmu = 0.0;
  for (int i = 0; i < n; ++i) {
    const int degree = static_cast<int>(nbrs[i].size());
    if (degree > 1) {
      const double f1 = wijk[i];
      const double f2 = degree * (degree - 1) * 0.5;
      sum_fenzi += f1;
      sum_fenmu += f2;
      c.factor[i] = f1 / f2;
    }
  }
  c.average = n > 0 ? std::accumulate(c.factor.begin(), c.factor.end(), 0.0) / n : 0.0;
  c.total = sum_fenzi / sum_fenmu;  // NaN when no node has two neighbours, exactly as upstream
  c.sorted_desc = c.factor;
  std::sort(c.sorted_desc.begin(), c.sorted_desc.end(), std::greater<double>());

  c.otsu = 0.0;
  if (!c.sorted_desc.empty() && c.sorted_desc.front() != 0) {
    if (c.sorted_desc.front() > c.sorted_desc.back()) {
      Eigen::VectorXd values(n);
      for (int i = 0; i < n; ++i) {
        values[i] = c.sorted_desc[i];
      }
      c.otsu = OTSU_thresh(values);
    }
    // else: all coefficients equal. Upstream would divide by a zero histogram step here;
    // OTSU_thresh's own default for that case is 0, which we use.
  }
  c.threshold = min(c.otsu, min(c.average, c.total));
  return c;
}

void to_upstream(const std::vector<Correspondence> & in, vector<Corre_3DMatch> & out)
{
  out.clear();
  out.reserve(in.size());
  for (const Correspondence & c : in) {
    Corre_3DMatch u;
    u.src_index = c.src_index;
    u.des_index = c.des_index;
    u.src = pcl::PointXYZ(c.src.x(), c.src.y(), c.src.z());
    u.des = pcl::PointXYZ(c.des.x(), c.des.y(), c.des.z());
    u.src_norm.setZero();
    u.des_norm.setZero();
    u.covariance_src.setZero();
    u.covariance_des.setZero();
    u.centeroid_src.setZero();
    u.centeroid_des.setZero();
    u.score = c.score;
    u.inlier_weight = 0;
    out.push_back(u);
  }
}

}  // namespace

Result Solver::solve(const std::vector<Correspondence> & corr_in, double pr, const Params & p) const
{
  std::lock_guard<std::mutex> lock(g_solve_mutex);
  Result r;
  r.t_cmp_used = p.t_cmp;
  r.inlier_thresh_m = p.inlier_thresh_pr * pr;
  if (corr_in.size() < 3) {
    r.reason = "too_few_correspondences";
    return r;
  }
  if (!(pr > 0.0) || !std::isfinite(pr)) {
    r.reason = "bad_point_resolution";
    return r;
  }

  const int threads = std::max(1, p.num_threads);
  omp_set_num_threads(threads);  // upstream hard-codes 12; OpenMP's count is per calling thread
  igraph_set_error_handler(igraph_error_handler_printignore);  // default handler aborts
  igraph_set_warning_handler(igraph_warning_handler_ignore);

  try {
    vector<Corre_3DMatch> correspondence;
    to_upstream(corr_in, correspondence);
    const string metric = metric_name(p.metric);
    // Graph_construction uses d_cmp = 10 * resolution (hard-coded). Passing
    // d_cmp_pr * pr / 10 makes its d_cmp equal d_cmp_pr * pr.
    const float graph_resolution = static_cast<float>(p.d_cmp_pr * pr / 10.0);
    const int min_clique = std::max(3, p.min_clique_size);

    // ---- Steps 1-4 with the explosion guard ------------------------------------------------
    const std::array<double, 3> t_levels{{p.t_cmp, std::max(p.t_cmp, 0.995), std::max(p.t_cmp, 0.9975)}};
    int level = 0;
    bool halved = false;
    Eigen::MatrixXf Graph;
    Clustering cl;
    std::unique_ptr<ig::Matrix> g_mat;
    const auto t_graph = Clock::now();
    while (true) {
      const double t_cmp = t_levels[level];
      // Step 1: compatibility graph (upstream line 1446), FOG or SOG.
      Graph = Graph_construction(correspondence, graph_resolution, p.use_sog, static_cast<float>(t_cmp));
      Eigen::setNbThreads(threads);  // upstream leaves Eigen at 6 threads
      if (Graph.norm() == 0) {       // upstream line 1451
        r.t_cmp_used = t_cmp;
        r.reason = "empty_graph";
        r.stage_ms[0] = ms_between(t_graph, Clock::now());
        return r;
      }
      // Steps 2-3: clustering coefficient, OTSU, scores (upstream lines 1455-1572).
      cl = clustering_coefficients(Graph);
      if (!add_overlap) {
        for (size_t i = 0; i < correspondence.size(); ++i) {
          correspondence[i].score = cl.factor[i];
        }
      }
      // Step 4: adjacency matrix for igraph, with upstream's graph reduction (lines 1719-1759).
      const int n = static_cast<int>(correspondence.size());
      g_mat = std::make_unique<ig::Matrix>(n, n);
      long edges = 0;
      if (cl.threshold > 3 && correspondence.size() > 100) {
        float f = 10;
        const double base = max(cl.otsu, cl.total);
        for (int guard = 0; guard < 100000; ++guard) {
          if (f * base > cl.sorted_desc[49]) {
            f -= 0.05;
          } else {
            break;
          }
        }
        for (int i = 0; i < n; ++i) {
          if (cl.factor[i] > f * base) {
            for (int j = i + 1; j < n; ++j) {
              if (cl.factor[j] > f * base) {
                MATRIX(g_mat->m, i, j) = Graph(i, j);
                MATRIX(g_mat->m, j, i) = MATRIX(g_mat->m, i, j);
                if (Graph(i, j)) {
                  ++edges;
                }
              }
            }
          }
        }
      } else {
        for (int i = 0; i < n; ++i) {
          for (int j = i + 1; j < n; ++j) {
            if (Graph(i, j)) {
              MATRIX(g_mat->m, i, j) = Graph(i, j);
              MATRIX(g_mat->m, j, i) = MATRIX(g_mat->m, i, j);
              ++edges;
            }
          }
        }
      }
      r.t_cmp_used = t_cmp;
      r.graph_edges = edges;
      if (p.max_graph_edges <= 0 || edges <= p.max_graph_edges) {
        break;
      }
      // Guard: a too-dense graph makes clique enumeration explode. Tighten, then shrink once.
      if (level < 2) {
        ++level;
        continue;
      }
      if (!halved) {
        vector<Corre_3DMatch> half;
        half.reserve(correspondence.size() / 2 + 1);
        for (size_t i = 0; i < correspondence.size(); i += 2) {
          half.push_back(correspondence[i]);
        }
        correspondence.swap(half);
        halved = true;
        continue;
      }
      r.reason = "graph_too_dense";
      r.stage_ms[0] = ms_between(t_graph, Clock::now());
      return r;
    }
    r.stage_ms[0] = ms_between(t_graph, Clock::now());

    // ---- Step 5: maximal cliques (upstream lines 1761-1804) -------------------------------
    const auto t_cliques = Clock::now();
    ig::IntList cliques;
    long clique_num = 0;
    {
      ig::Graph g;
      ig::Vector weight(0);
      ig::check(
        igraph_weighted_adjacency(&g.g, &g_mat->m, IGRAPH_ADJ_UNDIRECTED, &weight.v, IGRAPH_LOOPS_ONCE),
        "igraph_weighted_adjacency");
      g.valid = true;
      g_mat.reset();  // N x N doubles, no longer needed
      CliqueSink sink;
      sink.list = &cliques.l;
      sink.max = p.max_cliques > 0 ? p.max_cliques : LONG_MAX;
      // Same enumeration as igraph_maximal_cliques(), with a stop after `max_cliques`.
      ig::check(
        igraph_maximal_cliques_callback(&g.g, &on_clique, &sink, min_clique, 0),
        "igraph_maximal_cliques_callback");
      clique_num = sink.count;
    }
    r.clique_count = clique_num;
    r.stage_ms[1] = ms_between(t_cliques, Clock::now());
    if (clique_num == 0) {
      r.reason = "no_cliques";
      return r;
    }

    // ---- Step 6: node-guided clique selection (upstream lines 1811-1823) -------------------
    const auto t_select = Clock::now();
    const int n = static_cast<int>(correspondence.size());
    vector<int> remain(static_cast<size_t>(clique_num));
    std::iota(remain.begin(), remain.end(), 0);
    std::vector<node_cliques> node_best(static_cast<size_t>(n));
    const int est_num = p.clique_ranking ? std::max(1, p.top_k) : INT_MAX;
    find_largest_clique_of_node(Graph, &cliques.l, correspondence, node_best.data(), remain, n, est_num, "NULL");
    r.selected_cliques = static_cast<int>(remain.size());
    r.stage_ms[2] = ms_between(t_select, Clock::now());

    // ---- Steps 7-8: one SVD pose per clique, scored on ALL pairs (upstream 1825-1885) -------
    const auto t_hyp = Clock::now();
    PointCloudPtr src_corr_pts(new pcl::PointCloud<pcl::PointXYZ>);
    PointCloudPtr des_corr_pts(new pcl::PointCloud<pcl::PointXYZ>);
    for (const Corre_3DMatch & c : correspondence) {
      src_corr_pts->push_back(c.src);
      des_corr_pts->push_back(c.des);
    }
    const double inlier_thresh = p.inlier_thresh_pr * pr;
    const double weight_thresh = add_overlap ? 0.5 : 0.0;  // upstream lines 1556-1563
    const int H = static_cast<int>(remain.size());
    std::vector<double> scores(static_cast<size_t>(H), 0.0);
    std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> poses(
      static_cast<size_t>(H), Eigen::Matrix4d::Identity());
#pragma omp parallel for schedule(dynamic, 4)
    for (int i = 0; i < H; ++i) {
      vector<Corre_3DMatch> Group;
      igraph_vector_int_t * v = igraph_vector_int_list_get_ptr(&cliques.l, remain[i]);
      const int group_size = static_cast<int>(igraph_vector_int_size(v));
      Group.reserve(group_size);
      for (int j = 0; j < group_size; ++j) {
        Group.push_back(correspondence[VECTOR(*v)[j]]);
      }
      Eigen::Matrix4d est_trans = Eigen::Matrix4d::Identity();
      scores[i] = evaluation_trans(
        Group, correspondence, src_corr_pts, des_corr_pts, weight_thresh, est_trans,
        inlier_thresh, metric, static_cast<float>(pr), true);  // instance-equal SVD
      poses[i] = est_trans;
    }
    // Upstream picks the best inside the parallel loop under `omp critical`, so equal scores
    // are resolved by thread timing. Here: highest score, then lowest index.
    int best = -1;
    double best_score = 0.0;
    for (int i = 0; i < H; ++i) {
      if (scores[i] > 0 && best_score < scores[i]) {
        best_score = scores[i];
        best = i;
      }
    }
    if (best < 0) {
      r.reason = "no_valid_hypothesis";
      r.stage_ms[3] = ms_between(t_hyp, Clock::now());
      return r;
    }

    // ---- Step 9: refinement (upstream line 1890) ------------------------------------------
    Eigen::Matrix4d best_est = poses[best];
    post_refinement(
      correspondence, src_corr_pts, des_corr_pts, best_est, best_score, inlier_thresh,
      p.refinement_iterations, metric);
    r.stage_ms[3] = ms_between(t_hyp, Clock::now());
    if (!best_est.allFinite()) {
      r.reason = "non_finite_pose";
      return r;
    }
    r.T = best_est;
    r.mac_score = best_score;
    r.ok = true;
  } catch (const std::exception & e) {
    r.ok = false;
    r.reason = e.what();
  }
  return r;
}

}  // namespace mac
