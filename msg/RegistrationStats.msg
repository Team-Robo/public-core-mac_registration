# Per-frame diagnostics of mac_registration. Published on every processed frame,
# including frames the quality gate rejected, so thresholds can be tuned from data.

std_msgs/Header header          # copied from the input cloud (stamp + frame_id)
string label                    # reference label, e.g. "bottle"

bool accepted                   # true -> ~/pose, ~/aligned_cloud (and TF) were published
string reject_reason            # empty when accepted, e.g. "low_fitness", "no_reference"
bool has_reference

# Sizes
uint32 input_points             # finite points that passed the intensity filter
uint32 live_points              # after voxel downsampling
uint32 reference_points
uint32 correspondences          # after mutual matching and the cap
uint32 graph_edges              # edges of the (second-order) compatibility graph
uint32 cliques                  # maximal cliques found by igraph
uint32 hypotheses               # cliques kept by node-guided selection
float64 t_cmp_used              # compatibility threshold actually used (may be raised by the edge guard)

# Quality (all computed with the final pose, after ICP when enabled)
uint32 inlier_count             # correspondences with residual < inlier threshold
float64 mae                     # mean residual of those inliers [m]
float64 fitness                 # share of reference points with a live neighbour within fitness_radius
float64 cloud_rmse              # RMS of those neighbour distances [m]
float64 mac_score               # MAC's own MAE score (higher is better, unitless)
float64 point_resolution        # pr used for the graph [m]

# Timing [ms]
float64 preprocess_ms
float64 features_ms
float64 match_ms
float64 graph_ms
float64 cliques_ms
float64 selection_ms
float64 hypotheses_ms
float64 icp_ms
float64 total_ms

# Pose of the object frame in the input frame (filled even when rejected, for tuning)
geometry_msgs/Pose pose
