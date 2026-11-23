# Vendored: 3D Registration with Maximal Cliques (MAC)

| | |
|---|---|
| Upstream | https://github.com/zhangxy0517/3D-Registration-with-Maximal-Cliques |
| Commit | `b0d19635c4c9d69399397495d02422eda94248c2` ("Fix dataset link in README") |
| Vendored | 2026-10-05, from the local clone `ros2_ws/src/claude_code_ref/3D-Registration-with-Maximal-Cliques` |
| Licence | MIT, Copyright (c) 2023 zhangxy0517 — see `LICENSE` in this folder |
| Paper | Zhang, Yang, Zhang, Zhang. *3D Registration with Maximal Cliques*. CVPR 2023. arXiv:2305.10854 |

## What is here

| Path | Compiled? | Notes |
|---|---|---|
| `Linux/Eva.h` | header | unchanged |
| `Linux/funcs.cpp` | yes | patch 0002 |
| `Linux/desc_dec.cpp` | yes | patch 0002 |
| `Linux/PCR.cpp` | yes | unchanged |
| `Linux/registration.cpp` | yes | patches 0001 + 0002 |

Only the compiled sources are vendored. Upstream's CLI driver (`Linux/main.cpp`), its VTK
viewers (`Linux/visualization.cpp`), its build file and readme, the demo data, the Windows
build and the Python port are not. The uncompiled files and the demo data were removed on
2026-10-06, together with the package's tests.

## Patches (in `patches/`, applied to the files above)

1. `0001-fix-vis-memset.patch` — `find_largest_clique_of_node()` allocates `int vis[n]` but
   `memset(vis, 0, n)` clears only `n` *bytes*. The uninitialised counters can keep unused
   cliques alive (a leak) or cancel a selected clique's count and drop its hypothesis. Fix:
   `n * sizeof(int)`. Both the igraph 0.9 and 0.10 code paths are patched; only the 0.10 path
   is compiled.
2. `0002-drop-dead-includes.patch` — removes `#include <pcl/visualization/pcl_visualizer.h>`
   from `funcs.cpp`, `desc_dec.cpp` and `registration.cpp`. No visualisation symbol is used in
   those files; the include only pulled VTK headers into a headless build.

Verify: applying both patches to the upstream files reproduces the vendored files byte for
byte (`patch -p1 < patches/0001-*.patch; patch -p1 < patches/0002-*.patch` inside a copy of
upstream's tree).

## Build-level shims (no source edits)

- `src/mac_compat.hpp` is force-included (`-include`) into the vendored sources. It provides
  Boost.Random and `<iomanip>`, which upstream received indirectly from older PCL releases
  (PCL 1.12 no longer includes them), and includes Eigen before `Eva.h` so that `Eva.h`'s
  `Random(x)` macro cannot break Eigen.
- `src/mac_globals.cpp` defines the globals upstream defines in `main.cpp`
  (`add_overlap = false`, `low_inlieratio = false`, `no_logs = true`).
- `src/mac_viz_stubs.cpp` provides no-op versions of the four viewer functions that
  `registration.cpp` references (`visualization`, `Corres_Viewer_Score`,
  `Corres_selected_visual`, `RMSE_compute`).
- The sources are compiled as upstream compiles them: C++14, OpenMP, igraph >= 0.10 API
  (`IGRAPH_VERSION_OLD` not defined), here against igraph 0.10.17.

## How the code is used

`src/mac_solver.cpp` re-hosts the ~150 lines of `registration()` (overload 2) that call the
algorithm functions in order, without its console output, file writes, viewer windows,
`mkdir`/`exit(-1)` and hard-coded 12 threads, and returns the transform. Every algorithm
function (`Graph_construction`, `OTSU_thresh`, `find_largest_clique_of_node`,
`evaluation_trans`, `post_refinement`, `weight_SVD`) is called unchanged.

Fidelity check, 2026-10-05 (the test was removed on 2026-10-06): fed upstream's own demo
front-end, the wrapper reproduced the matrix printed in upstream's `Linux/readme.md` to
0.03 deg / 1.1 mm, with 521 558 maximal cliques and 9 716 hypotheses (upstream prints 528 867
and 9 762), and repeated bit for bit.
