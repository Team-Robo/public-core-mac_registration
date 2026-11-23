// Global switches that upstream MAC defines in its main.cpp (which we do not compile).
// Values reproduce the paper's default configuration:
//   add_overlap    = false : no learned overlap scores; every correspondence weighs the same
//   low_inlieratio = false : unused by the code path we call
//   no_logs        = true  : unused by the code path we call; keeps upstream quiet if ever read
bool add_overlap = false;
bool low_inlieratio = false;
bool no_logs = true;
