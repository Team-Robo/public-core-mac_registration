// Force-included (-include) into the vendored upstream sources only.
//
// 1. Upstream relies on headers that older PCL releases (and pcl_visualizer.h) pulled in
//    indirectly. PCL 1.12 no longer does, so they are provided here instead of editing the
//    upstream files:
//      boost::mt19937, boost::uniform_int, boost::normal_distribution, boost::variate_generator
//        (funcs.cpp, PCR.cpp)
//      std::setprecision (registration.cpp)
// 2. Eva.h defines a macro `Random(x)` before it includes <Eigen/Eigen>. Upstream files always
//    include PCL (and so Eigen) first, which keeps Eigen's own `Random` functions intact.
//    Including Eigen here guarantees that order for every file, including ours.
#pragma once
#include <Eigen/Eigen>
#include <boost/random.hpp>
#include <iomanip>
