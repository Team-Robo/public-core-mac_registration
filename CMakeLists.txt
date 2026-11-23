cmake_minimum_required(VERSION 3.16)
project(mac_registration C CXX)  # C: rosidl generates C type support

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# ---------------------------------------------------------------------------------------------
# Dependencies
# ---------------------------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)
# No `visualization` component: the vendored code is built headless (patch 0002).
find_package(PCL REQUIRED COMPONENTS
  common io kdtree search features filters registration keypoints sample_consensus segmentation)

# igraph 0.10.x, built by scripts/install_igraph.sh into /usr/local (apt only has 0.9.6).
find_package(igraph CONFIG QUIET)
if(NOT igraph_FOUND)
  message(FATAL_ERROR
    "igraph not found. Run: sudo ${CMAKE_CURRENT_SOURCE_DIR}/scripts/install_igraph.sh")
endif()
if(igraph_VERSION VERSION_LESS 0.10.0 OR NOT igraph_VERSION VERSION_LESS 0.11.0)
  message(FATAL_ERROR "mac_registration needs igraph 0.10.x, found ${igraph_VERSION}. "
    "Run scripts/install_igraph.sh")
endif()
message(STATUS "mac_registration: igraph ${igraph_VERSION}, PCL ${PCL_VERSION}")

# ---------------------------------------------------------------------------------------------
# Messages
# ---------------------------------------------------------------------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RegistrationStats.msg"
  DEPENDENCIES std_msgs geometry_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)

# ---------------------------------------------------------------------------------------------
# mac_core: the vendored upstream sources, unchanged except patches 0001/0002, plus the
# globals upstream defines in main.cpp and no-op viewer stubs. Upstream's own flags: C++14.
# ---------------------------------------------------------------------------------------------
set(MAC_UPSTREAM_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/MAC/Linux)
add_library(mac_core STATIC
  ${MAC_UPSTREAM_DIR}/funcs.cpp
  ${MAC_UPSTREAM_DIR}/desc_dec.cpp
  ${MAC_UPSTREAM_DIR}/PCR.cpp
  ${MAC_UPSTREAM_DIR}/registration.cpp
  src/mac_globals.cpp
  src/mac_viz_stubs.cpp
)
set_target_properties(mac_core PROPERTIES CXX_STANDARD 14 CXX_STANDARD_REQUIRED ON)
target_include_directories(mac_core
  PUBLIC ${MAC_UPSTREAM_DIR}
  SYSTEM PUBLIC ${PCL_INCLUDE_DIRS})
target_compile_definitions(mac_core PUBLIC ${PCL_DEFINITIONS})
# -w: upstream code is not ours to warn about. Note: no IGRAPH_VERSION_OLD (selects the 0.9 API).
# -include mac_compat.hpp: headers upstream got indirectly from older PCL (see that file).
target_compile_options(mac_core PRIVATE -O2 -w
  -include ${CMAKE_CURRENT_SOURCE_DIR}/src/mac_compat.hpp)
target_link_libraries(mac_core PUBLIC ${PCL_LIBRARIES} igraph::igraph OpenMP::OpenMP_CXX Eigen3::Eigen)

# ---------------------------------------------------------------------------------------------
# mac_registration_core: our wrapper, front-end and helpers (no ROS).
# ---------------------------------------------------------------------------------------------
add_library(mac_registration_core STATIC
  src/mac_solver.cpp
  src/frontend.cpp
  src/pipeline.cpp
  src/depth_cloud.cpp
  src/tabletop.cpp
  src/reference_store.cpp
)
target_include_directories(mac_registration_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_compile_options(mac_registration_core PRIVATE -Wall -Wextra -Wno-unused-parameter)
target_link_libraries(mac_registration_core PUBLIC mac_core)

# ---------------------------------------------------------------------------------------------
# ROS nodes
# ---------------------------------------------------------------------------------------------
add_executable(mac_registration_node src/mac_registration.cpp src/main.cpp)
target_compile_options(mac_registration_node PRIVATE -Wall -Wextra -Wno-unused-parameter)
target_link_libraries(mac_registration_node mac_registration_core "${cpp_typesupport_target}")
ament_target_dependencies(mac_registration_node
  rclcpp std_msgs std_srvs sensor_msgs geometry_msgs vision_msgs tf2_ros pcl_conversions)

add_executable(tabletop_object_cloud_node src/tabletop_object_cloud.cpp)
target_compile_options(tabletop_object_cloud_node PRIVATE -Wall -Wextra -Wno-unused-parameter)
target_link_libraries(tabletop_object_cloud_node mac_registration_core)
ament_target_dependencies(tabletop_object_cloud_node rclcpp sensor_msgs pcl_conversions)

install(TARGETS mac_registration_node tabletop_object_cloud_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/install_igraph.sh DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config rviz DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_package()
