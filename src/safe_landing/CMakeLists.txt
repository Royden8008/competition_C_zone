cmake_minimum_required(VERSION 3.0.2)
project(safe_landing)

add_compile_options(-std=c++17 -O2 -Wall)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  mavros_msgs
  pcl_ros
  pcl_conversions
)

find_package(PCL REQUIRED COMPONENTS common io filters segmentation sample_consensus search kdtree)
find_package(Eigen3 REQUIRED)

find_path(ALVAR_INCLUDE_DIR
  NAMES ar_track_alvar_msgs/AlvarMarkers.h
  PATHS /opt/ros/noetic/include
)
if(ALVAR_INCLUDE_DIR)
  message(STATUS "safe_landing: ar_track_alvar_msgs headers found, enabling marker support")
  add_definitions(-DSAFE_LANDING_HAVE_ALVAR)
  set(ALVAR_INCLUDE_DIRS ${ALVAR_INCLUDE_DIR})
else()
  message(STATUS "safe_landing: ar_track_alvar_msgs headers NOT found, marker support disabled")
  set(ALVAR_INCLUDE_DIRS "")
endif()

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs sensor_msgs nav_msgs
                 visualization_msgs tf2 tf2_ros tf2_geometry_msgs
                 mavros_msgs pcl_ros pcl_conversions
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${ALVAR_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/px4_interface.cpp
  src/landing_zone_evaluator.cpp
  src/guarded_descent.cpp
  src/precision_landing.cpp
  src/grid_planner.cpp
  src/waypoint_follower.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(safe_landing_node src/safe_landing_node.cpp)
target_link_libraries(safe_landing_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

# 滚动局部障碍地图:夹在 /cloud_registered 与消费者之间做累积+裁剪,独立节点。
add_executable(local_obstacle_map_node src/local_obstacle_map_node.cpp)
target_link_libraries(local_obstacle_map_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
