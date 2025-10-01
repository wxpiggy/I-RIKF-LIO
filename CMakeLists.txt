cmake_minimum_required(VERSION 3.0.2)
project(ieskf_slam)

## Compile as C++11, supported in ROS Kinetic and newer
SET( CMAKE_CXX_FLAGS "-std=c++17 -O3" )
add_compile_options(-std=c++17)
add_definitions(-DPROJ_DIR="${CMAKE_CURRENT_SOURCE_DIR}")


#从这里传给了global_define.h
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
add_definitions(-DMP_EN)
add_definitions(-DMP_PROC_NUM=4)

find_package(Pangolin REQUIRED)
find_package(OpenMP QUIET)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  rosbag
  rosbag_storage
  message_generation
  message_runtime
  livox_ros_driver
)
find_package(PCL REQUIRED)
find_package(TBB REQUIRED)
# YAML CPP
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
include_directories("/usr/include/eigen3")
catkin_package(

#  INCLUDE_DIRS include
#  LIBRARIES ieskf_slam
#  CATKIN_DEPENDS roscpp rospy sensor_msgs
#  DEPENDS system_lib
)
include_directories(
  # "/root/catkin_ws/devel/include"
 include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDEDIR}
)
SET(IESKF_SLAM_FILE
    src/ieskf_slam/modules/frontend/frontend.cpp
    src/ieskf_slam/modules/ieskf/ieskf.cpp
    src/ieskf_slam/modules/map/rect_map_manager.cpp
    src/ieskf_slam/modules/frontbackPropagate/frontback_propagate.cpp
    src/tools/pangolin_visualizer.cpp
    

)
SET(NOETIC_FRONTEND_WRAPPER_FILE
    src/wrapper/ros_noetic/ieskf_frontend_noetic_wrapper.cpp
)
# find_package(slam_viewer REQUIRED)
SET(LIB_FILE
# lib/scan_context/Scancontext.cpp
lib/ikd_Tree/ikd_Tree.cpp
)

add_executable(ieskf_frontend_node src/app/ros_noetic/ieskf_noetic_running_node.cpp
${NOETIC_FRONTEND_WRAPPER_FILE}
${IESKF_SLAM_FILE}
${LIB_FILE})


target_link_libraries(ieskf_frontend_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
${YAML_CPP_LIBRARIES}
 ${Pangolin_LIBRARIES}
 TBB::tbb
)
