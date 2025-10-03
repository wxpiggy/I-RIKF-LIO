cmake_minimum_required(VERSION 3.10)
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
add_message_files(
  FILES
  CloudWithPose.msg
)
generate_messages(
  DEPENDENCIES
  sensor_msgs
  geometry_msgs
)
find_package(Open3D REQUIRED)
include_directories(${Open3D_INCLUDE_DIRS})
find_package(PCL REQUIRED )
find_package(TBB REQUIRED)
# YAML CPP
find_package(Ceres REQUIRED)
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
  ${CERES_INCLUDE_DIRS}
 include
 lib/
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDEDIR}
  ${CATKIN_DEVEL_PREFIX}/include
)
SET(IESKF_SLAM_FRONT_FILE
    src/ieskf_slam/modules/frontend/frontend.cpp
    src/ieskf_slam/modules/ieskf/ieskf.cpp
    src/ieskf_slam/modules/map/rect_map_manager.cpp
    src/ieskf_slam/modules/propagater/propagater.cpp
    

)
SET(IESKF_SLAM_BACK_FILE
    src/ieskf_slam/modules/backend/posegraph.cpp
    src/ieskf_slam/modules/backend/backend.cpp

)
SET(NOETIC_FRONTEND_WRAPPER_FILE
    src/wrapper/ros_noetic/ieskf_frontend_noetic_wrapper.cpp
)
SET(NOETIC_BACKEND_WRAPPER_FILE
    src/wrapper/ros_noetic/ieskf_backend_noetic_wrapper.cpp
)
# find_package(slam_viewer REQUIRED)
SET(LIB_FILE
lib/scan_context/Scancontext.cpp
lib/ikd_Tree/ikd_Tree.cpp
)

add_executable(ieskf_frontend_node src/app/ros_noetic/ieskf_noetic_running_node.cpp
${NOETIC_FRONTEND_WRAPPER_FILE}
${IESKF_SLAM_FRONT_FILE}
${LIB_FILE})


target_link_libraries(ieskf_frontend_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
${YAML_CPP_LIBRARIES}
 TBB::tbb
)
add_dependencies(ieskf_frontend_node ${PROJECT_NAME}_generate_messages_cpp)
# add_executable(ieskf_backend_node src/app/ros_noetic/ieskf_noetic_backend_node.cpp
# ${NOETIC_BACKEND_WRAPPER_FILE}
# ${IESKF_SLAM_BACK_FILE}
# ${LIB_FILE}

# )
find_package(GTSAM REQUIRED)  
# add_dependencies(ieskf_backend_node ${PROJECT_NAME}_generate_messages_cpp)
# target_link_libraries(ieskf_backend_node
# ${Open3D_LIBRARIES}
#   ${catkin_LIBRARIES}
#   ${PCL_LIBRARIES}
#   pcl_common pcl_io pcl_kdtree pcl_registration pcl_filters pcl_search
#   ${YAML_CPP_LIBRARIES}
#   Ceres::ceres
#   # ${OpenCV_LIBRARIES}
#   # Ceres::ceres
# )

# 把 STDesc.cpp 先编译成一个库
add_library(STDesc lib/STD/STDesc.cpp)

# 编译 mapping 节点
add_executable(mapping src/mapping.cpp)

# 链接 mapping
target_link_libraries(mapping
    PRIVATE
    STDesc
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${CERES_LIBRARIES}     # 现代写法，优于 ${CERES_LIBRARIES}
    glog        # 更推荐这样写，而不是裸 glog
    gflags            # gflags 一般直接用 gflags 就行
    gtsam      # 推荐用命名空间写法
)





add_executable(pandar_to_velodyne /livox_ws/src/I-RIKF-LIO/src/tools/pandar_to_velodyne.cpp)
target_link_libraries(pandar_to_velodyne ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${PYTHON_LIBRARIES})
target_include_directories(pandar_to_velodyne PRIVATE ${PYTHON_INCLUDE_DIRS})