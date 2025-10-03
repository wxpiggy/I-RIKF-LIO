#pragma once

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <Eigen/Dense>
#include <boost/foreach.hpp>
#include <chrono>
#include <thread>

#include "ieskf_slam/globaldefine.h"
#include "ieskf_slam/modules/frontend/frontend.h"
#include "livox_ros_driver/CustomMsg.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/PointCloud2.h"
// #include "wrapper/ros_noetic/lidar_process/avia_process.h"
// #include "wrapper/ros_noetic/lidar_process/velodyne_process.h"
#include "wrapper/ros_noetic/lidar_processer.h"

namespace ROSNoetic {

enum LIDAR_TYPE { AVIA = 0, VELO = 1, OUSTER = 2 };

class IESKFFrontEndWrapper {
   private:
    IESKFSlam::FrontEnd::Ptr front_end_ptr;
    std::string config_file_name, lidar_topic, imu_topic;

    ros::Subscriber cloud_subscriber;
    ros::Subscriber imu_subscriber;
    // ros::Subscriber odometry_subscriber;
    ros::Publisher curr_cloud_pub;
    ros::Publisher path_pub;
    ros::Publisher local_map_pub;
    ros::Publisher cloud_pose_pub;
    ros::Publisher odom_pub;
    // std::shared_ptr<CommonLidarProcessInterface> lidar_process_ptr;
    std::shared_ptr<LidarPreprocessor> lidar_process;
    IESKFSlam::PCLPointCloud curr_cloud;

    Eigen::Quaterniond curr_q;
    Eigen::Vector3d curr_t;
    double time_unit = 0.0;

    // 离线播放参数
    std::string mode_;  // "realtime" 或 "offline"
    std::string bag_path_;
    double speed_factor_ = 1.0;
    bool save_pcd;
    // 可视化模式 "rviz" 或 "pangolin"
    int num_scans;
    int point_filter_num;
    double blind;

    Eigen::Vector3d last_kf_pos = Eigen::Vector3d::Zero();
    Eigen::Quaterniond last_kf_rot = Eigen::Quaterniond::Identity();
    ros::Time last_kf_time = ros::Time(0);
    
    void aviaCallBack(const livox_ros_driver::CustomMsgPtr &msg);
    void velodyneCallBack(const sensor_msgs::PointCloud2Ptr &msg);
    void ousterCallBack(const sensor_msgs::PointCloud2Ptr &msg);
    void imuMsgCallBack(const sensor_msgs::ImuPtr &msg);
    void publishMsg();
    void run();
    void playBagToIESKF_Streaming(const std::string &bag_path, double speed_factor);
    void savePCD();
    void publishTF(const IESKFSlam::IESKF::State18 &state);

   public:
    IESKFFrontEndWrapper(ros::NodeHandle &nh);
    ~IESKFFrontEndWrapper();
};

}  // namespace ROSNoetic