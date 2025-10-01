#include "wrapper/ros_noetic/ieskf_frontend_noetic_wrapper.h"

#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>

#include "ieskf_slam/CloudWithPose.h"
#include "ieskf_slam/globaldefine.h"
#include "ieskf_slam/tools/timer.h"
#include "livox_ros_driver/CustomMsg.h"
#include "livox_ros_driver/CustomPoint.h"

namespace ROSNoetic {

IESKFFrontEndWrapper::IESKFFrontEndWrapper(ros::NodeHandle &nh) {
    nh.param<std::string>("wrapper/mode", mode_, "offline");
    nh.param<std::string>("wrapper/bag_path", bag_path_, "");
    nh.param<double>("wrapper/speed_factor", speed_factor_, 1.0);
    nh.param<bool>("wrapper/save_pcd", save_pcd, false);
    // std::string config_file_name, lidar_topic, imu_topic;
    nh.param<std::string>("wrapper/config_file_name", config_file_name, "");
    nh.param<std::string>("wrapper/lidar_topic", lidar_topic, "/lidar");
    nh.param<std::string>("wrapper/imu_topic", imu_topic, "/imu");
    std::string time_unit_str;
    nh.param<std::string>("wrapper/time_unit", time_unit_str, "1e-3");

    nh.param<int>("wrapper/num_scans", num_scans, 6);
    nh.param<int>("wrapper/point_filter_num", point_filter_num, 1);
    nh.param<double>("wrapper/blind", blind, 0.011);

    time_unit = std::stod(time_unit_str);
    front_end_ptr = std::make_shared<IESKFSlam::FrontEnd>(CONFIG_DIR + config_file_name, "front_end");

    int lidar_type = 0;
    nh.param<int>("wrapper/lidar_type", lidar_type, AVIA);
    if (lidar_type == AVIA) {
        lidar_process_ptr = std::make_shared<AVIAProcess>(num_scans, point_filter_num, blind);
    } else if (lidar_type == VELO) {
        lidar_process_ptr = std::make_shared<VelodyneProcess>(num_scans, point_filter_num, blind);
    } else {
        ROS_ERROR("Unsupported lidar type");
        exit(100);
    }

    curr_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("curr_cloud", 100);
    path_pub = nh.advertise<nav_msgs::Path>("path", 100);
    local_map_pub = nh.advertise<sensor_msgs::PointCloud2>("local_map", 100);

    if (mode_ == "realtime") {
        if (lidar_type == AVIA) {
            cloud_subscriber = nh.subscribe(lidar_topic, 1000, &IESKFFrontEndWrapper::aviaCallBack, this);
        }
        if (lidar_type == VELO) {
            cloud_subscriber = nh.subscribe(lidar_topic, 1000, &IESKFFrontEndWrapper::velodyneCallBack, this);
        }

        imu_subscriber = nh.subscribe(imu_topic, 1000, &IESKFFrontEndWrapper::imuMsgCallBack, this);
    }
    cloud_pose_pub = nh.advertise<ieskf_slam::CloudWithPose>("cloud_with_pose", 10);
    run();
}

IESKFFrontEndWrapper::~IESKFFrontEndWrapper() {}

void IESKFFrontEndWrapper::aviaCallBack(const livox_ros_driver::CustomMsgPtr &msg) {
    IESKFSlam::PointCloud cloud;
    lidar_process_ptr->process(*msg, cloud, time_unit);
    front_end_ptr->addPointCloud(cloud);
}

void IESKFFrontEndWrapper::velodyneCallBack(const sensor_msgs::PointCloud2Ptr &msg) {
    IESKFSlam::PointCloud cloud;
    lidar_process_ptr->process(*msg, cloud, time_unit);
    front_end_ptr->addPointCloud(cloud);
}

void IESKFFrontEndWrapper::imuMsgCallBack(const sensor_msgs::ImuPtr &msg) {
    IESKFSlam::IMU imu;
    imu.time_stamp.fromNsec(msg->header.stamp.toNSec());
    imu.acceleration = {msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z};
    imu.gyroscope = {msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z};
    front_end_ptr->addImu(imu);
}

void IESKFFrontEndWrapper::publishMsg() {
    auto X = front_end_ptr->readState();
    IESKFSlam::PCLPointCloud cloud = front_end_ptr->readCurrentPointCloud();
    pcl::transformPointCloud(cloud, cloud, IESKFSlam::compositeTransform(X.rotation, X.position).cast<float>());

    // 发布TF变换
    publishTF(X);

    // 发布路径和点云
    static nav_msgs::Path path;
    path.header.frame_id = "map";

    geometry_msgs::PoseStamped psd;
    psd.header.frame_id = "map";
    psd.header.stamp = ros::Time::now();
    psd.pose.position.x = X.position.x();
    psd.pose.position.y = X.position.y();
    psd.pose.position.z = X.position.z();
    psd.pose.orientation.x = X.rotation.x();
    psd.pose.orientation.y = X.rotation.y();
    psd.pose.orientation.z = X.rotation.z();
    psd.pose.orientation.w = X.rotation.w();
    path.poses.push_back(psd);

    path_pub.publish(path);
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud, cloud_msg);
    cloud_msg.header.frame_id = "map";
    cloud_msg.header.stamp = ros::Time::now();
    curr_cloud_pub.publish(cloud_msg);

    IESKFSlam::PCLPointCloud local_map = front_end_ptr->readCurrentLocalMap();
    pcl::toROSMsg(local_map, cloud_msg);
    cloud_msg.header.frame_id = "map";
    cloud_msg.header.stamp = ros::Time::now();
    local_map_pub.publish(cloud_msg);

    ieskf_slam::CloudWithPose cloud_with_pose_msg;

    cloud = front_end_ptr->readUndistortedPointCloud();
    pcl::toROSMsg(cloud, cloud_with_pose_msg.point_cloud);
    cloud_with_pose_msg.pose.position = psd.pose.position;
    cloud_with_pose_msg.pose.orientation.x = X.rotation.x();
    cloud_with_pose_msg.pose.orientation.y = X.rotation.y();
    cloud_with_pose_msg.pose.orientation.z = X.rotation.z();
    cloud_with_pose_msg.pose.orientation.w = X.rotation.w();
    cloud_pose_pub.publish(cloud_with_pose_msg);
}

void IESKFFrontEndWrapper::publishTF(const IESKFSlam::IESKF::State18 &state) {
    static tf2_ros::TransformBroadcaster tf_broadcaster;

    // 发布从map到base_link的变换（机器人位姿）
    geometry_msgs::TransformStamped transform_stamped;
    transform_stamped.header.stamp = ros::Time::now();
    transform_stamped.header.frame_id = "map";
    transform_stamped.child_frame_id = "base_link";

    transform_stamped.transform.translation.x = state.position.x();
    transform_stamped.transform.translation.y = state.position.y();
    transform_stamped.transform.translation.z = state.position.z();

    transform_stamped.transform.rotation.x = state.rotation.x();
    transform_stamped.transform.rotation.y = state.rotation.y();
    transform_stamped.transform.rotation.z = state.rotation.z();
    transform_stamped.transform.rotation.w = state.rotation.w();

    tf_broadcaster.sendTransform(transform_stamped);
}

void IESKFFrontEndWrapper::playBagToIESKF_Streaming(const std::string &bag_path, double speed_factor) {
    TimerLoggerInit(RESULT_DIR + "track_time.txt");
    rosbag::Bag bag;
    bag.open(bag_path, rosbag::bagmode::Read);

    std::vector<std::string> topics = {imu_topic, lidar_topic};

    std::cout << imu_topic << std::endl;
    std::cout << lidar_topic << std::endl;

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    ros::Time prev_time;
    bool first = true;

    ROS_INFO("Start playing bag with speed_factor=%f", speed_factor);

    int count = 0;
    for (const rosbag::MessageInstance &msg : view) {
        if (!ros::ok()) {
            ROS_WARN("ros::ok() is false, exiting bag playback");
            break;
        }
        ros::Time curr_time = msg.getTime();

        if (first) {
            prev_time = curr_time;
            first = false;
        } else {
            double dt = (curr_time - prev_time).toSec();
            prev_time = curr_time;

            if (dt > 0 && speed_factor > 0 && speed_factor != 1) {
                // 1倍速不休眠，其他倍速根据倍数调整休眠时间
                double sleep_time = dt / speed_factor;
                std::this_thread::sleep_for(std::chrono::duration<double>(sleep_time));
            }
        }

        count++;
        // ROS_INFO("Processing message #%d at time %f", count, curr_time.toSec());

        auto imu_msg = msg.instantiate<sensor_msgs::Imu>();
        if (imu_msg) {
            IESKFSlam::IMU imu;
            imu.time_stamp.fromNsec(imu_msg->header.stamp.toNSec());
            imu.acceleration = {imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y,
                                imu_msg->linear_acceleration.z};
            imu.gyroscope = {imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z};
            front_end_ptr->addImu(imu);
            // 这里如果有耗时需求可以用 Timer::Evaluate 包裹
        }

        auto velo_msg = msg.instantiate<sensor_msgs::PointCloud2>();
        if (velo_msg) {
            IESKFSlam::PointCloud cloud;
            lidar_process_ptr->process(*velo_msg, cloud, time_unit);

            front_end_ptr->addPointCloud(cloud);

            bool track_result = false;
            Timer([&]() { track_result = front_end_ptr->track(); }, "完整流程", RESULT_DIR + "track_time.txt");

            if (track_result) {
                publishMsg();
            }
        }

        auto livox_msg = msg.instantiate<livox_ros_driver::CustomMsg>();
        if (livox_msg) {
            IESKFSlam::PointCloud cloud;
            lidar_process_ptr->process(*livox_msg, cloud, time_unit);

            front_end_ptr->addPointCloud(cloud);

            bool track_result = false;
            Timer([&]() { track_result = front_end_ptr->track(); }, "完整流程", RESULT_DIR + "track_time.txt");

            if (track_result) {
                publishMsg();
            }
        }
        // livox to do
    }
    savePCD();
    ROS_INFO("Finished playing bag with %d messages", count);

    bag.close();
}

void IESKFFrontEndWrapper::run() {
    if (mode_ == "offline") {
        ROS_INFO_STREAM("Running in offline mode, playing bag: " << bag_path_ << " at speed factor: " << speed_factor_);

        playBagToIESKF_Streaming(bag_path_, speed_factor_);
        ros::spin();
    } else {
        ROS_INFO("Running in realtime mode (ROS subscription).");
        ros::Rate rate(500);
        while (ros::ok()) {
            rate.sleep();
            ros::spinOnce();
            if (front_end_ptr->track()) {
                if (save_pcd) {
                    savePCD();
                }
                publishMsg();
            }
        }
    }
}

void IESKFFrontEndWrapper::savePCD() {
    static int file_counter = 0;
    auto &scan = front_end_ptr->readCurrentPointCloud();
    if (!scan.empty()) {
        // 按序号生成文件名
        std::string filename = RESULT_DIR + "scan/" + std::to_string(file_counter) + ".pcd";
        pcl::io::savePCDFileBinary(filename, scan);

        ROS_INFO("Saved point cloud to %s", filename.c_str());

        // 计数器递增
        file_counter++;
    } else {
        ROS_WARN("Point cloud is empty, nothing to save.");
    }
}
}  // namespace ROSNoetic