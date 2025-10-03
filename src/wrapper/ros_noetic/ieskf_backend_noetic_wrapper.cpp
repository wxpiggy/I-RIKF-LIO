#include "wrapper/ros_noetic/ieskf_backend_noetic_wrapper.h"

#include <visualization_msgs/Marker.h>

#include "ieskf_slam/globaldefine.h"
#include "nav_msgs/Path.h"
#include "pcl_conversions/pcl_conversions.h"
namespace ROSNoetic {
IESKFBackEndWrapper::IESKFBackEndWrapper(ros::NodeHandle &nh) {
    cloud_with_pose_sub = nh.subscribe("/cloud_with_pose", 100, &IESKFBackEndWrapper::cloudWithPoseMsgCallBack, this);
    loop_line_pub = nh.advertise<visualization_msgs::Marker>("loop_line", 10);
    loop_path_pub = nh.advertise<nav_msgs::Path>("loop_path", 10);
    global_opt_map_pub = nh.advertise<sensor_msgs::PointCloud2>("global_opt_map", 100);
    std::string config_file_name;
    nh.param<std::string>("wrapper/config_file_name", config_file_name, "");
    backend_ptr = std::make_shared<IESKFSlam::BackEnd>(CONFIG_DIR + config_file_name, "back_end");

    ros::spin();
}
IESKFBackEndWrapper::~IESKFBackEndWrapper() {}
void IESKFBackEndWrapper::cloudWithPoseMsgCallBack(const ieskf_slam::CloudWithPosePtr &msg) {
    pcl::fromROSMsg(msg->point_cloud, in_cloud);
    IESKFSlam::Pose pose;
    pose.rotation.x() = msg->pose.orientation.x;
    pose.rotation.y() = msg->pose.orientation.y;
    pose.rotation.z() = msg->pose.orientation.z;
    pose.rotation.w() = msg->pose.orientation.w;
    pose.position.x() = msg->pose.position.x;
    pose.position.y() = msg->pose.position.y;
    pose.position.z() = msg->pose.position.z;
    if (backend_ptr->addFrame(out_cloud, in_cloud, pose)) {
        publishConnection();
    }
    publishLoopPath();
}
void IESKFBackEndWrapper::publishLoopPath() {
    auto poses = backend_ptr->getPoses();  // 假设返回 std::vector<Pose>，Pose 里有 t 和 q

    nav_msgs::Path path_msg;
    path_msg.header.stamp = ros::Time::now();
    path_msg.header.frame_id = "map";  // 或 world，看你系统的坐标系

    for (size_t i = 0; i < poses.size(); ++i) {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = path_msg.header.stamp;
        pose_stamped.header.frame_id = path_msg.header.frame_id;

        // 取平移
        pose_stamped.pose.position.x = poses[i].position.x();
        pose_stamped.pose.position.y = poses[i].position.y();
        pose_stamped.pose.position.z = poses[i].position.z();

        // 取旋转
        Eigen::Quaterniond q(poses[i].rotation);
        pose_stamped.pose.orientation.x = q.x();
        pose_stamped.pose.orientation.y = q.y();
        pose_stamped.pose.orientation.z = q.z();
        pose_stamped.pose.orientation.w = q.w();

        path_msg.poses.push_back(pose_stamped);
    }

    loop_path_pub.publish(path_msg);
}
void IESKFBackEndWrapper::publishConnection() {
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(out_cloud, msg);
    msg.header.frame_id = "map";
    global_opt_map_pub.publish(msg);
    static int loop_count = 0;

    auto egeds = backend_ptr->getEdge().back();
    // if (loop_count >= egeds.size()) return;
    std::cout << "=======loop from " << egeds.from_vertex << " to " << egeds.to_vertex << "============" << std::endl;
    auto from = backend_ptr->getPoseWithID(egeds.from_vertex);
    auto to = backend_ptr->getPoseWithID(egeds.to_vertex);

    // 定义 Marker
    visualization_msgs::Marker line;
    line.header.frame_id = "map";
    line.header.stamp = ros::Time::now();
    line.ns = "loop_closure";
    line.id = loop_count;
    line.type = visualization_msgs::Marker::LINE_LIST;
    line.action = visualization_msgs::Marker::ADD;
    line.pose.orientation.w = 1.0;

    // 线宽
    line.scale.x = 0.2;

    // 颜色 (红色)
    line.color.r = 1.0;
    line.color.g = 1.0;
    line.color.b = 0.0;
    line.color.a = 1.0;

    // 两个端点
    geometry_msgs::Point p1, p2;

    p1.x = from.position.x();
    p1.y = from.position.y();
    p1.z = from.position.z();
    p2.x = to.position.x();
    p2.y = to.position.y();
    p2.z = to.position.z();
    std::cout << from.position << "connect with " << to.position << std::endl;
    line.points.push_back(p1);
    line.points.push_back(p2);

    loop_line_pub.publish(line);

    loop_count++;
}
}  // namespace ROSNoetic