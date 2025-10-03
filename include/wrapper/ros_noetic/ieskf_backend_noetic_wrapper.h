#pragma once
#include <ros/ros.h>

#include "ieskf_slam/CloudWithPose.h"
#include "ieskf_slam/modules/backend/backend.h"
namespace ROSNoetic {
class IESKFBackEndWrapper {
   private:
    ros::Publisher global_opt_map_pub;
    ros::Publisher loop_line_pub;
    ros::Subscriber cloud_with_pose_sub;
    ros::Publisher loop_path_pub;
    IESKFSlam::PCLPointCloud in_cloud, out_cloud;
    // function
    IESKFSlam::BackEnd::Ptr backend_ptr;

    void cloudWithPoseMsgCallBack(const ieskf_slam::CloudWithPosePtr &msg);
    void publishConnection();
    void publishLoopPath();
   public:
    IESKFBackEndWrapper(ros::NodeHandle &nh);

    ~IESKFBackEndWrapper();
};
}  // namespace ROSNoetic