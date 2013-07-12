#ifndef __OCULUS_ROS_UTIL__
#define __OCULUS_ROS_UTIL__

#include <OVR.h>
#include <oculus_ros/HMDInfo.h>
#include <geometry_msgs/Quaternion.h>

namespace oculus_ros {

void convertHMDInfoToMsg(const OVR::HMDInfo& info, oculus_ros::HMDInfo& msg);
void convertQuaternionToMsg(const OVR::Quatf& quaternion,
														geometry_msgs::Quaternion& msg);
}  // namespace oculus_ros

#endif  // __OCULUS_ROS_UTIL__
