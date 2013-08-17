#ifndef __OCULUS_DRIVER_UTIL__
#define __OCULUS_DRIVER_UTIL__

#include <OVR.h>
#include <oculus_msgs/HMDInfo.h>
#include <geometry_msgs/Quaternion.h>

namespace oculus_driver {

void convertHMDInfoToMsg(const OVR::HMDInfo& info, oculus_msgs::HMDInfo& msg);
void convertQuaternionToMsg(const OVR::Quatf& quaternion,
														geometry_msgs::Quaternion& msg);
}  // namespace oculus_driver

#endif  // __OCULUS_DRIVER_UTIL__
