#ifndef __OCULUS_ROS_OCULUS_ROS__
#define __OCULUS_ROS_OCULUS_ROS__

#include <OVR.h>
#include <ros/ros.h>

namespace oculus_ros {

class OculusRos {
 public:
	explicit OculusRos(ros::NodeHandle& node);
	virtual bool init();
	virtual void publish();
	virtual ~OculusRos();
 private:
	bool is_info_loaded_;
	ros::NodeHandle node_;
	OVR::Ptr<OVR::DeviceManager> manager_;
	OVR::Ptr<OVR::HMDDevice> hmd_;
	OVR::Ptr<OVR::SensorDevice> sensor_;
	OVR::SensorFusion fusion_result_;
	OVR::HMDInfo info_;
	ros::Publisher pub_;
	ros::Publisher hmd_pub_;
};

}  // namespace oculus_ros

#endif  // __OCULUS_ROS_OCULUS_ROS__
