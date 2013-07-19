#ifndef __OCULUS_ROS_OCULUS_ROS__
#define __OCULUS_ROS_OCULUS_ROS__

#include <OVR.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

namespace oculus_ros {

class OculusRos {
 public:
	explicit OculusRos(ros::NodeHandle& node);
	virtual bool init();
	virtual void publish();
	virtual ~OculusRos();
 private:
	bool is_info_loaded_;
	std::string parent_frame_;
	std::string oculus_frame_;
	ros::NodeHandle node_;
	OVR::Ptr<OVR::DeviceManager> manager_;
	OVR::Ptr<OVR::HMDDevice> hmd_;
	OVR::Ptr<OVR::SensorDevice> sensor_;
	OVR::SensorFusion fusion_result_;
	OVR::HMDInfo info_;
	ros::Publisher pub_;
	ros::Publisher hmd_pub_;
	tf::TransformBroadcaster br_;
};

}  // namespace oculus_ros

#endif  // __OCULUS_ROS_OCULUS_ROS__
