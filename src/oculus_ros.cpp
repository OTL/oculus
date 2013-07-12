#include <oculus_ros/oculus_ros.h>
#include <oculus_ros/HMDInfo.h>
#include <oculus_ros/util.h>

namespace oculus_ros {

OculusRos::OculusRos(ros::NodeHandle& node)
	: is_info_loaded_(false)
	, node_(node) {
}

bool OculusRos::init() {
	OVR::System::Init();

	manager_ = *OVR::DeviceManager::Create();
	hmd_ = *manager_->EnumerateDevices<OVR::HMDDevice>().CreateDevice();

	if (hmd_) {
		is_info_loaded_ = hmd_->GetDeviceInfo(&info_);
		sensor_ = *hmd_->GetSensor();
		hmd_pub_ = node_.advertise<oculus_ros::HMDInfo>("/oculus/hmd_info", 10);
	} else {
		sensor_ = *manager_->EnumerateDevices<OVR::SensorDevice>().CreateDevice();
	}

	if (sensor_) {
		fusion_result_.AttachToSensor(sensor_);
		pub_ = node_.advertise<geometry_msgs::Quaternion>("/oculus/orientation", 10);
	}
	return is_info_loaded_ || sensor_;
}

OculusRos::~OculusRos() {
	sensor_.Clear();
	hmd_.Clear();
	manager_.Clear();
	OVR::System::Destroy();
}

void OculusRos::publish() {
	if (is_info_loaded_) {
		oculus_ros::HMDInfo hmd_msg;
		convertHMDInfoToMsg(info_, hmd_msg);
		hmd_pub_.publish(hmd_msg);
	}
	if (sensor_) {
		geometry_msgs::Quaternion q_msg;
		convertQuaternionToMsg(fusion_result_.GetOrientation(), q_msg);
		pub_.publish(q_msg);
	}
}

} 	// namespace oculus_ros
