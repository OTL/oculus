#ifndef __OCULUS_DRIVER_OCULUS_ROS__
#define __OCULUS_DRIVER_OCULUS_ROS__

#include <OVR.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

namespace oculus_driver
{

    class OculusRos
    {
        public:
            explicit OculusRos(ros::NodeHandle& node);
            virtual bool init();
            virtual void publish();
            virtual ~OculusRos();
        private:
            bool is_info_loaded;
            std::string parent_frame;
            std::string oculus_frame;
            ros::NodeHandle node;
            OVR::Ptr<OVR::DeviceManager> manager;
            OVR::Ptr<OVR::HMDDevice> hmd;
            OVR::Ptr<OVR::SensorDevice> sensor;
            OVR::SensorFusion* fusion_result;
            OVR::HMDInfo info;
            ros::Publisher pub;
            ros::Publisher hmd_pub;
            tf::TransformBroadcaster br;
    };

}  // namespace oculus_driver

#endif  // __OCULUS_DRIVER_OCULUS_ROS__
