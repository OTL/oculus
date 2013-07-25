#include <oculus_ros/oculus_ros.h>
#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "oculus_ros_node");
  ros::NodeHandle node;
  oculus_ros::OculusRos oculus(node);
  ros::NodeHandle local_node("~");
  double frequency(10.0);
  local_node.getParam("frequency", frequency);
  ros::Rate r(frequency);
  if(oculus.init()) {
    while(ros::ok()) {
      oculus.publish();
      r.sleep();
    }
  } else {
    ROS_ERROR("Oculus Rift not found");
  }
}
