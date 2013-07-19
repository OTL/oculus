#include <ros/ros.h>
#include <oculus_ros/HMDInfo.h>
#include <oculus_ros/distort.h>

namespace oculus_ros {

class DistortNode {
 public:
  DistortNode();
  void init();
  void HMDInfoCallback(const oculus_ros::HMDInfoPtr& info);
 private:
  DistortImage left_;
  DistortImage right_;
  ros::Subscriber sub_;  
};

DistortNode::DistortNode() {
}

void DistortNode::init() {
	ros::NodeHandle node;
  left_.init("left/image");
  right_.init("right/image");
  sub_ = node.subscribe("/oculus/hmd_info",
                        1,
                        &oculus_ros::DistortNode::HMDInfoCallback,
                        this);
}

void DistortNode::HMDInfoCallback(const oculus_ros::HMDInfoPtr& info) {
  left_.setK(info->distortion_K);
  right_.setK(info->distortion_K);
}

}  // namespace oculus_ros


int main(int argc, char** argv) {
	ros::init(argc, argv, "oculus_distort_node");
  try {
    oculus_ros::DistortNode dis;
    dis.init();
    ros::spin();
  } catch(ros::Exception& e) {
    ROS_ERROR("ros error: %s", e.what());
  } catch(...) {
    ROS_FATAL("unexpected error");
  }
}

