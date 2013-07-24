#include <ros/ros.h>
#include <oculus_ros/HMDInfo.h>
#include <oculus_ros/distort.h>
#include "viewer.h"

namespace oculus_ros {

class DistortNode {
 public:
  DistortNode();
  void init();
  void HMDInfoCallback(const oculus_ros::HMDInfoPtr& info);
  void show();
 private:
  DistortImage left_;
  DistortImage right_;
  ros::Subscriber sub_;
  Viewer viewer_;
};

DistortNode::DistortNode()
    : viewer_("oculus camera view") {
}

void DistortNode::init() {
	ros::NodeHandle node;
  left_.init("camera_left/image_raw");
  right_.init("camera_right/image_raw");
  sub_ = node.subscribe("/oculus/hmd_info",
                        1,
                        &oculus_ros::DistortNode::HMDInfoCallback,
                        this);
  ros::NodeHandle private_node("~");
  int32_t offset_x = 0;
  private_node.param<int32_t>("display_offset_x", offset_x, 0);
  int32_t offset_y = 0;
  private_node.param<int32_t>("display_offset_y", offset_y, 0);
  viewer_.setDisplayOffset(offset_x, offset_y);

}

void DistortNode::show() {
  if ((!right_.getImage().empty()) &&
      (!left_.getImage().empty())) {
    viewer_.show(right_.getImage(), left_.getImage());
  }
}

void DistortNode::HMDInfoCallback(const oculus_ros::HMDInfoPtr& info) {
  if (info->horizontal_screen_size > 0) {
    double lens_center = 1 - 2 * info->lens_separation_distance / info->horizontal_screen_size;
    double scale = 1 + lens_center;
    left_.setK(info->distortion_K);
    left_.setScale(scale);
    left_.setOffset(-lens_center);
    right_.setK(info->distortion_K);
    right_.setScale(scale);
    right_.setOffset(lens_center);
  }
}

}  // namespace oculus_ros


int main(int argc, char** argv) {
	ros::init(argc, argv, "oculus_distort_node");
  try {
    oculus_ros::DistortNode dis;
    dis.init();
    while(ros::ok()) {
      cv::waitKey(10);
      ros::spinOnce();
      dis.show();
    }
  } catch(ros::Exception& e) {
    ROS_ERROR("ros error: %s", e.what());
  } catch(...) {
    ROS_FATAL("unexpected error");
  }
}

