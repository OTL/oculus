#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

namespace oculus_ros {

IplImage* barrel_dist(IplImage* img, double Cx, double Cy,
                      double kx, double ky);

class DistortImage {
 public:
	DistortImage();
  void init(const std::string& topic_name);
  const std::vector<float>& getK() const {
    return K_;
  }
  void setK(const std::vector<float>& K) {
    K_ = K;
  }
  void imageCb(const sensor_msgs::ImageConstPtr& msg);
 private:
  ros::NodeHandle nh_;
	image_transport::Subscriber sub_;
	image_transport::Publisher pub_;
  image_transport::ImageTransport it_;
  std::vector<float> K_;
};

}  // namespace oculus_ros
