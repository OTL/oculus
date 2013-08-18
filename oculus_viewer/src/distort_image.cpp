#include <oculus_viewer/distort.h>
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp> // tmp
#include <sensor_msgs/image_encodings.h>

namespace enc = sensor_msgs::image_encodings;

namespace oculus_viewer {

IplImage* barrel_dist(IplImage* img, double Cx, double Cy,
                      double k0, double k1, double k2) {
  IplImage* mapx = cvCreateImage( cvGetSize(img), IPL_DEPTH_32F, 1 );
  IplImage* mapy = cvCreateImage( cvGetSize(img), IPL_DEPTH_32F, 1 );
  
  int w= img->width;
  int h= img->height;
  
  float* pbuf = (float*)mapx->imageData;
  const float unit_xr2 = (w - Cx) * (w - Cx);
  for (int y = 0; y < h; y++) {
    for (int x = 0; x < w; x++) {
      float r2 = (x-Cx)*(x-Cx)+(y-Cy)*(y-Cy);
      r2 /= unit_xr2;
      const float r4 = r2 * r2;
      *pbuf = Cx + (x - Cx) * (k0 + k1 * r2 + k2 * r4);
      ++pbuf;
    }
  }
  
  pbuf = (float*)mapy->imageData;
  const float unit_yr2 = (h -Cy) * (h -Cy);
  for (int y = 0;y < h; y++) {
    for (int x = 0; x < w; x++) {
      float r2 = (x-Cx)*(x-Cx)+(y-Cy)*(y-Cy);
      r2 /= unit_yr2;
      const float r4 = r2 * r2;
      *pbuf = Cy + (y - Cy) * (k0 + k1 * r2 + k2 * r4);
      ++pbuf;
    }
  }

  IplImage* temp = cvCloneImage(img);
  cvRemap( temp, img, mapx, mapy ); 
  cvReleaseImage(&temp);
  cvReleaseImage(&mapx);
  cvReleaseImage(&mapy);
  
  return img;
}

DistortImage::DistortImage()
  : nh_()
  , it_(nh_)
  , offset_(0.0)
  , scale_(1.0) {
}
  
void DistortImage::init(const std::string& topic_name) {
  ros::NodeHandle node;
  try {
    pub_ = it_.advertise(topic_name + "/distorted", 1);
    sub_ = it_.subscribe(topic_name, 1, &DistortImage::imageCb, this);
  } catch(ros::Exception& e) {
    ROS_ERROR("init ROS error: %s", e.what());
  } catch(image_transport::TransportLoadException& e) {
    ROS_ERROR("image_transport error: %s", e.what());
  } catch(...) {
    ROS_ERROR("some error");
  }
}

void DistortImage::imageCb(const sensor_msgs::ImageConstPtr& msg) {
  cv_bridge::CvImagePtr ptr;
  try {
    ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
  IplImage img = ptr->image;
  if (K_.size() > 2) {
    barrel_dist(&img,
                (msg->width / 2 * (1.0 + offset_)), msg->height / 2,
                K_.at(0), K_.at(1), K_.at(2));
    cv::Mat resized_image;
    cv::resize(ptr->image, resized_image, cv::Size(), scale_, scale_);
    ptr->image = resized_image(
        cv::Rect((resized_image.cols - ptr->image.cols) / 2,
                 (resized_image.rows - ptr->image.rows) /2,
                 ptr->image.cols,
                 ptr->image.rows));
    img_ = ptr->image;
    pub_.publish(ptr->toImageMsg());
  }
}

}  // namespace oculus_viewer
