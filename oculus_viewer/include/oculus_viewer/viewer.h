#ifndef __OCULUS_VIEWER_VIEWER_H__
#define __OCULUS_VIEWER_VIEWER_H__

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>

namespace oculus_viewer {

class Viewer {
 public:
  explicit Viewer(const std::string& window_name);
  void show(const cv::Mat& right_image, const cv::Mat& left_image);
  void setDisplaySize(const cv::Size& size) { display_size_ = size; }
  void setDisplayOffset(int32_t x, int32_t y) {
    display_offset_x_ = x;
    display_offset_y_ = y;
  }
 private:
  std::string window_name_;
  cv::Size display_size_;
  int32_t display_offset_x_;
  int32_t display_offset_y_;
};

}  // namespace oculus_viewer

#endif  // __OCULUS_VIEWER_VIEWER_H__
