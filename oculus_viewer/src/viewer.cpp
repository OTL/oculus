#include <oculus_viewer/viewer.h>
#include <iostream>

namespace oculus_viewer {

Viewer::Viewer(const std::string& window_name)
  : window_name_(window_name)
  , display_size_(1280, 800) {
}

void Viewer::show(const cv::Mat& right_image,
                  const cv::Mat& left_image) {
  cv::Mat dst_right;
  cv::Mat dst_left;
  const double scale =
      static_cast<double>(display_size_.width/2) / right_image.cols;
  cv::resize(right_image, dst_right, cv::Size(), scale, scale);
  cv::resize(left_image, dst_left, cv::Size(), scale, scale);

  const double centering_offset_y =
      (display_size_.height - dst_right.rows) / 2;
  cv::Mat combined(display_size_.height,
                   display_size_.width,
                   dst_right.type());
  cv::Mat roi_right = 
      combined(cv::Rect(display_offset_x_/2 + display_offset_x_/2,
                        display_offset_y_ + centering_offset_y,
                        dst_right.cols,
                        dst_right.rows));
  dst_right.copyTo(roi_right);
  cv::Mat roi_left =
      combined(cv::Rect(display_size_.width/2 - display_offset_x_/2,
                        centering_offset_y,
                        dst_left.cols,
                        dst_left.rows));
  dst_left.copyTo(roi_left);
  cv::imshow(window_name_, combined);
}

}  // namespace oculus_viewer
