#include <oculus_driver/util.h>

namespace oculus_driver {

void convertHMDInfoToMsg(const OVR::HMDInfo& info, oculus_msgs::HMDInfo& msg) {
  msg.display_device_name = info.DisplayDeviceName;
  msg.product_name = info.ProductName;
  msg.manufacturer = info.Manufacturer;
  msg.version = info.Version;
  msg.horizontal_resolution = info.HResolution;
  msg.vertical_resolution = info.VResolution;
  msg.horizontal_screen_size = info.HScreenSize;
  msg.vertical_screen_size = info.VScreenSize;
  msg.vertical_screen_center = info.VScreenCenter;
  msg.eye_to_screen_distance = info.EyeToScreenDistance;
  msg.lens_separation_distance = info.LensSeparationDistance;
  msg.interpupillary_distance = info.InterpupillaryDistance;
  msg.distortion_K.push_back(info.DistortionK[0]);
  msg.distortion_K.push_back(info.DistortionK[1]);
  msg.distortion_K.push_back(info.DistortionK[2]);
  msg.desktop_x = info.DesktopX;
  msg.desktop_y = info.DesktopY;
  msg.display_id = info.DisplayId;
}

void convertQuaternionToMsg(const OVR::Quatf& quaternion,
			    geometry_msgs::Quaternion& msg) {
  msg.x = quaternion.x;
  msg.y = quaternion.y;
  msg.z = quaternion.z;
  msg.w = quaternion.w;
}

}  // namespace oculus_ros
