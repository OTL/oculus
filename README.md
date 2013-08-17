ROS node for Oculus Rift
=========================
This is [ROS](http://ros.org) driver for [Oculus Rift](http://www.oculusvr.com).
see http://developer.oculusvr.com for Oculus Rift SDK.

install
-----------------
Please copy OculusSDK/LibOVR to this directory and build it in Debug mode.
(If you are using OSX, please enable rtti and exceptions.)
If you want to use Relase mode, please edit CMakeLists.txt.

Supported OS
-----------------
This package supports OSX and Linux.

oculus_node
------------------
publishes sensor data and HMD information of Oculus Rift.

### publish

* /oculus/orientaion (geometry_msgs/Quaternion) orientation of sensor.
* /oculus/hmd_info (oculus_ros/HMDInfo) HMD device info.
* /tf

### param
* ~frequency (double: default 10.0) [Hz] rate of publish
* ~parent_frame (string: default parent) tf frame name of parent
* ~oculus_frame (string: default oculus) tf frame name of oculus

image_distort_viewer
-------------------
distort images by /oculus/hmd_info params and display it.

### Subscribe
* /camera/left/image_raw (sensor_msgs/Image)
* /camera/right/image_raw (sensor_msgs/Image)
* /oculus/hmd_info (oculus_ros/HMDInfo) HMD device info.

### Publish
* /camera/left/image_raw/distorted (sensor_msgs/Image) left distorted image
* /camera/right/image_raw/distorted (sensor_msgs/Image) right distorted image

### param
* ~image_transport (string: default "raw") this node uses image_transport for subscribe.
* ~use_display (bool: default true) true: show distored and combined image in GUI. false: publish distored image only.

License
-----------
BSD
