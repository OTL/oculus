ROS node for Oculus Rift
=========================
This is [ROS](http://ros.org) driver for [Oculus Rift](http://www.oculusvr.com).
see http://developer.oculusvr.com for Oculus Rift SDK.

Install
-----------------
Please copy OculusSDK/LibOVR to oculus_driver/ directory and build it in Debug mode.
(If you are using OSX, please enable rtti and exceptions.)
If you want to use Release mode, please edit CMakeLists.txt of oculus_driver/.

Supported OS
-----------------
This package supports OSX (10.7.5 and MacPorts) and Linux (Ubuntu12.04).

Packages
------------------
* oculus_driver: oculus rift HMD ROS driver.
* oculus_msgs: oculus HMD message definitions.
* oculus_viewer: viewer for stereo cameras.

Nodes
=============

oculus_node (in oculus_driver package)
------------------
publishes sensor data and HMD information of Oculus Rift.

### publish

* /oculus/orientation (geometry_msgs/Quaternion) orientation of sensor.
* /oculus/hmd_info (oculus_ros/HMDInfo) HMD device info.
* /tf

### param
* ~frequency (double: default 10.0) [Hz] rate of publish
* ~parent_frame (string: default parent) tf frame name of parent
* ~oculus_frame (string: default oculus) tf frame name of oculus

image_distort_viewer (in oculus_viewer package)
-------------------
Distort images by /oculus/hmd_info params and display it.
You have to adjust some params for your stereo camera manually.
A sample launch file is in oculus_viewer/launch/oculus_sample.launch.

### Subscribe
* /camera/left/image_raw (sensor_msgs/Image)
* /camera/right/image_raw (sensor_msgs/Image)
* /oculus/hmd_info (oculus_ros/HMDInfo) HMD device info.

### Publish
* /camera/left/image_raw/distorted (sensor_msgs/Image) left distorted image
* /camera/right/image_raw/distorted (sensor_msgs/Image) right distorted image

### param
* ~image_transport (string: default "raw") this node uses image_transport for subscribe. If you are transporting remote images, you should use "compressed".
* ~use_display (bool: default true) true: show distored and combined image in GUI. false: publish distored image only.

License
-----------
BSD
