# IAI Kinect2

## Maintainer

- [Thiemo Wiedemeyer](https://ai.uni-bremen.de/team/thiemo_wiedemeyer) <<wiedemeyer@cs.uni-bremen.de>>, [Institute for Artificial Intelligence](http://ai.uni-bremen.de/), University of Bremen

## Description

This is a collection of tools and libraries for a ROS Interface to the Kinect One (Kinect v2).

It contains:
- [a calibration tool](https://github.com/code-iai/iai_kinect2/tree/master/camera_calibration) for calibrating the IR sensor of the Kinect One to the RGB sensor
- [a library](https://github.com/code-iai/iai_kinect2/tree/master/depth_registration) for depth registration with OpenCL support
- [the bridge](https://github.com/code-iai/iai_kinect2/tree/master/kinect2_bridge) between [libfreenect2](https://github.com/OpenKinect/libfreenect2) and [ROS](http://www.ros.org/)
- [a viewer](https://github.com/code-iai/iai_kinect2/tree/master/registration_viewer) for the images / point clouds

## Dependencies from all parts

- ROS Hydro/Indigo
- OpenCV
- PCL
- Eigen (optional)
- OpenCL (optional)
- libfreenect2 (either directly form [this fork](https://github.com/wiedemeyer/libfreenect2) or with the changes of [this pull-request](https://github.com/OpenKinect/libfreenect2/pull/48) and [this pull-request](https://github.com/OpenKinect/libfreenect2/pull/47).)

## Install

1. Install the dependencies. *Note that a modified version of libfreenect2 is needed*
2. Clone this repository into your catkin workspace.
3. Build it.
4. Connect your sensor and run `kinect2_bridge`.
5. Calibrate your sensor using the `camera_calibration`. [Further details](https://github.com/code-iai/iai_kinect2/tree/master/camera_calibration#calibrating-the-kinect-one)
6. Add the calibration files to the `kinect2_bridge/data/<serialnumber>` folder. [Further details](https://github.com/code-iai/iai_kinect2/tree/master/kinect2_bridge#first-steps)
7. Restart `kinect2_bridge` and view the results using `rosrun registration_viewer viewer -kinect2 -cloud`.

## Permissions to access the Kinect One

To gain access to the Kinect One for non root users you have to add a rule to the udev rules.

1. Create a file named `90-kinect2.rules` in `/etc/udev/rules.d/`.
2. Write the following lines into that file:

  ```
# ATTR{product}=="Kinect2"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02c4", MODE="0666"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02d8", MODE="0666"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02d9", MODE="0666"
```
3. Check if the `idProduct` of your sensor is in the list. If not just add another line with the `idProduct` of your sensor. You can obtain it by running `dmesg | grep "045e"`.
4. Reconnect the sensor and you should be able to access it.

## Screenshots

Here are some screenshots from our toolkit:
![color image](http://ai.uni-bremen.de/wiki/_media/software/kinect2_color.jpg)
![depth image](http://ai.uni-bremen.de/wiki/_media/software/kinect2_depth_colored.png)
![point cloud](http://ai.uni-bremen.de/wiki/_media/software/kinect2_cloud.png)
![image viewer](http://ai.uni-bremen.de/wiki/_media/software/kinect2_viewer.png)

