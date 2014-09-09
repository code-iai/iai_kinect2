# IAI Kinect2

## Maintainer

- Thiemo Wiedemeyer <<wiedemeyer@cs.uni-bremen.de>>, University of Bremen, [Institute for Artificial Intelligence](http://ai.uni-bremen.de/)

## Description

This is a collection of tools and libraries for a ROS Interface to the Kinect One (Kinect v2).

It contains:
- [a calibration tool](https://github.com/code-iai/iai_kinect2/tree/master/camera_calibration) for calibrating the IR sensor of the Kinect One to the RGB sensor
- [a library](https://github.com/code-iai/iai_kinect2/tree/master/depth_registration) for depth registration with OpenCL support
- [the bridge](https://github.com/code-iai/iai_kinect2/tree/master/kinect2_bridge) between [libfreenect2](https://github.com/OpenKinect/libfreenect2) and [ROS](http://www.ros.org/)
- [a viewer](https://github.com/code-iai/iai_kinect2/tree/master/registration_viewer) for the images / point clouds

## Screenshots

Here are some screenshots from our toolkit:
![color image](http://ai.uni-bremen.de/wiki/_media/software/kinect2_color.jpg)
![depth image](http://ai.uni-bremen.de/wiki/_media/software/kinect2_depth_colored.png)
![point cloud](http://ai.uni-bremen.de/wiki/_media/software/kinect2_cloud.png)
![image viewer](http://ai.uni-bremen.de/wiki/_media/software/kinect2_viewer.png)

