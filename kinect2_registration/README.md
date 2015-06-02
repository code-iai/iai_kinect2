# Kinect2 Registration

## Maintainer

- [Thiemo Wiedemeyer](https://ai.uni-bremen.de/team/thiemo_wiedemeyer) <<wiedemeyer@cs.uni-bremen.de>>, [Institute for Artificial Intelligence](http://ai.uni-bremen.de/), University of Bremen

## Description

This is a library for projecting the depth image obtained by Kinect like sensors to a color image. It has a OpenCL implementation for registering the depth image, to reduce CPU load.

## Dependencies

- ROS Hydro/Indigo
- OpenCV
- Eigen (optional)
- OpenCL (optional)

At least one of OpenCL or Eigen has to be installed. If OpenCL is not installed the CPU will be used. For optimal performance OpenCL is recommended.

*for the ROS packages look at the package.xml*

