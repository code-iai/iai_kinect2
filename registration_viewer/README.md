# Registration Viewer

## Maintainer

- [Thiemo Wiedemeyer](https://ai.uni-bremen.de/team/thiemo_wiedemeyer) <<wiedemeyer@cs.uni-bremen.de>>, [Institute for Artificial Intelligence](http://ai.uni-bremen.de/), University of Bremen

## Description

This is a simple viewer for the combined color an depth image provided by Kinect like depth sensors.

It just listens to two ROS topics and displays a the color with the overlayed colored depth image or a registered point cloud.

## Dependencies

- ROS Hydro/Indigo
- OpenCV
- PCL

*for the ROS packages look at the package.xml*

## Usage

```
viewer [options]
Image topics:
  -depth      ROS topic of depth image
  -color      ROS topic of color image
Visualization:
  -image      displays the depth image overlayed to the color image
  -cloud      displays the point cloud in a PCL visualizer
  -both       displays both of the above
Predefined topics for color and depth:
  -kinect2    topics for the low res depth and color
  -kinect2hd  topics for the high res depth and color
  -pr2        topics for the head mount kinect on pr2
  -xtion      topics for the xtion
```

Example: `rosrun registration_viewer viewer -kinect2 -cloud`

## Key bindings

Windows:
- `ESC`, `q`: Quit
- `SPACE`, `s`: Save the current image, cloud in the current directory

Terminal:
- `CRTL`+`c`: Quit
