# Kinect2 Viewer

## Maintainer

- [Nishanth Koganti](https://buntyke.github.io) <<buntyke@gmail.com>>, Kyushu Institute of Technology

## Description

This package contains programs to record the color and depth image provided by Kinect V2 depth sensor.

It subscribes to three ROS topics and records these three topics to a rosbag file.

## Dependencies

- ROS Hydro/Indigo
- OpenCV
- ROS Bag

*for the ROS packages look at the package.xml*

## Usage

```
kinect2_recorder [options]
  name: 'any string' equals to the rosbag filename
  mode: 'qhd', 'hd', 'sd' or 'ir'
  options:
    'compressed' use compressed instead of raw topics
    'approx' use approximate time synchronization
```

Example: `rosrun kinect2_recorder kinect2_recorder kinect.bag sd`

```
kinect2_processor [options]
  name: 'any string' equals to the rosbag filename
  mode: 'qhd', 'hd', 'sd' or 'ir'
```

Example: `rosrun kinect2_processor kinect2_processor kinect.bag sd`


## Key bindings

Windows for kinect2_processor:
- `ESC`, `q`: Quit

Terminal for kinect2_recorder:
- `CRTL`+`c`: Quit
