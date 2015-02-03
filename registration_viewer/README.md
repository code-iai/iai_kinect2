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
  name: 'any string' equals to the kinect2_bridge topic base name
  mode: 'sd', 'hd', or 'ir'
  visualization: 'image', 'cloud' or 'both'
  options:
    'raw' use raw instead of compressed topics
    'approx' use approximate time synchronization
```

Example: `rosrun registration_viewer viewer kinect2 sd cloud`

## Key bindings

Windows:
- `ESC`, `q`: Quit
- `SPACE`, `s`: Save the current image, cloud in the current directory

Terminal:
- `CRTL`+`c`: Quit
