# Camera Calibration

## Maintainer

- Thiemo Wiedemeyer <<wiedemeyer@cs.uni-bremen.de>>, University of Bremen, Institute for Artificial Intelligence

## Description

This tool uses OpenCV to calibrate two cameras to each other. It is specially designed for the Kinect  One. It uses chess or circle boards.

## Dependencies

- ROS Hydro/Indigo
- OpenCV

*for the ROS packages look at the package.xml*

## Usage

```
camera_calibration [options]
  mode: 'record' or 'calibrate'
  source: 'color', 'ir', 'sync'
  board: 'circle<WIDTH>x<HEIGHT>x<SIZE>' or 'chess<WIDTH>x<HEIGHT>x<SIZE>'
  topics: '-color <TOPIC>' and/or '-ir <TOPIC>'
  output path: '<PATH>'
```
