# Camera Calibration

## Maintainer

- [Thiemo Wiedemeyer](https://ai.uni-bremen.de/team/thiemo_wiedemeyer) <<wiedemeyer@cs.uni-bremen.de>>, [Institute for Artificial Intelligence](http://ai.uni-bremen.de/), University of Bremen

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
  board:
    'circle<WIDTH>x<HEIGHT>x<SIZE>'  for symmentric cirle grid
    'acircle<WIDTH>x<HEIGHT>x<SIZE>' for asymmentric cirle grid
    'chess<WIDTH>x<HEIGHT>x<SIZE>'   for chessboard pattern
  topics: '-color <TOPIC>' and/or '-ir <TOPIC>'
  output path: '<PATH>'
```

## Key bindings

Windows:
- `ESC`, `q`: Quit
- `SPACE`, `s`: Save the current image for calibration

Terminal:
- `CRTL`+`c`: Quit

## Calibration patterns

Any chessboard pattern or symmetric or asymmetric circle grid should work. If you need a calibration pattern , you can use the ones provided by OpenCV:
- [Chessboard pattern](http://docs.opencv.org/_downloads/pattern.png)
- [Asymmetric circle grid](http://docs.opencv.org/_downloads/acircles_pattern.png)

## Calibrating the Kinect One

1. Record images for the color camera: `rosrun camera_calibration camera_calibration record color`
2. Calibrate the intrinsics: `rosrun camera_calibration camera_calibration calibrate color`
3. Redo step 1. and 2. for the infrared camera
4. Record images on both cameras synchronized: `rosrun camera_calibration camera_calibration record sync`
4. Calibrate the extrinsics: `rosrun camera_calibration camera_calibration calibrate sync`

The standard board is a 7x6 0.108m chessboard from the PR2. But any other board can be specified with as parameter. For example a circle board with 8x7 circles in 0.02m distance between them `rosrun camera_calibration camera_calibration record color circle8x7x0.02`.

## Example results

Example calibration results can be found in the directory [kinect2_bridge/data/](https://github.com/code-iai/iai_kinect2/tree/master/kinect2_bridge/data).
