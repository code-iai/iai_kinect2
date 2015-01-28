# Kinect2 Calibration

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
kinect2_calibration [options]
  mode: 'record' or 'calibrate'
  source: 'color', 'ir', 'sync', 'depth'
  board:
    'circle<WIDTH>x<HEIGHT>x<SIZE>'  for symmentric cirle grid
    'acircle<WIDTH>x<HEIGHT>x<SIZE>' for asymmentric cirle grid
    'chess<WIDTH>x<HEIGHT>x<SIZE>'   for chessboard pattern
  topics: '-color <TOPIC>', '-ir <TOPIC>', '-depth <TOPIC>'
  distortion model: 'rational' for using model with 8 instead of 5 coefficients
  output path: '<PATH>'
```

## Key bindings

Windows:
- `ESC`, `q`: Quit
- `SPACE`, `s`: Save the current image for calibration
- `l`: decreas min and max value for IR value rage
- `h`: increas min and max value for IR value rage
- `1`: decreas min value for IR value rage
- `2`: increas min value for IR value rage
- `3`: decreas max value for IR value rage
- `4`: increas max value for IR value rage

Terminal:
- `CRTL`+`c`: Quit

## Calibration patterns

Any chessboard pattern or symmetric or asymmetric circle grid should work. Three different chessboard patterns are located inside the `kinect2_calibration/patterns` folder:
- [chess5x7x0.03.pdf](patterns/chess5x7x0.03.pdf)
- [chess7x9x0.025.pdf](patterns/chess7x9x0.025.pdf)
- [chess9x11x0.02.pdf](patterns/chess9x11x0.02.pdf)

Other patterns are available at OpenCV:
- [Chessboard pattern](http://docs.opencv.org/_downloads/pattern.png)
- [Asymmetric circle grid](http://docs.opencv.org/_downloads/acircles_pattern.png)

## Calibrating the Kinect One

1. Record images for the color camera: `rosrun kinect2_calibration kinect2_calibration chess5x7x0.03 record color`
2. Calibrate the intrinsics: `rosrun kinect2_calibration kinect2_calibration chess5x7x0.03 calibrate color`
3. Redo step 1. and 2. for the infrared camera
4. Record images on both cameras synchronized: `rosrun kinect2_calibration kinect2_calibration chess5x7x0.03 record sync`
5. Calibrate the extrinsics: `rosrun kinect2_calibration kinect2_calibration chess5x7x0.03 calibrate sync`
5. Calibrate the depth measurements: `rosrun kinect2_calibration kinect2_calibration chess5x7x0.03 calibrate depth`

The standard board is a 7x6 0.108m chessboard from the PR2. But any other board can be specified with as parameter. For example a circle board with 8x7 circles in 0.02m distance between them `rosrun kinect2_calibration kinect2_calibration record color circle8x7x0.02`.

## Calibration of the depth measurements

I did some tests on the measured and the computed distance based on the detected chess board. It seems like the Kinect2 has a static offset of around 24 mm. As shown in the following images, one can see, that the difference between measured and computed distance is unrelated to the x and y coordinates of the pixel and also unrelated to the distance.

![plot.png](http://ai.uni-bremen.de/wiki/_media/software/plot.png)
![plot_x.png](http://ai.uni-bremen.de/wiki/_media/software/plot_x.png)
![plot_y.png](http://ai.uni-bremen.de/wiki/_media/software/plot_y.png)
![plot_xy.png](http://ai.uni-bremen.de/wiki/_media/software/plot_xy.png)

For the images above ~400 images of a 4x5x0.03 chessboard in different orientatations, distances and image positions were used. The code for computing the depth offset is added to the calibration tool.

### GNUPlot

The depth calibration creates a file named `plot.dat` inside the calibration folder. This files contains the results of the calibration in 5 columns: x, y, computed depth, measured depth, difference between computed and measured depth.

- Difference between measured/computed distance

  ```
set xlabel "Measured distance"
set ylabel "Computed distance"
plot 'plot.dat' using 3:4 with dots title "Difference between measured/computed distance"
```

- Difference relative to x coordinate

  ```
set xlabel "X"
set ylabel "Distance difference"
plot 'plot.dat' using 1:5 with dots title "Difference relative to X-coordinate"
```

- Difference relative to y coordinate

  ```
set xlabel "Y"
set ylabel "Distance difference"
plot 'plot.dat' using 2:5 with dots title "Difference relative to Y-coordinate"
```

- Difference relative to XY-coordinate

  ```
set xlabel "X"
set ylabel "Y"
set zlabel "Distance difference"
splot 'plot.dat' using 1:2:5 with dots palette title "Difference relative to XY-coordinate"
```

## Example results

Example calibration results can be found in the directory [kinect2_bridge/data/](../kinect2_bridge/data).
