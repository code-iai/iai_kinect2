# Kinect2 Bridge

## Maintainer

- [Thiemo Wiedemeyer](https://ai.uni-bremen.de/team/thiemo_wiedemeyer) <<wiedemeyer@cs.uni-bremen.de>>, [Institute for Artificial Intelligence](http://ai.uni-bremen.de/), University of Bremen

## Description

This is a bridge between [libfreenect2](https://github.com/OpenKinect/libfreenect2) and ROS.

### Highlights

- delivers up to 30 frames per second on non high end hardware
- delivers up to 30 frames per second over gigabit ethernet
- support for compressed image transport
- utilizes multiple cores and uses special OpenCL based implementation of the depth registration and libfreenect2

## Dependencies

- ROS Hydro/Indigo
- OpenCV
- libfreenect2 (either directly form [this fork](https://github.com/wiedemeyer/libfreenect2) or with the changes of [this pull-request](https://github.com/OpenKinect/libfreenect2/pull/48) and [this pull-request](https://github.com/OpenKinect/libfreenect2/pull/47).)

*for the ROS packages look at the package.xml*

## First steps

For the depth registration the camera intrinsics and extrinsics need to be known. The program reads in the values from the `data/<serialnumber>` folder. For each new sensor you need to add a subfolder with the serial number of the device as the folder name. In this folder you need to provide 3 yaml files with the intrinsics and extrinsics. These files can be created by the `camera_calibration` tool (or you can copy the files provided in one of the other folders, but results can be sub optimal). The device serial number is shown when `kinect2_bridge` or `Protonect` from libfreenect2 is started, it also appears in `dmesg` when you connect the sensor. [More information on calibration](https://github.com/code-iai/iai_kinect2/tree/master/camera_calibration#calibrating-the-kinect-one).

When `kinect2_bridge` is running you can use the `registration_viewer` to display the images or point cloud: `rosrun registration_viewer viewer -kinect2 -image` or `rosrun registration_viewer viewer -kinect2 -cloud`.

## Topics

### Depth Topics

###### Raw depth image
```
/kinect2_head/depth/camera_info
/kinect2_head/depth/image
/kinect2_head/depth/image/compressedDepth
```

###### Rectified depth image
```
/kinect2_head/depth_rect/camera_info
/kinect2_head/depth_rect/image
/kinect2_head/depth_rect/image/compressedDepth
```

###### Depth image registered to low resolution image (960x540)
```
/kinect2_head/depth_lowres/camera_info
/kinect2_head/depth_lowres/image
/kinect2_head/depth_lowres/image/compressedDepth
```

###### Depth image registered to high resolution image
```
/kinect2_head/depth_highres/camera_info
/kinect2_head/depth_highres/image
/kinect2_head/depth_highres/image/compressedDepth
```

### Infrared Topics

###### Raw ir image
```
/kinect2_head/ir/camera_info
/kinect2_head/ir/image
/kinect2_head/ir/image/compressed
```

###### Rectified ir image
```
/kinect2_head/ir_rect/camera_info
/kinect2_head/ir_rect/image
/kinect2_head/ir_rect/image/compressed
```

### Mono Topics

###### Raw mono image
```
/kinect2_head/mono/camera_info
/kinect2_head/mono/image
/kinect2_head/mono/image/compressed
```

###### Rectified mono image
```
/kinect2_head/mono_rect/camera_info
/kinect2_head/mono_rect/image
/kinect2_head/mono_rect/image/compressed
```

###### Mono image in low resolution (960x540)
```
/kinect2_head/mono_lowres/camera_info
/kinect2_head/mono_lowres/image
/kinect2_head/mono_lowres/image/compressed
```

### Color Topics

###### Raw color image
```
/kinect2_head/rgb/camera_info
/kinect2_head/rgb/image
/kinect2_head/rgb/image/compressed
```

###### Rectified color image
```
/kinect2_head/rgb_rect/camera_info
/kinect2_head/rgb_rect/image
/kinect2_head/rgb_rect/image/compressed
```

###### Color image in low resolution (960x540)
```
/kinect2_head/rgb_lowres/camera_info
/kinect2_head/rgb_lowres/image
/kinect2_head/rgb_lowres/image/compressed
```

## Notes

- Images from the same frame have the same timestamp. Using the `message_filters::sync_policies::ExactTime` policy is recommended.

## Usage

```
kinect2_bridge [options]
  -fps <num>     limit the frames per second to <num> (float)
  -calib <path>  path to the calibration files
  -raw           output raw depth image as 512x424 instead of 960x540
```

## Key bindings

Terminal:
- `CRTL`+`c`: Quit

