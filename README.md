# IAI Kinect2

## Maintainer

- [Thiemo Wiedemeyer](https://ai.uni-bremen.de/team/thiemo_wiedemeyer) <<wiedemeyer@cs.uni-bremen.de>>, [Institute for Artificial Intelligence](http://ai.uni-bremen.de/), University of Bremen

## Description

This is a collection of tools and libraries for a ROS Interface to the Kinect One (Kinect v2).

It contains:
- [a calibration tool](https://github.com/code-iai/iai_kinect2/tree/master/kinect2_calibration) for calibrating the IR sensor of the Kinect One to the RGB sensor
- [a library](https://github.com/code-iai/iai_kinect2/tree/master/depth_registration) for depth registration with OpenCL support
- [the bridge](https://github.com/code-iai/iai_kinect2/tree/master/kinect2_bridge) between [libfreenect2](https://github.com/OpenKinect/libfreenect2) and [ROS](http://www.ros.org/)
- [a viewer](https://github.com/code-iai/iai_kinect2/tree/master/registration_viewer) for the images / point clouds

## Dependencies from all parts

- ROS Hydro/Indigo
- OpenCV
- PCL
- Eigen (optional)
- OpenCL (optional)
- [libfreenect2](https://github.com/OpenKinect/libfreenect2) or the iai_kinect2 branch of [my fork](https://github.com/wiedemeyer/libfreenect2/tree/iai_kinect2) which already includes the modifications

## Modifications to Upstream libfreenect2

[install_deps.sh](https://github.com/OpenKinect/libfreenect2/blob/master/depends/install_deps.sh):
- Replace [line 17](https://github.com/OpenKinect/libfreenect2/blob/master/depends/install_deps.sh#L17) with `./configure --prefix=$LIBUSB_INSTALL_DIR CFLAGS="$CFLAGS -fPIC"`

[CMakeLists.txt](https://github.com/OpenKinect/libfreenect2/blob/master/examples/protonect/CMakeLists.txt):
- Enable C++11 in [line 8](https://github.com/OpenKinect/libfreenect2/blob/master/examples/protonect/CMakeLists.txt#L8)
- Change [line 102](https://github.com/OpenKinect/libfreenect2/blob/master/examples/protonect/CMakeLists.txt#L102) from 'usb-1.0' to 'usb-1.0.a'
- Replace [line 51 to 57](https://github.com/OpenKinect/libfreenect2/blob/master/examples/protonect/CMakeLists.txt#L51-57) with

  ```
# GLEW
FIND_PACKAGE(GLEW REQUIRED)
INCLUDE_DIRECTORIES(${GLEW_INCLUDE_DIR})
```

## Install

1. Install the ROS. [Instructions for Ubuntu 14.04](http://wiki.ros.org/indigo/Installation/Ubuntu)
2. [Setup your ROS environment](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)
3. Install [libfreenect2](https://github.com/OpenKinect/libfreenect2) with the described modifications or use [my fork](https://github.com/wiedemeyer/libfreenect2/tree/iai_kinect2) which includes the modifications:
   ```
cd ~
sudo apt-get install -y build-essential libturbojpeg libtool autoconf libudev-dev cmake mesa-common-dev freeglut3-dev libxrandr-dev doxygen libxi-dev libopencv-dev
sudo ln -s /usr/lib/x86_64-linux-gnu/libturbojpeg.so.0 /usr/lib/x86_64-linux-gnu/libturbojpeg.so
git clone https://github.com/wiedemeyer/libfreenect2.git
cd libfreenect2/depends
./install_ubuntu.sh
cd ../build
mkdir linux
cd linux
cmake ../../examples/protonect/
make && sudo make installsudo
```
4. Clone this repository into your catkin workspace, install the dependencies and build it:
   ```
git clone https://github.com/code-iai/iai_kinect2.git ~/catkin_ws/src/iai_kinect2
rosdep install -r depth_registration kinect2_bridge kinect2_calibration registration_viewer
cd ~/catkin_ws
catkin_make -DCMAKE_BUILD_TYPE="Release"
```
5. Connect your sensor and run `kinect2_bridge`:
   ```
rosrun kinect2_bridge kinect2_bridge
```
6. Calibrate your sensor using the `kinect2_calibration`. [Further details](https://github.com/code-iai/iai_kinect2/tree/master/kinect2_calibration#calibrating-the-kinect-one)
7. Add the calibration files to the `kinect2_bridge/data/<serialnumber>` folder. [Further details](https://github.com/code-iai/iai_kinect2/tree/master/kinect2_bridge#first-steps)
8. Restart `kinect2_bridge` and view the results using `rosrun registration_viewer viewer -kinect2 -cloud`.

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

## OpenCL with Intel GPU on Linux

#### Known configuration
- Ubuntu 14.04
- Kernel 3.13 (>= 3.13.0-35-generic) or Kernel 3.16 (needed for the Intel USB 3.0 Controller)
- Beignet v1.0 (http://www.freedesktop.org/wiki/Software/Beignet/)

#### Dependencies for Beignet
For Beignet the following depencies have to be installed manually:
* ocl-icd-dev
* ocl-icd-libopencl1
* libdrm / libdrm-dev
* llvm-3.5 / llvm-3.5-dev
* clang-3.5 / clang-3.5-dev
* libegl1-mesa-dev
* libedit-dev

#### Building Beignet
Download and compile the Beignet v1.0 release from source (there is a Beignet_v0.3 binary for Trusty, but it is very old, buggy and slow)

##### Additional steps (if needed):

* Error "clang: not found":

  ```
sudo ln -s /usr/lib/llvm-3.5/bin/clang /usr/bin/clang
```
* Known Beignet issue with Kernel 3.15/3.16 (see Beignet readme); fix is to disable cmd_parser:

  ```
sudo su
echo 0 > /sys/module/i915/parameters/enable_cmd_parser
```

* To get 100% pass rate on the Beignet unit tests you may have to:
  * Execute directly on hw: ssh-session might not work
  * Execute as root

*Note: Both previous points have to to with the fact that no x-server was installed. Apparently this will be fixed in a future release of Beignet.*

#### Results on Intel i7-3840QM (mobile hardware)
* **~100 fps** on the OpenCLDepthPacketProcessor (compared to < *5 fps* on same hardware using CPU-based depth registration!)

  ```
...
[OpenCLDepthPacketProcessor] avg. time: 10.1716ms -> ~98.3129Hz
[TurboJpegRgbPacketProcessor] avg. time: 16.0787ms -> ~62.194Hz
...
```

## Screenshots

Here are some screenshots from our toolkit:
![color image](http://ai.uni-bremen.de/wiki/_media/software/kinect2_color.jpg)
![depth image](http://ai.uni-bremen.de/wiki/_media/software/kinect2_depth_colored.png)
![point cloud](http://ai.uni-bremen.de/wiki/_media/software/kinect2_cloud.png)
![image viewer](http://ai.uni-bremen.de/wiki/_media/software/kinect2_viewer.png)

