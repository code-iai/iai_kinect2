# IAI Kinect2

## Maintainer

- [Thiemo Wiedemeyer](https://ai.uni-bremen.de/team/thiemo_wiedemeyer) <<wiedemeyer@cs.uni-bremen.de>>, [Institute for Artificial Intelligence](http://ai.uni-bremen.de/), University of Bremen

*Note:* ***Please use the GitHub issues*** *for questions and problems regarding the iai_kinect2 package and its components.* ***Do not write emails.***

## Table of contents
- [Description](#description)
- [FAQ](#faq)
- [Dependencies](#dependencies)
- [Install](#install)
- [OpenCL](#opencl)
  - [OpenCL with AMD](#opencl-with-amd)
  - [OpenCL with Nvidia](#opencl-with-nvidia)
  - [OpenCL with Intel](#opencl-with-intel)
- [Citation](#citation)
- [Screenshots](#screenshots)

## Description

This is a collection of tools and libraries for a ROS Interface to the Kinect One (Kinect v2).

It contains:
- [a calibration tool](kinect2_calibration) for calibrating the IR sensor of the Kinect One to the RGB sensor and the depth measurements
- [a library](kinect2_registration) for depth registration with OpenCL support
- [the bridge](kinect2_bridge) between [libfreenect2](https://github.com/OpenKinect/libfreenect2) and [ROS](http://www.ros.org/)
- [a viewer](kinect2_viewer) for the images / point clouds

## FAQ

#### If I have any question or someting is not working, what should I do first?

First you should look at this FAQ and the [FAQ from libfreenect2](https://github.com/OpenKinect/libfreenect2#faq).
Secondly, look at [issue page from libfreenect2](https://github.com/OpenKinect/libfreenect2/issues) and
the [issue page of iai_kinect2](https://github.com/code-iai/iai_kinect2/issues) for similar issues and solutions.

#### Point clouds are not being published?

Point clouds are only published when the launch file is used. Make sure to start kinect2_bridge with `roslaunch kinect2_bridge kinect2_bridge.launch`.

#### Will it work with OpenCV 3.0

Short answer: No.

Long answer: Yes, it is possible to compile this package with OpenCV 3.0, but it will not work.
This is because cv_bridge is used, which itself is compiled with OpenCV 2.4.x in ROS Indigo/Jade and
linking against both OpenCV versions is not possible. Working support for OpenCV 3.0 might come with a future ROS release.

#### kinect2_bridge is not working / crashing, what is wrong?

There are many reasons why `kinect2_bridge` might not working. The first thing to find out whether the problem is related to `kinect2_bridge` or `libfreenect2`.
A good tool for testing is `Protonect`, it is a binary located in `libfreenect2/build/bin/Protonect`.
It uses libfreenect2 directly with a minimal dependency on other libraries, so it is a good tool for the first tests.

Execute:
- `./Protonect gl` to test OpenGL support.
- `./Protonect cl` to test OpenCL support.
- `./Protonect cpu` to test CPU support.

Before running `kinect2_bridge` please make sure `Protonect` is working and showing color, depth and ir images.
If some of them are black, than there is a problem not related to `kinect2_bridge` and you should look at the issues from the libfreenect2 GitHub page for help.

If one of them works, try out the one that worked with `kinect2_bridge`: `rosrun kinect2_bridge kinect2_bridge _depth_method:=<opengl|opencl|cpu>`.
You can also change the registration method with `_reg_method:=<cpu|opencl>`.

#### Protonect works fine, but kinect2_bridge is still not working / crashing.

If that is the case, you have to make sure that `Protonect` uses the same version of `libfreenect2` as `kinect2_bridge` does.
To do so, run `make` and `sudo make install` in the build folder again. And try out `kinect2_bridge` again.

```
cd libfreenect2/build
make & sudo make install
```

Also make sure that you are not using OpenCV 3.0.

If it is still crashing, compile it in debug and run it with gdb:

```
cd <catkin_ws>
catkin_make -DCMAKE_BUILD_TYPE="Debug"
cd devel/lib/kinect2_bridge
gdb kinect2_bridge
// inside gdb: run until it crashes and do a backtrace
run
bt
quit
```

Open an issue and post the problem description and the output from the backtrace (`bt`).

#### kinect2_bridge hangs and prints "waiting for clients to connect"

This is the normal behavior. 'kinect2_bridge' will only process data when clients are connected (ROS nodes listening to at least one of the topics).
This saves CPU and GPU resources. As soon as you start the `kinect_viewer` or `rostopic hz` on one of the topics, processing should start.

#### rosdep: Cannot locate rosdep definition for [kinect2_bridge] or [kinect2_registration]

`rosdep` will output errors on not being able to locate `[kinect2_bridge]` and `[kinect2_registration]`.
That is fine because they are all part of the iai_kinect2 package and `rosdep` does not know these packages.

#### Protonect or kinect2_bridge outputs [TransferPool::submit] failed to submit transfer

This indicates problems with the USB connection.

#### I still have an issue, what should I do?

First of all, check the issue pages on GitHub for similar issues, as they might contain solutions for them.
By default you will only see the open issues, but if you click on `closed` you will the the ones solved. There is also a search field which helps to find similar issues.

If you found no solution in the issues, feel free to open a new issue for your problem. Please describe your problem in detail and provide error messages and log output.

## Dependencies

- ROS Hydro/Indigo
- OpenCV (2.4.x, using the one from the official Ubuntu repositories is recommended)
- PCL (1.7.x, using the one from the official Ubuntu repositories is recommended)
- Eigen (optional, but recommended)
- OpenCL (optional, but recommended)
- [libfreenect2](https://github.com/OpenKinect/libfreenect2) (for stability checkout the latest stable release)

## Install

1. Install the ROS. [Instructions for Ubuntu 14.04](http://wiki.ros.org/indigo/Installation/Ubuntu)
2. [Setup your ROS environment](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)
3. Install [libfreenect2](https://github.com/OpenKinect/libfreenect2):

   Follow [the instructions](https://github.com/OpenKinect/libfreenect2#debianubuntu-1404) and enable C++11 by using `cmake .. -DENABLE_CXX11=ON` instead of `cmake ..`

   If something is not working, check out the latest stable release, for example `git checkout v0.1.1`.

4. Copy the udev rule file `sudo cp libfreenect2/rules/90-kinect2.rules /etc/udev/rules.d/` and reconnect the sensor
5. Clone this repository into your catkin workspace, install the dependencies and build it:

   ```
cd ~/catkin_ws/src/
git clone https://github.com/code-iai/iai_kinect2.git
cd iai_kinect2
rosdep install -r --from-paths .
cd ~/catkin_ws
catkin_make -DCMAKE_BUILD_TYPE="Release"
```

   *Note: `rosdep` will output errors on not being able to locate `[kinect2_bridge]` and `[depth_registration]`.
   That is fine because they are all part of the iai_kinect2 package and `rosdep` does not know these packages.*

   *Note: If you installed libfreenect2 somewhere else than in `$HOME/freenect2` or a standard location like `/usr/local`
   you have to specify the path to it by adding `-Dfreenect2_DIR=path_to_freenect2/lib/cmake/freenect2` to `catkin_make`.*

6. Connect your sensor and run `kinect2_bridge`:

   ```
roslaunch kinect2_bridge kinect2_bridge.launch
```
7. Calibrate your sensor using the `kinect2_calibration`. [Further details](kinect2_calibration#calibrating-the-kinect-one)
8. Add the calibration files to the `kinect2_bridge/data/<serialnumber>` folder. [Further details](kinect2_bridge#first-steps)
9. Restart `kinect2_bridge` and view the results using `rosrun kinect2_viewer kinect2_viewer kinect2 sd cloud`.

## OpenCL

### OpenCL with AMD

Install the latest version of the AMD Catalyst drivers from https://support.amd.com and `opencl-headers`.

### OpenCL with Nvidia

Install the latest version of the Nvidia drivers,
for example `nvidia-355` and `nvidia-modprobe` from [ppa:graphics-drivers/ppa](https://launchpad.net/~graphics-drivers/+archive/ubuntu/ppa) and `opencl-headers`.

### OpenCL with Intel

You can either install a binary package from a PPA like [ppa:pmjdebruijn/beignet-testing](https://launchpad.net/~pmjdebruijn/+archive/ubuntu/beignet-testing), or build beignet yourself.
It's recommended to use the binary from the PPA.

#### Building Beignet

Download and compile the newest Beignet release from source.

##### Known configuration
- Ubuntu 14.04
- Kernel 3.13 (>= 3.13.0-35-generic) or Kernel 3.16 (needed for the Intel USB 3.0 Controller)
- Beignet v1.0 (http://www.freedesktop.org/wiki/Software/Beignet/)

##### Dependencies for Beignet
For Beignet the following dependencies have to be installed manually:
* ocl-icd-dev
* ocl-icd-libopencl1
* libdrm / libdrm-dev
* llvm-3.5 / llvm-3.5-dev
* clang-3.5 / clang-3.5-dev
* libegl1-mesa-dev
* libedit-dev

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

## Citation

If you used `iai_kinect2` for your work, please cite it.

```tex
@misc{iai_kinect2,
  author = {Wiedemeyer, Thiemo},
  title = {{IAI Kinect2}},
  organization = {Institute for Artificial Intelligence},
  address = {University Bremen},
  year = {2014 -- 2015},
  howpublished = {\url{https://github.com/code-iai/iai\_kinect2}},
  note = {Accessed June 12, 2015}
}
```

The result should look something similar to this (may depend on the bibliography style used):

```
T. Wiedemeyer, “IAI Kinect2,” https://github.com/code-iai/iai_kinect2,
Institute for Artificial Intelligence, University Bremen, 2014 – 2015,
accessed June 12, 2015.
```

## Screenshots

Here are some screenshots from our toolkit:
![color image](http://ai.uni-bremen.de/wiki/_media/software/kinect2_color.jpg)
![depth image](http://ai.uni-bremen.de/wiki/_media/software/kinect2_depth_colored.png)
![point cloud](http://ai.uni-bremen.de/wiki/_media/software/kinect2_cloud.png)
![image viewer](http://ai.uni-bremen.de/wiki/_media/software/kinect2_viewer.png)

