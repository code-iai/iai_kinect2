/**
 * Copyright 2014 University of Bremen, Institute for Artificial Intelligence
 * Author: Thiemo Wiedemeyer <wiedemeyer@cs.uni-bremen.de>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <chrono>
#include <sys/stat.h>

#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/image_encodings.h>

#include <tf/transform_broadcaster.h>

#include <compressed_depth_image_transport/compression_common.h>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/config.h>

#include <kinect2_definitions.h>
#include <depth_registration.h>

class Kinect2Bridge
{
private:
  std::vector<int> compressionParams;
  std::string compression16BitExt, compression16BitString, ns, baseNameTF;

  cv::Size sizeColor, sizeIr, sizeLowRes;
  cv::Mat color, ir, depth;
  cv::Mat cameraMatrixColor, distortionColor, cameraMatrixLowRes, cameraMatrixIr, distortionIr;
  cv::Mat rotation, translation;
  cv::Mat map1Color, map2Color, map1Ir, map2Ir, map1LowRes, map2LowRes;

  std::vector<std::thread> threads;
  std::mutex lockIrDepth, lockColor;
  std::mutex lockSync, lockPub, lockTime;
  std::mutex lockRegLowRes, lockRegHighRes;

  bool publishTF;
  std::thread tfPublisher;

  libfreenect2::Freenect2 freenect2;
  libfreenect2::Freenect2Device *device;
  libfreenect2::SyncMultiFrameListener *listenerColor, *listenerIrDepth;
  libfreenect2::PacketPipeline *packetPipeline;

  ros::NodeHandle nh;

  DepthRegistration *depthRegLowRes, *depthRegHighRes;

  size_t frameColor, frameIrDepth, pubFrameColor, pubFrameIrDepth;
  ros::Time lastColor, lastDepth;

  bool nextColor, nextIrDepth;
  double deltaT, depthShift, elapsedTimeColor, elapsedTimeIrDepth;
  bool running;

  enum Image
  {
    IR = 0,
    IR_RECT,

    DEPTH,
    DEPTH_RECT,
    DEPTH_LORES,
    DEPTH_HIRES,

    COLOR,
    COLOR_RECT,
    COLOR_LORES,

    MONO,
    MONO_RECT,
    MONO_LORES,

    COUNT
  };

  enum Status
  {
    UNSUBCRIBED = 0,
    RAW,
    COMPRESSED,
    BOTH
  };

  std::vector<ros::Publisher> imagePubs, compressedPubs, infoPubs;
  std::vector<sensor_msgs::CameraInfo> infos;

public:
  Kinect2Bridge(const ros::NodeHandle &nh = ros::NodeHandle("~"))
    : sizeColor(1920, 1080), sizeIr(512, 424), sizeLowRes(sizeColor.width / 2, sizeColor.height / 2), nh(nh), frameColor(0), frameIrDepth(0),
      pubFrameColor(0), pubFrameIrDepth(0), lastColor(0, 0), lastDepth(0, 0), nextColor(false), nextIrDepth(false), depthShift(0), running(false)
  {
    color = cv::Mat::zeros(sizeColor, CV_8UC3);
    ir = cv::Mat::zeros(sizeIr, CV_32F);
    depth = cv::Mat::zeros(sizeIr, CV_32F);
  }

  void run()
  {
    if(!initialize())
    {
      return;
    }
    running = true;

    if(publishTF)
    {
      tfPublisher = std::thread(&Kinect2Bridge::publishStaticTF, this);
    }

    for(size_t i = 0; i < threads.size(); ++i)
    {
      threads[i] = std::thread(&Kinect2Bridge::threadDispatcher, this, i);
    }

    std::cout << "starting main loop" << std::endl << std::endl;
    double nextFrame = ros::Time::now().toSec() + deltaT;
    double fpsTime = ros::Time::now().toSec();
    size_t oldFrameIrDepth = 0, oldFrameColor = 0;
    nextColor = true;
    nextIrDepth = true;

    for(; running && ros::ok();)
    {
      double now = ros::Time::now().toSec();

      if(now - fpsTime >= 3.0)
      {
        fpsTime = now - fpsTime;
        size_t framesIrDepth = frameIrDepth - oldFrameIrDepth;
        size_t framesColor = frameColor - oldFrameColor;
        oldFrameIrDepth = frameIrDepth;
        oldFrameColor = frameColor;

        lockTime.lock();
        double tColor = elapsedTimeColor;
        double tDepth = elapsedTimeIrDepth;
        elapsedTimeColor = 0;
        elapsedTimeIrDepth = 0;
        lockTime.unlock();

        std::cout << "[kinect2_bridge] depth processing: ~" << framesIrDepth / tDepth << "Hz (" << (tDepth / framesIrDepth) * 1000 << "ms) publishing rate: ~" << framesIrDepth / fpsTime << "Hz" << std::endl
                  << "[kinect2_bridge] color processing: ~" << framesColor / tColor << "Hz (" << (tColor / framesColor) * 1000 << "ms) publishing rate: ~" << framesColor / fpsTime << "Hz" << std::endl << std::flush;
        fpsTime = now;
      }

      if(now >= nextFrame)
      {
        nextColor = true;
        nextIrDepth = true;
        nextFrame += deltaT;
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    for(size_t i = 0; i < threads.size(); ++i)
    {
      threads[i].join();
    }
    threads.clear();

    if(publishTF)
    {
      tfPublisher.join();
    }

    device->stop();
    device->close();
    delete listenerIrDepth;
    delete listenerColor;

    for(size_t i = 0; i < COUNT; ++i)
    {
      imagePubs[i].shutdown();
      compressedPubs[i].shutdown();
      infoPubs[i].shutdown();
    }

    nh.shutdown();
  }

  void stop()
  {
    running = false;
  }

private:
  bool initialize()
  {
    double fps_limit, maxDepth, minDepth;
    bool use_png, bilateral_filter, edge_aware_filter;
    int32_t jpeg_quality, png_level, queueSize, reg_dev, depth_dev, worker_threads;
    double tmp;
    std::string depth_method, reg_method, calib_path, sensor;

    std::string depthDefault = "cpu";
    std::string regDefault = "default";

#ifdef LIBFREENECT2_WITH_OPENGL_SUPPORT
    depthDefault = "opengl";
#endif
#ifdef LIBFREENECT2_WITH_OPENCL_SUPPORT
    depthDefault = "opencl";
#endif
#ifdef DEPTH_REG_OPENCL
    regDefault = "opencl";
#endif

    nh.param("base_name", ns, std::string(K2_DEFAULT_NS));
    nh.param("sensor", tmp, -1.0);
    nh.param("fps_limit", fps_limit, -1.0);
    nh.param("calib_path", calib_path, std::string(K2_CALIB_PATH));
    nh.param("use_png", use_png, false);
    nh.param("jpeg_quality", jpeg_quality, 90);
    nh.param("png_level", png_level, 1);
    nh.param("depth_method", depth_method, depthDefault);
    nh.param("depth_device", depth_dev, -1);
    nh.param("reg_method", reg_method, regDefault);
    nh.param("reg_devive", reg_dev, -1);
    nh.param("max_depth", maxDepth, 12.0);
    nh.param("min_depth", minDepth, 0.1);
    nh.param("queue_size", queueSize, 2);
    nh.param("bilateral_filter", bilateral_filter, true);
    nh.param("edge_aware_filter", edge_aware_filter, true);
    nh.param("publish_tf", publishTF, false);
    nh.param("base_name_tf", baseNameTF, ns);
    nh.param("worker_threads", worker_threads, 4);

    if(tmp > 0)
    {
      sensor = std::to_string((uint64_t)tmp);
    }

    worker_threads = std::max(1, worker_threads);
    threads.resize(worker_threads);

    std::cout << "parameter:" << std::endl
              << "        base_name: " << ns << std::endl
              << "           sensor: " << sensor << std::endl
              << "        fps_limit: " << fps_limit << std::endl
              << "       calib_path: " << calib_path << std::endl
              << "          use_png: " << (use_png ? "true" : "false") << std::endl
              << "     jpeg_quality: " << jpeg_quality << std::endl
              << "        png_level: " << png_level << std::endl
              << "     depth_method: " << depth_method << std::endl
              << "     depth_device: " << depth_dev << std::endl
              << "       reg_method: " << reg_method << std::endl
              << "       reg_devive: " << reg_dev << std::endl
              << "        max_depth: " << maxDepth << std::endl
              << "        min_depth: " << minDepth << std::endl
              << "       queue_size: " << queueSize << std::endl
              << " bilateral_filter: " << (bilateral_filter ? "true" : "false") << std::endl
              << "edge_aware_filter: " << (edge_aware_filter ? "true" : "false") << std::endl
              << "       publish_tf: " << (publishTF ? "true" : "false") << std::endl
              << "     base_name_tf: " << baseNameTF << std::endl
              << "   worker_threads: " << worker_threads << std::endl << std::endl;

    deltaT = fps_limit > 0 ? 1.0 / fps_limit : 0.0;

    if(calib_path.empty() || calib_path.back() != '/')
    {
      calib_path += '/';
    }

    initCompression(jpeg_quality, png_level, use_png);
    initTopics(queueSize);

    bool ret = true;
    ret = ret && initPipeline(depth_method, depth_dev, bilateral_filter, edge_aware_filter, minDepth, maxDepth);
    ret = ret && initDevice(sensor);

    if(ret)
    {
      initCalibration(calib_path, sensor);
    }

    ret = ret && initRegistration(reg_method, reg_dev, maxDepth);

    if(ret)
    {
      createCameraInfo();
    }

    return ret;
  }

  bool initRegistration(const std::string &method, const int32_t device, const double maxDepth)
  {
    DepthRegistration::Method reg;

    if(method == "default")
    {
      reg = DepthRegistration::DEFAULT;
    }
    else if(method == "cpu")
    {
#ifdef DEPTH_REG_CPU
      reg = DepthRegistration::CPU;
#else
      std::cerr << "CPU registration is not available!" << std::endl;
      return -1;
#endif
    }
    else if(method == "opencl")
    {
#ifdef DEPTH_REG_OPENCL
      reg = DepthRegistration::OPENCL;
#else
      std::cerr << "OpenCL registration is not available!" << std::endl;
      return -1;
#endif
    }
    else
    {
      std::cerr << "Unknown registration method: " << method << std::endl;
      return false;
    }

    depthRegLowRes = DepthRegistration::New(reg);
    depthRegHighRes = DepthRegistration::New(reg);

    bool ret = true;
    ret = ret && depthRegLowRes->init(cameraMatrixLowRes, sizeLowRes, cameraMatrixIr, sizeIr, distortionIr, rotation, translation, 0.5f, maxDepth, device);
    ret = ret && depthRegHighRes->init(cameraMatrixColor, sizeColor, cameraMatrixIr, sizeIr, distortionIr, rotation, translation, 0.5f, maxDepth, device);

    return ret;
  }

  bool initPipeline(const std::string &method, const int32_t device, const bool bilateral_filter, const bool edge_aware_filter, const double minDepth, const double maxDepth)
  {
    if(method == "default")
    {
#ifdef LIBFREENECT2_WITH_OPENCL_SUPPORT
      packetPipeline = new libfreenect2::OpenCLPacketPipeline(device);
#elif defined(LIBFREENECT2_WITH_OPENGL_SUPPORT)
      packetPipeline = new libfreenect2::OpenGLPacketPipeline();
#else
      packetPipeline = new libfreenect2::CpuPacketPipeline();
#endif
    }
    else if(method == "cpu")
    {
      packetPipeline = new libfreenect2::CpuPacketPipeline();
    }
    else if(method == "opencl")
    {
#ifdef LIBFREENECT2_WITH_OPENCL_SUPPORT
      packetPipeline = new libfreenect2::OpenCLPacketPipeline(device);
#else
      std::cerr << "OpenCL depth processing is not available!" << std::endl;
      return false;
#endif
    }
    else if(method == "opengl")
    {
#ifdef LIBFREENECT2_WITH_OPENGL_SUPPORT
      packetPipeline = new libfreenect2::OpenGLPacketPipeline();
#else
      std::cerr << "OpenGL depth processing is not available!" << std::endl;
      return false;
#endif
    }
    else
    {
      std::cerr << "Unknown depth processing method: " << method << std::endl;
      return false;
    }

    libfreenect2::DepthPacketProcessor::Config config;
    config.EnableBilateralFilter = bilateral_filter;
    config.EnableEdgeAwareFilter = edge_aware_filter;
    config.MinDepth = minDepth;
    config.MaxDepth = maxDepth;
    packetPipeline->getDepthPacketProcessor()->setConfiguration(config);
    return true;
  }

  void initCompression(const int32_t jpegQuality, const int32_t pngLevel, const bool use_png)
  {
    compressionParams.resize(7, 0);
    compressionParams[0] = CV_IMWRITE_JPEG_QUALITY;
    compressionParams[1] = jpegQuality;
    compressionParams[2] = CV_IMWRITE_PNG_COMPRESSION;
    compressionParams[3] = pngLevel;
    compressionParams[4] = CV_IMWRITE_PNG_STRATEGY;
    compressionParams[5] = CV_IMWRITE_PNG_STRATEGY_RLE;
    compressionParams[6] = 0;

    if(use_png)
    {
      compression16BitExt = ".png";
      compression16BitString = "; png compressed ";
    }
    else
    {
      compression16BitExt = ".tiff";
      compression16BitString = "; tiff compressed ";
    }
  }

  void initTopics(const int32_t queueSize)
  {
    std::vector<std::string> topics(COUNT);
    topics[IR] = K2_TOPIC_IMAGE_IR;
    topics[IR_RECT] = K2_TOPIC_RECT_IR;
    topics[DEPTH] = K2_TOPIC_IMAGE_DEPTH;
    topics[DEPTH_RECT] = K2_TOPIC_RECT_DEPTH;
    topics[DEPTH_LORES] = K2_TOPIC_LORES_DEPTH;
    topics[DEPTH_HIRES] = K2_TOPIC_HIRES_DEPTH;
    topics[COLOR] = K2_TOPIC_IMAGE_COLOR;
    topics[COLOR_RECT] = K2_TOPIC_RECT_COLOR;
    topics[COLOR_LORES] = K2_TOPIC_LORES_COLOR;
    topics[MONO] = K2_TOPIC_IMAGE_MONO;
    topics[MONO_RECT] = K2_TOPIC_RECT_MONO;
    topics[MONO_LORES] = K2_TOPIC_LORES_MONO;

    imagePubs.resize(COUNT);
    compressedPubs.resize(COUNT);
    infoPubs.resize(COUNT);
    infos.resize(COUNT);

    const std::string base = "/" + ns;
    for(size_t i = 0; i < COUNT; ++i)
    {
      if(i < DEPTH || i > DEPTH_HIRES)
      {
        imagePubs[i] = nh.advertise<sensor_msgs::Image>(base + topics[i] + K2_TOPIC_IMAGE, queueSize);
        compressedPubs[i] = nh.advertise<sensor_msgs::CompressedImage>(base + topics[i] + K2_TOPIC_RAW + K2_TOPIC_COMPRESSED, queueSize);
      }
      else
      {
        imagePubs[i] = nh.advertise<sensor_msgs::Image>(base + topics[i] + K2_TOPIC_RAW, queueSize);
        compressedPubs[i] = nh.advertise<sensor_msgs::CompressedImage>(base + topics[i] + K2_TOPIC_RAW + K2_TOPIC_COMP_DEPTH, queueSize);
      }
      infoPubs[i] = nh.advertise<sensor_msgs::CameraInfo>(base + topics[i] + K2_TOPIC_INFO, queueSize);
    }
  }

  bool initDevice(std::string &sensor)
  {
    bool deviceFound = false;
    const int numOfDevs = freenect2.enumerateDevices();

    if(numOfDevs <= 0)
    {
      std::cerr << "Error: no Kinect2 devices found!" << std::endl;
      return false;
    }

    if(sensor.empty())
    {
      sensor = freenect2.getDefaultDeviceSerialNumber();
    }

    std::cout << "Kinect2 devices found: " << std::endl;
    for(int i = 0; i < numOfDevs; ++i)
    {
      const std::string &s = freenect2.getDeviceSerialNumber(i);
      deviceFound = deviceFound || s == sensor;
      std::cout << "  " << i << ": " << s << (s == sensor ? " (selected)" : "") << std::endl;
    }

    if(!deviceFound)
    {
      std::cerr << "Error: Device with serial '" << sensor << "' not found!" << std::endl;
      return false;
    }

    device = freenect2.openDevice(sensor, packetPipeline);

    if(device == 0)
    {
      std::cout << "no device connected or failure opening the default one!" << std::endl;
      return -1;
    }

    listenerColor = new libfreenect2::SyncMultiFrameListener(libfreenect2::Frame::Color);
    listenerIrDepth = new libfreenect2::SyncMultiFrameListener(libfreenect2::Frame::Ir | libfreenect2::Frame::Depth);

    device->setColorFrameListener(listenerColor);
    device->setIrAndDepthFrameListener(listenerIrDepth);

    std::cout << std::endl << "starting kinect2" << std::endl << std::endl;
    device->start();

    std::cout << std::endl << "device serial: " << sensor << std::endl;
    std::cout << "device firmware: " << device->getFirmwareVersion() << std::endl;

    libfreenect2::Freenect2Device::ColorCameraParams colorParams = device->getColorCameraParams();
    libfreenect2::Freenect2Device::IrCameraParams irParams = device->getIrCameraParams();

    std::cout << std::endl << "default ir camera parameters: " << std::endl;
    std::cout << "fx " << irParams.fx << ", fy " << irParams.fy << ", cx " << irParams.cx << ", cy " << irParams.cy << std::endl;
    std::cout << "k1 " << irParams.k1 << ", k2 " << irParams.k2 << ", p1 " << irParams.p1 << ", p2 " << irParams.p2 << ", k3 " << irParams.k3 << std::endl;

    std::cout << std::endl << "default color camera parameters: " << std::endl;
    std::cout << "fx " << colorParams.fx << ", fy " << colorParams.fy << ", cx " << colorParams.cx << ", cy " << colorParams.cy << std::endl;

    cameraMatrixColor = cv::Mat::eye(3, 3, CV_64F);
    distortionColor = cv::Mat::zeros(1, 5, CV_64F);

    cameraMatrixColor.at<double>(0, 0) = colorParams.fx;
    cameraMatrixColor.at<double>(1, 1) = colorParams.fy;
    cameraMatrixColor.at<double>(0, 2) = colorParams.cx;
    cameraMatrixColor.at<double>(1, 2) = colorParams.cy;
    cameraMatrixColor.at<double>(2, 2) = 1;

    cameraMatrixIr = cv::Mat::eye(3, 3, CV_64F);
    distortionIr = cv::Mat::zeros(1, 5, CV_64F);

    cameraMatrixIr.at<double>(0, 0) = irParams.fx;
    cameraMatrixIr.at<double>(1, 1) = irParams.fy;
    cameraMatrixIr.at<double>(0, 2) = irParams.cx;
    cameraMatrixIr.at<double>(1, 2) = irParams.cy;
    cameraMatrixIr.at<double>(2, 2) = 1;

    distortionIr.at<double>(0, 0) = irParams.k1;
    distortionIr.at<double>(0, 1) = irParams.k2;
    distortionIr.at<double>(0, 2) = irParams.p1;
    distortionIr.at<double>(0, 3) = irParams.p2;
    distortionIr.at<double>(0, 4) = irParams.k3;

    rotation = cv::Mat::eye(3, 3, CV_64F);
    translation = cv::Mat::zeros(3, 1, CV_64F);
    translation.at<double>(0) = -0.0520;
    return true;
  }

  void initCalibration(const std::string &calib_path, const std::string &sensor)
  {
    std::string calibPath = calib_path + sensor + '/';

    struct stat fileStat;
    bool calibDirNotFound = stat(calibPath.c_str(), &fileStat) != 0 || !S_ISDIR(fileStat.st_mode);
    if(calibDirNotFound || !loadCalibrationFile(calibPath + K2_CALIB_COLOR, cameraMatrixColor, distortionColor))
    {
      std::cerr << "using sensor defaults for color intrinsic parameters." << std::endl;
    }

    if(calibDirNotFound || !loadCalibrationFile(calibPath + K2_CALIB_IR, cameraMatrixIr, distortionIr))
    {
      std::cerr << "using sensor defaults for ir intrinsic parameters." << std::endl;
    }

    if(calibDirNotFound || !loadCalibrationPoseFile(calibPath + K2_CALIB_POSE, rotation, translation))
    {
      std::cerr << "using defaults for rotation and translation." << std::endl;
    }

    if(calibDirNotFound || !loadCalibrationDepthFile(calibPath + K2_CALIB_DEPTH, depthShift))
    {
      std::cerr << "using defaults for depth shift." << std::endl;
      depthShift = 0.0;
    }

    cameraMatrixLowRes = cameraMatrixColor.clone();
    cameraMatrixLowRes.at<double>(0, 0) /= 2;
    cameraMatrixLowRes.at<double>(1, 1) /= 2;
    cameraMatrixLowRes.at<double>(0, 2) /= 2;
    cameraMatrixLowRes.at<double>(1, 2) /= 2;

    const int mapType = CV_16SC2;
    cv::initUndistortRectifyMap(cameraMatrixColor, distortionColor, cv::Mat(), cameraMatrixColor, sizeColor, mapType, map1Color, map2Color);
    cv::initUndistortRectifyMap(cameraMatrixIr, distortionIr, cv::Mat(), cameraMatrixIr, sizeIr, mapType, map1Ir, map2Ir);
    cv::initUndistortRectifyMap(cameraMatrixColor, distortionColor, cv::Mat(), cameraMatrixLowRes, sizeLowRes, mapType, map1LowRes, map2LowRes);

    std::cout << std::endl << "camera parameters used:" << std::endl
              << "camera matrix color:" << std::endl << cameraMatrixColor << std::endl
              << "distortion coefficients color:" << std::endl << distortionColor << std::endl
              << "camera matrix ir:" << std::endl << cameraMatrixIr << std::endl
              << "distortion coefficients ir:" << std::endl << distortionIr << std::endl
              << "rotation:" << std::endl << rotation << std::endl
              << "translation:" << std::endl << translation << std::endl
              << "depth shift:" << std::endl << depthShift << std::endl << std::endl;
  }

  bool loadCalibrationFile(const std::string &filename, cv::Mat &cameraMatrix, cv::Mat &distortion) const
  {
    cv::FileStorage fs;
    if(fs.open(filename, cv::FileStorage::READ))
    {
      fs[K2_CALIB_CAMERA_MATRIX] >> cameraMatrix;
      fs[K2_CALIB_DISTORTION] >> distortion;
      fs.release();
    }
    else
    {
      std::cerr << "can't open calibration file: " << filename << std::endl;
      return false;
    }
    return true;
  }

  bool loadCalibrationPoseFile(const std::string &filename, cv::Mat &rotation, cv::Mat &translation) const
  {
    cv::FileStorage fs;
    if(fs.open(filename, cv::FileStorage::READ))
    {
      fs[K2_CALIB_ROTATION] >> rotation;
      fs[K2_CALIB_TRANSLATION] >> translation;
      fs.release();
    }
    else
    {
      std::cerr << "can't open calibration pose file: " << filename << std::endl;
      return false;
    }
    return true;
  }

  bool loadCalibrationDepthFile(const std::string &filename, double &depthShift) const
  {
    cv::FileStorage fs;
    if(fs.open(filename, cv::FileStorage::READ))
    {
      fs[K2_CALIB_DEPTH_SHIFT] >> depthShift;
      fs.release();
    }
    else
    {
      std::cerr << "can't open calibration depth file: " << filename << std::endl;
      return false;
    }
    return true;
  }

  void createCameraInfo()
  {
    cv::Mat projColor = cv::Mat::zeros(3, 4, CV_64F);
    cv::Mat projIr = cv::Mat::zeros(3, 4, CV_64F);
    cv::Mat projLowRes = cv::Mat::zeros(3, 4, CV_64F);

    cameraMatrixColor.copyTo(projColor(cv::Rect(0, 0, 3, 3)));
    cameraMatrixIr.copyTo(projIr(cv::Rect(0, 0, 3, 3)));
    cameraMatrixLowRes.copyTo(projLowRes(cv::Rect(0, 0, 3, 3)));

    createCameraInfo(sizeColor, cameraMatrixColor, distortionColor, cv::Mat::eye(3, 3, CV_64F), projColor, infos[COLOR]);
    createCameraInfo(sizeColor, cameraMatrixColor, distortionColor, cv::Mat::eye(3, 3, CV_64F), projColor, infos[MONO]);
    createCameraInfo(sizeIr, cameraMatrixIr, distortionIr, cv::Mat::eye(3, 3, CV_64F), projIr, infos[IR]);
    infos[DEPTH] = infos[IR];

    createCameraInfo(sizeColor, cameraMatrixColor, cv::Mat::zeros(1, 5, CV_64F), cv::Mat::eye(3, 3, CV_64F), projColor, infos[COLOR_RECT]);
    createCameraInfo(sizeColor, cameraMatrixColor, cv::Mat::zeros(1, 5, CV_64F), cv::Mat::eye(3, 3, CV_64F), projColor, infos[MONO_RECT]);
    createCameraInfo(sizeIr, cameraMatrixIr, cv::Mat::zeros(1, 5, CV_64F), cv::Mat::eye(3, 3, CV_64F), projIr, infos[IR_RECT]);

    createCameraInfo(sizeIr, cameraMatrixIr, cv::Mat::zeros(1, 5, CV_64F), cv::Mat::eye(3, 3, CV_64F), projIr, infos[DEPTH_RECT]);
    createCameraInfo(sizeLowRes, cameraMatrixLowRes, cv::Mat::zeros(1, 5, CV_64F), cv::Mat::eye(3, 3, CV_64F), projLowRes, infos[COLOR_LORES]);
    createCameraInfo(sizeLowRes, cameraMatrixLowRes, cv::Mat::zeros(1, 5, CV_64F), cv::Mat::eye(3, 3, CV_64F), projLowRes, infos[MONO_LORES]);
    createCameraInfo(sizeLowRes, cameraMatrixLowRes, cv::Mat::zeros(1, 5, CV_64F), cv::Mat::eye(3, 3, CV_64F), projLowRes, infos[DEPTH_LORES]);

    createCameraInfo(sizeColor, cameraMatrixColor, cv::Mat::zeros(1, 5, CV_64F), cv::Mat::eye(3, 3, CV_64F), projColor, infos[DEPTH_HIRES]);
  }

  void createCameraInfo(const cv::Size &size, const cv::Mat &cameraMatrix, const cv::Mat &distortion, const cv::Mat &rotation, const cv::Mat &projection, sensor_msgs::CameraInfo &cameraInfo) const
  {
    cameraInfo.height = size.height;
    cameraInfo.width = size.width;

    const double *itC = cameraMatrix.ptr<double>(0, 0);
    for(size_t i = 0; i < 9; ++i, ++itC)
    {
      cameraInfo.K[i] = *itC;
    }

    const double *itR = rotation.ptr<double>(0, 0);
    for(size_t i = 0; i < 9; ++i, ++itR)
    {
      cameraInfo.R[i] = *itR;
    }

    const double *itP = projection.ptr<double>(0, 0);
    for(size_t i = 0; i < 12; ++i, ++itP)
    {
      cameraInfo.P[i] = *itP;
    }

    cameraInfo.distortion_model = "plumb_bob";
    cameraInfo.D.resize(distortion.cols);
    const double *itD = distortion.ptr<double>(0, 0);
    for(size_t i = 0; i < (size_t)distortion.cols; ++i, ++itD)
    {
      cameraInfo.D[i] = *itD;
    }
  }

  void threadDispatcher(const size_t id)
  {
    const size_t checkFirst = id % 2;
    bool processedFrame = false;
    int oldNice = nice(0);
    oldNice = nice(19 - oldNice);

    for(; running && ros::ok();)
    {
      processedFrame = false;

      for(size_t i = 0; i < 2; ++i)
      {
        if(i == checkFirst)
        {
          if(nextIrDepth && lockIrDepth.try_lock())
          {
            nextIrDepth = false;
            receiveIrDepth();
            processedFrame = true;
          }
        }
        else
        {
          if(nextColor && lockColor.try_lock())
          {
            nextColor = false;
            receiveColor();
            processedFrame = true;
          }
        }
      }

      if(!processedFrame)
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    }
  }

  void receiveIrDepth()
  {
    libfreenect2::FrameMap frames;
    libfreenect2::Frame *irFrame, *depthFrame;
    cv::Mat depth, ir;
    std_msgs::Header header;
    std::vector<cv::Mat> images(COUNT);
    std::vector<Status> status(COUNT, UNSUBCRIBED);
    size_t frame;

    if(!receiveFrames(listenerIrDepth, frames))
    {
      return;
    }
    double now = ros::Time::now().toSec();

    header = createHeader(lastDepth, lastColor);

    irFrame = frames[libfreenect2::Frame::Ir];
    depthFrame = frames[libfreenect2::Frame::Depth];

    ir = cv::Mat(irFrame->height, irFrame->width, CV_32FC1, irFrame->data);
    depth = cv::Mat(depthFrame->height, depthFrame->width, CV_32FC1, depthFrame->data);

    frame = frameIrDepth++;
    lockIrDepth.unlock();

    updateStatus(status);
    processIrDepth(ir, depth, images, status);
    listenerIrDepth->release(frames);

    publishImages(images, header, status, frame, pubFrameIrDepth, IR, COLOR);

    double elapsed = ros::Time::now().toSec() - now;
    lockTime.lock();
    elapsedTimeIrDepth += elapsed;
    lockTime.unlock();
  }

  void receiveColor()
  {
    libfreenect2::FrameMap frames;
    libfreenect2::Frame *colorFrame;
    cv::Mat color;
    std_msgs::Header header;
    std::vector<cv::Mat> images(COUNT);
    std::vector<Status> status(COUNT, UNSUBCRIBED);
    size_t frame;

    if(!receiveFrames(listenerColor, frames))
    {
      return;
    }
    double now = ros::Time::now().toSec();

    header = createHeader(lastColor, lastDepth);

    colorFrame = frames[libfreenect2::Frame::Color];
    color = cv::Mat(colorFrame->height, colorFrame->width, CV_8UC3, colorFrame->data);

    frame = frameColor++;
    lockColor.unlock();

    updateStatus(status);
    processColor(color, images, status);
    listenerColor->release(frames);

    publishImages(images, header, status, frame, pubFrameColor, COLOR, COUNT);

    double elapsed = ros::Time::now().toSec() - now;
    lockTime.lock();
    elapsedTimeColor += elapsed;
    lockTime.unlock();
  }

  bool receiveFrames(libfreenect2::SyncMultiFrameListener *listener, libfreenect2::FrameMap &frames)
  {
    bool newFrames = false;
    for(; !newFrames;)
    {
#ifdef LIBFREENECT2_THREADING_STDLIB
      newFrames = listener->waitForNewFrame(frames, 1000);
#else
      newFrames = true;
      listener->waitForNewFrame(frames);
#endif
      if(!running || !ros::ok())
      {
        if(newFrames)
        {
          listener->release(frames);
        }
        return false;
      }
    }
    return true;
  }

  std_msgs::Header createHeader(ros::Time &last, ros::Time &other)
  {
    ros::Time timestamp = ros::Time::now();
    lockSync.lock();
    if(other.isZero())
    {
      last = timestamp;
    }
    else
    {
      timestamp = other;
      other = ros::Time(0, 0);
    }
    lockSync.unlock();

    std_msgs::Header header;
    header.seq = 0;
    header.stamp = timestamp;
    header.frame_id = K2_TF_RGB_OPT_FRAME;
    return header;
  }

  bool updateStatus(std::vector<Status> &status)
  {
    bool any = false;
    for(size_t i = 0; i < COUNT; ++i)
    {
      Status s = UNSUBCRIBED;
      if(imagePubs[i].getNumSubscribers() > 0)
      {
        s = RAW;
      }
      if(compressedPubs[i].getNumSubscribers() > 0)
      {
        s = s == RAW ? BOTH : COMPRESSED;
      }

      status[i] = s;
      any = any || s != UNSUBCRIBED || infoPubs[i].getNumSubscribers() > 0;
    }
    return any;
  }

  void processIrDepth(const cv::Mat &ir, const cv::Mat &depth, std::vector<cv::Mat> &images, const std::vector<Status> &status)
  {

    // IR
    if(status[IR] || status[IR_RECT])
    {
      ir.convertTo(images[IR], CV_16U);
      cv::flip(images[IR], images[IR], 1);
    }
    if(status[IR_RECT])
    {
      cv::remap(images[IR], images[IR_RECT], map1Ir, map2Ir, cv::INTER_AREA);
    }

    // DEPTH
    cv::Mat depthShifted;
    if(status[DEPTH])
    {
      depth.convertTo(images[DEPTH], CV_16U, 1);
      cv::flip(images[DEPTH], images[DEPTH], 1);
    }
    if(status[DEPTH_RECT] || status[DEPTH_LORES] || status[DEPTH_HIRES])
    {
      depth.convertTo(depthShifted, CV_16U, 1, depthShift);
      cv::flip(depthShifted, depthShifted, 1);
    }
    if(status[DEPTH_RECT])
    {
      cv::remap(depthShifted, images[DEPTH_RECT], map1Ir, map2Ir, cv::INTER_NEAREST);
    }
    if(status[DEPTH_LORES])
    {
      lockRegLowRes.lock();
      depthRegLowRes->registerDepth(depthShifted, images[DEPTH_LORES]);
      lockRegLowRes.unlock();
    }
    if(status[DEPTH_HIRES])
    {
      lockRegHighRes.lock();
      depthRegHighRes->registerDepth(depthShifted, images[DEPTH_HIRES]);
      lockRegHighRes.unlock();
    }
  }

  void processColor(const cv::Mat &color, std::vector<cv::Mat> &images, const std::vector<Status> &status)
  {
    // COLOR
    if(status[COLOR] || status[COLOR_RECT] || status[COLOR_LORES] || status[MONO] || status[MONO_RECT] || status[MONO_LORES])
    {
      cv::flip(color, images[COLOR], 1);
    }
    if(status[COLOR_RECT] || status[MONO_RECT])
    {
      cv::remap(images[COLOR], images[COLOR_RECT], map1Color, map2Color, cv::INTER_AREA);
    }
    if(status[COLOR_LORES] || status[MONO_LORES])
    {
      cv::remap(images[COLOR], images[COLOR_LORES], map1LowRes, map2LowRes, cv::INTER_AREA);
    }

    // MONO
    if(status[MONO])
    {
      cv::cvtColor(images[COLOR], images[MONO], CV_BGR2GRAY);
    }
    if(status[MONO_RECT])
    {
      cv::cvtColor(images[COLOR_RECT], images[MONO_RECT], CV_BGR2GRAY);
    }
    if(status[MONO_LORES])
    {
      cv::cvtColor(images[COLOR_LORES], images[MONO_LORES], CV_BGR2GRAY);
    }
  }

  void publishImages(const std::vector<cv::Mat> &images, const std_msgs::Header &header, const std::vector<Status> &status, const size_t frame, size_t &pubFrame, const size_t begin, const size_t end)
  {
    std::vector<sensor_msgs::Image> imageMsgs(COUNT);
    std::vector<sensor_msgs::CompressedImage> compressedMsgs(COUNT);
    std::vector<sensor_msgs::CameraInfo> infoMsgs(COUNT);

    for(size_t i = begin; i < end; ++i)
    {
      infoMsgs[i] = infos[i];
      infoMsgs[i].header = header;
      infoMsgs[i].header.frame_id = baseNameTF + (i < DEPTH_LORES ? K2_TF_IR_OPT_FRAME : K2_TF_RGB_OPT_FRAME);

      switch(status[i])
      {
      case UNSUBCRIBED:
        break;
      case RAW:
        createImage(images[i], infoMsgs[i].header, Image(i), imageMsgs[i]);
        break;
      case COMPRESSED:
        createCompressed(images[i], infoMsgs[i].header, Image(i), compressedMsgs[i]);
        break;
      case BOTH:
        createImage(images[i], infoMsgs[i].header, Image(i), imageMsgs[i]);
        createCompressed(images[i], infoMsgs[i].header, Image(i), compressedMsgs[i]);
        break;
      }
    }

    while(frame != pubFrame)
    {
      std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
    lockPub.lock();
    for(size_t i = begin; i < end; ++i)
    {
      switch(status[i])
      {
      case UNSUBCRIBED:
        if(infoPubs[i].getNumSubscribers() > 0)
        {
          infoPubs[i].publish(infoMsgs[i]);
        }
        break;
      case RAW:
        imagePubs[i].publish(imageMsgs[i]);
        infoPubs[i].publish(infoMsgs[i]);
        break;
      case COMPRESSED:
        compressedPubs[i].publish(compressedMsgs[i]);
        infoPubs[i].publish(infoMsgs[i]);
        break;
      case BOTH:
        imagePubs[i].publish(imageMsgs[i]);
        compressedPubs[i].publish(compressedMsgs[i]);
        infoPubs[i].publish(infoMsgs[i]);
        break;
      }
    }
    ++pubFrame;
    lockPub.unlock();
  }

  void createImage(const cv::Mat &image, const std_msgs::Header &header, const Image type, sensor_msgs::Image &msgImage) const
  {
    size_t step, size;
    step = image.cols * image.elemSize();
    size = image.rows * step;

    switch(type)
    {
    case IR:
    case IR_RECT:
      msgImage.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
      break;
    case DEPTH:
    case DEPTH_RECT:
    case DEPTH_LORES:
    case DEPTH_HIRES:
      msgImage.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
      break;
    case COLOR:
    case COLOR_RECT:
    case COLOR_LORES:
      msgImage.encoding = sensor_msgs::image_encodings::BGR8;
      break;
    case MONO:
    case MONO_RECT:
    case MONO_LORES:
      msgImage.encoding = sensor_msgs::image_encodings::MONO8;
      break;
    case COUNT:
      return;
    }

    msgImage.header = header;
    msgImage.height = image.rows;
    msgImage.width = image.cols;
    msgImage.is_bigendian = false;
    msgImage.step = step;
    msgImage.data.resize(size);
    memcpy(msgImage.data.data(), image.data, size);
  }

  void createCompressed(const cv::Mat &image, const std_msgs::Header &header, const Image type, sensor_msgs::CompressedImage &msgImage) const
  {
    msgImage.header = header;

    switch(type)
    {
    case IR:
    case IR_RECT:
      msgImage.format = sensor_msgs::image_encodings::TYPE_16UC1 + compression16BitString;
      cv::imencode(compression16BitExt, image, msgImage.data, compressionParams);
      break;
    case DEPTH:
    case DEPTH_RECT:
    case DEPTH_LORES:
    case DEPTH_HIRES:
      {
        compressed_depth_image_transport::ConfigHeader compressionConfig;
        const size_t headerSize = sizeof(compressed_depth_image_transport::ConfigHeader);
        compressionConfig.format = compressed_depth_image_transport::INV_DEPTH;

        std::vector<uint8_t> data;
        msgImage.format = sensor_msgs::image_encodings::TYPE_16UC1 + "; compressedDepth";
        cv::imencode(compression16BitExt, image, data, compressionParams);

        msgImage.data.resize(headerSize + data.size());
        memcpy(&msgImage.data[0], &compressionConfig, headerSize);
        memcpy(&msgImage.data[headerSize], &data[0], data.size());
        break;
      }
    case COLOR:
    case COLOR_RECT:
    case COLOR_LORES:
      msgImage.format = sensor_msgs::image_encodings::BGR8 + "; jpeg compressed bgr8";
      cv::imencode(".jpg", image, msgImage.data, compressionParams);
      break;
    case MONO:
    case MONO_RECT:
    case MONO_LORES:
      msgImage.format = sensor_msgs::image_encodings::MONO8 + "; png compressed ";
      cv::imencode(".png", image, msgImage.data, compressionParams);
      break;
    case COUNT:
      return;
    }
  }

  void publishStaticTF()
  {
    tf::TransformBroadcaster broadcaster;
    tf::StampedTransform stColor, stColorOpt, stIr, stIrOpt;
    ros::Time now = ros::Time::now();

    tf::Matrix3x3 rot(rotation.at<double>(0, 0), rotation.at<double>(0, 1), rotation.at<double>(0, 2),
                      rotation.at<double>(1, 0), rotation.at<double>(1, 1), rotation.at<double>(1, 2),
                      rotation.at<double>(2, 0), rotation.at<double>(2, 1), rotation.at<double>(2, 2));
    rot = rot.inverse();

    tf::Quaternion qZero, qOpt;
    qZero.setRPY(0, 0, 0);
    qOpt.setRPY(-M_PI_2, 0, -M_PI_2);
    tf::Vector3 trans(translation.at<double>(2), -translation.at<double>(0), -translation.at<double>(1));
    tf::Vector3 vZero(0, 0, 0);
    tf::Transform tIr(rot, trans), tOpt(qOpt, vZero), tZero(qZero, vZero);

    stColor = tf::StampedTransform(tZero, now, baseNameTF + K2_TF_LINK, baseNameTF + K2_TF_RGB_FRAME);
    stIr = tf::StampedTransform(tIr, now, baseNameTF + K2_TF_LINK, baseNameTF + K2_TF_IR_FRAME);
    stColorOpt = tf::StampedTransform(tOpt, now, baseNameTF + K2_TF_RGB_FRAME, baseNameTF + K2_TF_RGB_OPT_FRAME);
    stIrOpt = tf::StampedTransform(tOpt, now, baseNameTF + K2_TF_IR_FRAME, baseNameTF + K2_TF_IR_OPT_FRAME);

    for(; running && ros::ok();)
    {
      now = ros::Time::now();
      stColor.stamp_ = now;
      stColorOpt.stamp_ = now;
      stIr.stamp_ = now;
      stIrOpt.stamp_ = now;

      broadcaster.sendTransform(stColor);
      broadcaster.sendTransform(stColorOpt);
      broadcaster.sendTransform(stIr);
      broadcaster.sendTransform(stIrOpt);

      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
};

class Kinect2BridgeNodelet : public nodelet::Nodelet
{
private:
  std::thread kinect2BridgeThread;
  Kinect2Bridge *pKinect2Bridge;

public:
  Kinect2BridgeNodelet() : Nodelet(), pKinect2Bridge(NULL)
  {
  }

  ~Kinect2BridgeNodelet()
  {
    if(pKinect2Bridge)
    {
      pKinect2Bridge->stop();
      kinect2BridgeThread.join();
      delete pKinect2Bridge;
    }
  }

  virtual void onInit()
  {
    kinect2BridgeThread = std::thread(&Kinect2BridgeNodelet::runKinect2Brigde, this);
  }

private:
  void runKinect2Brigde()
  {
    pKinect2Bridge = new Kinect2Bridge(getPrivateNodeHandle());
    pKinect2Bridge->run();
  }
};

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(Kinect2BridgeNodelet, nodelet::Nodelet)

void helpOption(const std::string &name, const std::string &stype, const std::string &value, const std::string &desc)
{
  std::cout  << '_' << name << ":=<" << stype << '>' << std::endl
             << "    default: " << value << std::endl
             << "    info:    " << desc << std::endl;
}

void help(const std::string &path)
{
  std::string depthMethods = "cpu";
  std::string depthDefault = "cpu";
  std::string regMethods = "default";
  std::string regDefault = "default";

#ifdef LIBFREENECT2_WITH_OPENGL_SUPPORT
  depthMethods += ", opengl";
  depthDefault = "opengl";
#endif
#ifdef LIBFREENECT2_WITH_OPENCL_SUPPORT
  depthMethods += ", opencl";
  depthDefault = "opencl";
#endif
#ifdef DEPTH_REG_CPU
  regMethods += ", cpu";
#endif
#ifdef DEPTH_REG_OPENCL
  regMethods += ", opencl";
  regDefault = "opencl";
#endif

  std::cout << path << " [_options:=value]" << std::endl;
  helpOption("base_name",         "string", K2_DEFAULT_NS,  "set base name for all topics");
  helpOption("sensor",            "double", "-1.0",         "serial of the sensor to use");
  helpOption("fps_limit",         "double", "-1.0",         "limit the frames per second");
  helpOption("calib_path",        "string", K2_CALIB_PATH,  "path to the calibration files");
  helpOption("use_png",           "bool",   "false",        "Use PNG compression instead of TIFF");
  helpOption("jpeg_quality",      "int",    "90",           "JPEG quality level from 0 to 100");
  helpOption("png_level",         "int",    "1",            "PNG compression level from 0 to 9");
  helpOption("depth_method",      "string", depthDefault,   "Use specific depth processing: " + depthMethods);
  helpOption("depth_device",      "int",    "-1",           "openCL device to use for depth processing");
  helpOption("reg_method",        "string", regDefault,     "Use specific depth registration: " + regMethods);
  helpOption("reg_devive",        "int",    "-1",           "openCL device to use for depth registration");
  helpOption("max_depth",         "double", "12.0",         "max depth value");
  helpOption("min_depth",         "double", "0.1",          "min depth value");
  helpOption("queue_size",        "int",    "2",            "queue size of publisher");
  helpOption("bilateral_filter",  "bool",   "true",         "enable bilateral filtering of depth images");
  helpOption("edge_aware_filter", "bool",   "true",         "enable edge aware filtering of depth images");
  helpOption("publish_tf",        "bool",   "false",        "publish static tf transforms for camera");
  helpOption("base_name_tf",      "string", "as base_name", "base name for the tf frames");
  helpOption("worker_threads",    "int",    "4",            "number of threads used for processing the images");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "kinect2_bridge", ros::init_options::AnonymousName);

  for(int argI = 1; argI < argc; ++ argI)
  {
    std::string arg(argv[argI]);

    if(arg == "--help" || arg == "--h" || arg == "-h" || arg == "-?" || arg == "--?")
    {
      help(argv[0]);
      ros::shutdown();
      return 0;
    }
    else
    {
      std::cerr << "Unknown argument: " << arg << std::endl;
      return -1;
    }
  }

  if(!ros::ok())
  {
    std::cerr << "ros::ok failed!" << std::endl;
    return -1;
  }

  Kinect2Bridge kinect2;

  kinect2.run();

  ros::shutdown();
  return 0;
}
