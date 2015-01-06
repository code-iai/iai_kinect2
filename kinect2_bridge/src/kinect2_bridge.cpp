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
#include <std_msgs/Header.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/image_encodings.h>

#include <compressed_depth_image_transport/compression_common.h>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/packet_pipeline.h>

#include <kinect2_definitions.h>
#include <depth_registration.h>

class Kinect2Bridge
{
private:
  const int32_t jpegQuality;
  const int32_t pngLevel;
  const double maxDepth;
  const size_t queueSize;
  std::vector<int> compressionParams;
  const bool rawDepth;
  std::string compression16BitExt;
  std::string compression16BitString;

  size_t frame;
  const cv::Size sizeColor, sizeIr, sizeDepth;
  cv::Mat color, ir, depth;
  cv::Mat cameraMatrixColor, distortionColor, cameraMatrixDepth;
  cv::Mat cameraMatrixIr, distortionIr;
  cv::Mat rotation, translation;
  cv::Mat map1Color, map2Color;
  cv::Mat map1Ir, map2Ir;
  cv::Mat map1ColorReg, map2ColorReg;

  std::vector<std::thread> threads;
  std::mutex lock;
  std::mutex lockProcess;
  std::mutex lockPub;
  std::mutex lockRegLowRes;
  std::mutex lockRegHighRes;
  size_t pubFrame;
  bool newFrame;

  libfreenect2::Freenect2 freenect2;
  libfreenect2::Freenect2Device *device;
  libfreenect2::SyncMultiFrameListener *listener;
  libfreenect2::FrameMap frames;
  libfreenect2::PacketPipeline *packetPipeline;

  ros::NodeHandle nh;

  DepthRegistration *depthRegLowRes;
  DepthRegistration *depthRegHighRes;

  const double deltaT;

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

  std::vector<std::string> topics;

  std_msgs::Header header;
  std::vector<ros::Publisher> imagePubs;
  std::vector<ros::Publisher> compressedPubs;
  std::vector<ros::Publisher> infoPubs;
  std::vector<sensor_msgs::CameraInfo> infos;
  std::vector<Status> statusPubs;

public:
  Kinect2Bridge(const double fps, const bool rawDepth, const int compression)
    : jpegQuality(compression), pngLevel(1), maxDepth(10.0), queueSize(2), rawDepth(rawDepth), sizeColor(1920, 1080), sizeIr(512, 424),
      sizeDepth(rawDepth ? sizeIr : cv::Size(sizeColor.width / 2, sizeColor.height / 2)), newFrame(false), nh(),
      deltaT(1.0 / fps), topics(COUNT)
  {
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

    color = cv::Mat::zeros(sizeColor, CV_8UC3);
    ir = cv::Mat::zeros(sizeIr, CV_32F);
    depth = cv::Mat::zeros(sizeIr, CV_32F);

    compressionParams.resize(7, 0);
    compressionParams[0] = CV_IMWRITE_JPEG_QUALITY;
    compressionParams[1] = jpegQuality;
    compressionParams[2] = CV_IMWRITE_PNG_COMPRESSION;
    compressionParams[3] = pngLevel;
    compressionParams[4] = CV_IMWRITE_PNG_STRATEGY;
    compressionParams[5] = CV_IMWRITE_PNG_STRATEGY_RLE;
    compressionParams[6] = 0;

#ifdef USE_OPENCL_REGISTRATION
    depthRegLowRes = DepthRegistration::New(DepthRegistration::OPENCL);
    depthRegHighRes = DepthRegistration::New(DepthRegistration::OPENCL);
#else
    depthRegLowRes = DepthRegistration::New(DepthRegistration::CPU);
    depthRegHighRes = DepthRegistration::New(DepthRegistration::CPU);
#endif
#ifdef USE_OPENCL_PIPELINE
    packetPipeline = new libfreenect2::OpenCLPacketPipeline();
#else
    packetPipeline = new libfreenect2::DefaultPacketPipeline();
#endif
#ifdef USE_TIFF_COMRESSION
    compression16BitExt = ".tiff";
    compression16BitString = "; tiff compressed ";
#else
    compression16BitExt = ".png";
    compression16BitString = "; png compressed ";
#endif
  }

  bool init(const std::string &path, const std::string &cam, const int deviceId)
  {
    std::string serial;

    if(cam.empty())
    {
      device = freenect2.openDefaultDevice(packetPipeline);
    }
    else
    {
      device = freenect2.openDevice(cam, packetPipeline);
    }

    if(device == 0)
    {
      std::cout << "no device connected or failure opening the default one!" << std::endl;
      return -1;
    }
    listener = new libfreenect2::SyncMultiFrameListener(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth);

    device->setColorFrameListener(listener);
    device->setIrAndDepthFrameListener(listener);

    std::cout << std::endl << "starting kinect2" << std::endl << std::endl;
    device->start();

    serial = device->getSerialNumber();
    std::cout << std::endl << "device serial: " << serial << std::endl;
    std::cout << "device firmware: " << device->getFirmwareVersion() << std::endl;


    libfreenect2::Freenect2Device::ColorCameraParams colorParams = device->getColorCameraParams();
    libfreenect2::Freenect2Device::IrCameraParams irParams = device->getIrCameraParams();

    std::cout << std::endl << "default ir camera parameters: " << std::endl;
    std::cout << "fx " << irParams.fx << ", fy " << irParams.fy << ", cx " << irParams.cx << ", cy " << irParams.cy << std::endl;
    std::cout << "k1 " << irParams.k1 << ", k2 " << irParams.k2 << ", p1 " << irParams.p1 << ", p2 " << irParams.p2 << ", k3 " << irParams.k3 << std::endl;

    std::cout << std::endl << "default color camera parameters: " << std::endl;
    std::cout << "fx " << colorParams.fx << ", fy " << colorParams.fy << ", cx " << colorParams.cx << ", cy " << colorParams.cy << std::endl;

    /*std::cout << "unknown color camera parameters: " << std::endl;
    for(size_t i = 3; i < 25; ++i)
    {
      std::cout << i + 1 << ": " << colorParams.all[i] << std::endl;
    }*/

    std::string calibPath = path + serial;

    struct stat fileStat;
    if(stat(calibPath.c_str(), &fileStat) != 0 || !S_ISDIR(fileStat.st_mode)
       || !loadCalibrationFile(calibPath + K2_CALIB_COLOR, cameraMatrixColor, distortionColor)
       || !loadCalibrationFile(calibPath + K2_CALIB_IR, cameraMatrixIr, distortionIr)
       || !loadCalibrationPoseFile(calibPath + K2_CALIB_POSE, rotation, translation))
    {
      std::cerr << std::endl << "could not load calibration data from \"" << calibPath << "\". using sensor defaults." << std::endl;

      cameraMatrixColor = cv::Mat::eye(3, 3, CV_64F);
      distortionColor = cv::Mat::zeros(1, 5, CV_64F);
      cameraMatrixIr = cv::Mat::eye(3, 3, CV_64F);
      distortionIr = cv::Mat::zeros(1, 5, CV_64F);

      cameraMatrixColor.at<double>(0, 0) = colorParams.fx;
      cameraMatrixColor.at<double>(1, 1) = colorParams.fy;
      cameraMatrixColor.at<double>(0, 2) = colorParams.cx;
      cameraMatrixColor.at<double>(1, 2) = colorParams.cy;
      cameraMatrixColor.at<double>(2, 2) = 1;

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
    }

    if(rawDepth)
    {
      cameraMatrixDepth = cameraMatrixIr;
    }
    else
    {
      cameraMatrixDepth = cameraMatrixColor.clone();
      cameraMatrixDepth.at<double>(0, 0) /= 2;
      cameraMatrixDepth.at<double>(1, 1) /= 2;
      cameraMatrixDepth.at<double>(0, 2) /= 2;
      cameraMatrixDepth.at<double>(1, 2) /= 2;
    }

    std::cout << std::endl << "camera parameters used:" << std::endl
              << "camera matrix color:" << std::endl << cameraMatrixColor << std::endl
              << "distortion coefficients color:" << std::endl << distortionColor << std::endl
              << "camera matrix ir:" << std::endl << cameraMatrixIr << std::endl
              << "distortion coefficients ir:" << std::endl << distortionIr << std::endl
              << "rotation:" << std::endl << rotation << std::endl
              << "translation:" << std::endl << translation << std::endl << std::endl;

    const int mapType = CV_16SC2;
    cv::initUndistortRectifyMap(cameraMatrixColor, distortionColor, cv::Mat(), cameraMatrixColor, sizeColor, mapType, map1Color, map2Color);
    cv::initUndistortRectifyMap(cameraMatrixIr, distortionIr, cv::Mat(), cameraMatrixIr, sizeIr, mapType, map1Ir, map2Ir);
    cv::initUndistortRectifyMap(cameraMatrixColor, distortionColor, cv::Mat(), cameraMatrixDepth, sizeDepth, mapType, map1ColorReg, map2ColorReg);

    bool ret = true;
    ret = ret && depthRegLowRes->init(cameraMatrixDepth, sizeDepth, cameraMatrixIr, sizeIr, distortionIr, rotation, translation, 0.5f, maxDepth, deviceId);
    ret = ret && depthRegHighRes->init(cameraMatrixColor, sizeColor, cameraMatrixIr, sizeIr, distortionIr, rotation, translation, 0.5f, maxDepth, deviceId);

    return ret;
  }

  void run()
  {
    int oldNice = nice(0);
    int newNice = nice(10 - oldNice);

    imagePubs.resize(COUNT);
    compressedPubs.resize(COUNT);
    infoPubs.resize(COUNT);
    infos.resize(COUNT);
    statusPubs.resize(COUNT);

    for(size_t i = 0; i < COUNT; ++i)
    {
      if(i < DEPTH || i > DEPTH_HIRES)
      {
        imagePubs[i] = nh.advertise<sensor_msgs::Image>(topics[i] + K2_TOPIC_IMAGE, queueSize);
        compressedPubs[i] = nh.advertise<sensor_msgs::CompressedImage>(topics[i] + K2_TOPIC_RAW + K2_TOPIC_COMPRESSED, queueSize);
      }
      else
      {
        imagePubs[i] = nh.advertise<sensor_msgs::Image>(topics[i] + K2_TOPIC_RAW, queueSize);
        compressedPubs[i] = nh.advertise<sensor_msgs::CompressedImage>(topics[i] + K2_TOPIC_RAW + K2_TOPIC_COMP_DEPTH, queueSize);
      }
      infoPubs[i] = nh.advertise<sensor_msgs::CameraInfo>(topics[i] + K2_TOPIC_INFO, queueSize);
      statusPubs[i] = UNSUBCRIBED;
    }

    newNice = nice(oldNice - newNice);

    createCameraInfo();
    frame = 0;
    pubFrame = 0;

    lockProcess.lock();
    size_t numOfThreads = std::thread::hardware_concurrency();
    if(numOfThreads == 0)
    {
      std::cerr << "std::thread::hardware_concurrency() not returning a valid value. Using " << MIN_WORKER_THREADS << " worker threads." << std::endl;
    }
    numOfThreads = std::max(MIN_WORKER_THREADS, (int)numOfThreads);
    threads.resize(numOfThreads);
    for(size_t i = 0; i < threads.size(); ++i)
    {
      threads[i] = std::thread(&Kinect2Bridge::run_thread, this);
    }

    std::cout << "starting main loop" << std::endl;
    double nextFrame = ros::Time::now().toSec() + deltaT;
    for(;;)
    {
      cv::Mat color, depth, ir;

      bool newFrames = listener->waitForNewFrame(frames, 1000);

      if(!ros::ok())
      {
        if(newFrames)
        {
          listener->release(frames);
        }
        break;
      }
      if(!newFrames)
      {
        continue;
      }

      libfreenect2::Frame *colorFrame = frames[libfreenect2::Frame::Color];
      libfreenect2::Frame *irFrame = frames[libfreenect2::Frame::Ir];
      libfreenect2::Frame *depthFrame = frames[libfreenect2::Frame::Depth];

      cv::Mat colorMat = cv::Mat(colorFrame->height, colorFrame->width, CV_8UC3, colorFrame->data);
      cv::Mat irMat = cv::Mat(irFrame->height, irFrame->width, CV_32FC1, irFrame->data);
      cv::Mat depthMat = cv::Mat(depthFrame->height, depthFrame->width, CV_32FC1, depthFrame->data);

      cv::flip(colorMat, color, 1);
      cv::flip(irMat, ir, 1);
      cv::flip(depthMat, depth, 1);

      listener->release(frames);

      double now = ros::Time::now().toSec();
      if(now < nextFrame)
      {
        continue;
      }
      nextFrame += deltaT;

      lock.lock();
      if(!updateStatus())
      {
        lock.unlock();
        continue;
      }
      if(!newFrame)
      {
        newFrame = true;
        lockProcess.unlock();
      }
      createHeader();
      this->color = color;
      this->depth = depth;
      this->ir = ir;
      lock.unlock();
    }

    device->stop();
    device->close();
    delete listener;

    for(size_t i = 0; i < threads.size(); ++i)
    {
      lockProcess.unlock();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    for(size_t i = 0; i < threads.size(); ++i)
    {
      threads[i].join();
    }
    threads.clear();

    for(size_t i = 0; i < COUNT; ++i)
    {
      imagePubs[i].shutdown();
      compressedPubs[i].shutdown();
      infoPubs[i].shutdown();
    }

    nh.shutdown();
  }

private:
  void run_thread()
  {
    cv::Mat color, ir, depth;
    std::vector<cv::Mat> images(COUNT);
    std::vector<Status> status;
    size_t frame;
    std_msgs::Header header;

    int oldNice = nice(0);
    int newNice = nice(19 - oldNice);
    std::cout << "set process thread priority to: " << newNice << std::endl;

    for(; ros::ok();)
    {
      lockProcess.lock();
      if(!newFrame)
      {
        continue;
      }
      lock.lock();
      newFrame = false;
      frame = this->frame;
      color = this->color;
      ir = this->ir;
      depth = this->depth;
      header = this->header;
      status = statusPubs;
      ++(this->frame);
      lock.unlock();

      processImages(color, ir, depth, images, status);
      publishImages(images, header, status, frame);
    }
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

  void createCameraInfo()
  {
    cv::Mat projColor = cv::Mat::zeros(3, 4, CV_64F);
    cv::Mat projIr = cv::Mat::zeros(3, 4, CV_64F);
    cv::Mat projDepth = cv::Mat::zeros(3, 4, CV_64F);

    cameraMatrixColor.copyTo(projColor(cv::Rect(0, 0, 3, 3)));
    cameraMatrixIr.copyTo(projIr(cv::Rect(0, 0, 3, 3)));
    cameraMatrixDepth.copyTo(projDepth(cv::Rect(0, 0, 3, 3)));

    createCameraInfo(sizeColor, cameraMatrixColor, distortionColor, cv::Mat::eye(3, 3, CV_64F), projColor, infos[COLOR]);
    createCameraInfo(sizeColor, cameraMatrixColor, distortionColor, cv::Mat::eye(3, 3, CV_64F), projColor, infos[MONO]);
    createCameraInfo(sizeIr, cameraMatrixIr, distortionIr, cv::Mat::eye(3, 3, CV_64F), projIr, infos[IR]);
    infos[DEPTH] = infos[IR];

    createCameraInfo(sizeColor, cameraMatrixColor, cv::Mat::zeros(1, 5, CV_64F), cv::Mat::eye(3, 3, CV_64F), projColor, infos[COLOR_RECT]);
    createCameraInfo(sizeColor, cameraMatrixColor, cv::Mat::zeros(1, 5, CV_64F), cv::Mat::eye(3, 3, CV_64F), projColor, infos[MONO_RECT]);
    createCameraInfo(sizeIr, cameraMatrixIr, cv::Mat::zeros(1, 5, CV_64F), cv::Mat::eye(3, 3, CV_64F), projIr, infos[IR_RECT]);

    createCameraInfo(sizeDepth, cameraMatrixDepth, cv::Mat::zeros(1, 5, CV_64F), cv::Mat::eye(3, 3, CV_64F), projDepth, infos[DEPTH_RECT]);
    createCameraInfo(sizeDepth, cameraMatrixDepth, cv::Mat::zeros(1, 5, CV_64F), cv::Mat::eye(3, 3, CV_64F), projDepth, infos[COLOR_LORES]);
    createCameraInfo(sizeDepth, cameraMatrixDepth, cv::Mat::zeros(1, 5, CV_64F), cv::Mat::eye(3, 3, CV_64F), projDepth, infos[MONO_LORES]);
    createCameraInfo(sizeDepth, cameraMatrixDepth, cv::Mat::zeros(1, 5, CV_64F), cv::Mat::eye(3, 3, CV_64F), projDepth, infos[DEPTH_LORES]);

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

  bool updateStatus()
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

      statusPubs[i] = s;
      any = any || s != UNSUBCRIBED || infoPubs[i].getNumSubscribers() > 0;
    }
    return any;
  }

  void processImages(const cv::Mat &color, const cv::Mat &ir, const cv::Mat &depth, std::vector<cv::Mat> &images, const std::vector<Status> &status)
  {
    // IR
    if(status[IR] || status[IR_RECT])
    {
      ir.convertTo(images[IR], CV_16U);
    }
    if(status[IR_RECT])
    {
      cv::remap(images[IR], images[IR_RECT], map1Ir, map2Ir, cv::INTER_AREA);
    }

    // DEPTH
    if(status[DEPTH] || status[DEPTH_RECT] || status[DEPTH_LORES] || status[DEPTH_HIRES])
    {
      depth.convertTo(images[DEPTH], CV_16U);
    }
    if(status[DEPTH_RECT])
    {
      cv::remap(images[DEPTH], images[DEPTH_RECT], map1Ir, map2Ir, cv::INTER_NEAREST);
    }
    if(status[DEPTH_LORES])
    {
      lockRegLowRes.lock();
      depthRegLowRes->registerDepth(images[DEPTH], images[DEPTH_LORES]);
      lockRegLowRes.unlock();
    }
    if(status[DEPTH_HIRES])
    {
      lockRegHighRes.lock();
      depthRegHighRes->registerDepth(images[DEPTH], images[DEPTH_HIRES]);
      lockRegHighRes.unlock();
    }

    // COLOR
    images[COLOR] = color;
    if(status[COLOR_RECT] || status[MONO_RECT])
    {
      cv::remap(images[COLOR], images[COLOR_RECT], map1Color, map2Color, cv::INTER_AREA);
    }
    if(status[COLOR_LORES] || status[MONO_LORES])
    {
      cv::remap(images[COLOR], images[COLOR_LORES], map1ColorReg, map2ColorReg, cv::INTER_AREA);
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

  void createHeader()
  {
    header.seq = 0;
    header.stamp = ros::Time::now();
    header.frame_id = K2_TF_RGB_FRAME;
  }

  void publishImages(const std::vector<cv::Mat> &images, const std_msgs::Header &header, const std::vector<Status> &status, const size_t frame)
  {
    std::vector<sensor_msgs::Image> imageMsgs(COUNT);
    std::vector<sensor_msgs::CompressedImage> compressedMsgs(COUNT);
    std::vector<sensor_msgs::CameraInfo> infoMsgs(COUNT);

    //#pragma omp parallel for schedule(dynamic)
    for(size_t i = 0; i < COUNT; ++i)
    {
      infoMsgs[i] = infos[i];
      infoMsgs[i].header = header;
      infoMsgs[i].header.frame_id = i < DEPTH_LORES ? K2_TF_IR_FRAME : K2_TF_RGB_FRAME;

      switch(status[i])
      {
      case UNSUBCRIBED:
        break;
      case RAW:
        createImage(images[i], header, Image(i), imageMsgs[i]);
        break;
      case COMPRESSED:
        createCompressed(images[i], header, Image(i), compressedMsgs[i]);
        break;
      case BOTH:
        createImage(images[i], header, Image(i), imageMsgs[i]);
        createCompressed(images[i], header, Image(i), compressedMsgs[i]);
        break;
      }
    }

    while(frame != pubFrame)
    {
      std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
    lockPub.lock();
    for(size_t i = 0; i < COUNT; ++i)
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
    //std::cout << "published frame: " << pubFrame << " delay: " << (ros::Time::now().toSec() - header.stamp.toSec()) * 1000.0 << " ms." << std::endl;
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
};

void help(const std::string &path)
{
  std::cout << path << " [options]" << std::endl
            << "  -fps <num>       limit the frames per second to <num> (float)" << std::endl
            << "  -calib <path>    path to the calibration files" << std::endl
            << "  -raw             output raw depth image as 512x424 instead of 960x540" << std::endl
            << "  -comp <num>      jpg compression level from 0 to 100 (default 90)." << std::endl
            << "  -oclReg <num>    openCL device to use for depth registration." << std::endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "kinect2_bridge");

#ifdef K2_CALIB_PATH
  std::string path = K2_CALIB_PATH;
#else
  std::string path = "";
#endif
  std::string cam = "";
  double fps = -1;
  bool rawDepth = false;
  int compression = 90;
  int deviceId = -1;

  for(int argI = 1; argI < argc; ++argI)
  {
    const std::string arg = argv[argI];

    if(arg == "-h" || arg == "--help" || arg == "-?" || arg == "--?")
    {
      help(argv[0]);
      return 0;
    }
    else if(arg == "-cam")
    {
      if(++argI < argc)
      {
        cam = argv[argI];
      }
      else
      {
        std::cerr << "Camera serial number not given!" << std::endl;
        return -1;
      }
    }
    else if(arg == "-fps" && argI + 1 < argc)
    {
      fps = atof(argv[++argI]);
    }
    else if(arg == "-comp" && argI + 1 < argc)
    {
      compression = std::max(0, std::min(100, atoi(argv[++argI])));
    }
    else if(arg == "-calib")
    {
      if(++argI < argc)
      {
        path = argv[argI];
      }
      else
      {
        std::cerr << "Calibration data path not given!" << std::endl;
        return -1;
      }
    }
    else if(arg == "-oclReg")
    {
      if(++argI < argc)
      {
        deviceId = atoi(argv[argI]);
      }
      else
      {
        std::cerr << "Device ID not given!" << std::endl;
        return -1;
      }
    }
    else if(arg == "-raw")
    {
      rawDepth = true;
    }
  }

  struct stat fileStat;
  if(stat(path.c_str(), &fileStat) || !S_ISDIR(fileStat.st_mode))
  {
    std::cerr << "Calibration data path \"" << path << "\" does not exist." << std::endl;
    return -1;
  }

  if(!ros::ok())
  {
    std::cerr << "ros::ok failed!" << std::endl;
    return -1;
  }

  Kinect2Bridge kinect2(fps, rawDepth, compression);

  if(kinect2.init(path, cam, deviceId))
  {
    kinect2.run();
  }
  else
  {
    std::cerr << "Initialization failed!" << std::endl;
  }

  ros::shutdown();
  return 0;
}
