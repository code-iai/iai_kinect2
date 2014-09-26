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
#include <mutex>
#include <thread>

#include <dirent.h>
#include <sys/stat.h>

#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <ros/spinner.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

#include <camera_calibration_definitions.h>
#include <kinect2_definitions.h>


enum Mode
{
  RECORD,
  CALIBRATE
};

enum Source
{
  COLOR,
  IR,
  SYNC
};

class Recorder
{
private:
  const bool circleBoard;
  int circleFlags;

  const cv::Size boardDims;
  const float boardSize;
  const Source mode;

  const std::string path;
  const std::string topicColor, topicIr;
  std::mutex lock;

  bool update;
  bool running;
  bool foundColor, foundIr;
  cv::Mat color, ir, irGrey;
  cv::Ptr<cv::CLAHE> clahe;

  size_t frame;
  std::vector<int> params;

  std::vector<cv::Point3f> board;
  std::vector<cv::Point2f> pointsColor, pointsIr;

  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image> ColorIrSyncPolicy;
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner;
  image_transport::ImageTransport it;
  image_transport::SubscriberFilter *subImageColor, *subImageIr;
  message_filters::Synchronizer<ColorIrSyncPolicy> *sync;

  std::thread displayThread;

  int minIr, maxIr;

public:
  Recorder(const std::string &path, const std::string &topicColor, const std::string &topicIr, const Source mode, const bool circleBoard, const bool symmetric, const cv::Size &boardDims, const float boardSize)
    : circleBoard(circleBoard), boardDims(boardDims), boardSize(boardSize), mode(mode), path(path), topicColor(topicColor), topicIr(topicIr), update(false), foundColor(false), foundIr(false), frame(0), nh(), spinner(0), it(nh), minIr(0), maxIr(0x7FFF)
  {
    if(symmetric)
    {
      circleFlags = cv::CALIB_CB_SYMMETRIC_GRID + cv::CALIB_CB_CLUSTERING;
    }
    else
    {
      circleFlags = cv::CALIB_CB_ASYMMETRIC_GRID + cv::CALIB_CB_CLUSTERING;
    }

    params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    params.push_back(9);

    board.resize(boardDims.width * boardDims.height);
    for(size_t r = 0, i = 0; r < (size_t)boardDims.height; ++r)
    {
      for(size_t c = 0; c < (size_t)boardDims.width; ++c, ++i)
      {
        board[i] = cv::Point3f(c * boardSize, r * boardSize, 0);
      }
    }

    clahe = cv::createCLAHE(4.0, cv::Size(8, 8));
  }

  ~Recorder()
  {
  }

  void startRecord()
  {
    running = true;
    displayThread = std::thread(&Recorder::display, this);

    image_transport::TransportHints hints("compressed");
    image_transport::TransportHints hintsIr("compressed");
    subImageColor = new image_transport::SubscriberFilter(it, topicColor, 4, hints);
    subImageIr = new image_transport::SubscriberFilter(it, topicIr, 4, hintsIr);

    sync = new message_filters::Synchronizer<ColorIrSyncPolicy>(ColorIrSyncPolicy(4), *subImageColor, *subImageIr);
    sync->registerCallback(boost::bind(&Recorder::callback, this, _1, _2));

    spinner.start();
  }

  void stopRecord()
  {
    spinner.stop();

    delete sync;
    delete subImageColor;
    delete subImageIr;

    running = false;
    displayThread.join();
  }

  bool isRunning()
  {
    return running;
  }

private:
  void convertIr(const cv::Mat &ir, cv::Mat &grey, const int min, const int max)
  {
    const float factor = 255.0f / (max - min);
    grey.create(ir.rows, ir.cols, CV_8U);

    #pragma omp parallel for
    for(size_t r = 0; r < (size_t)ir.rows; ++r)
    {
      const uint16_t *itI = ir.ptr<uint16_t>(r);
      uint8_t *itO = grey.ptr<uint8_t>(r);

      for(size_t c = 0; c < (size_t)ir.cols; ++c, ++itI, ++itO)
      {
        *itO = std::min(std::max(*itI - min, 0) * factor, 255.0f);
      }
    }
  }

  void findMinMax(const cv::Mat &ir)
  {
    minIr = 0xFFFF;
    maxIr = 0;

    for(size_t r = 0; r < (size_t)ir.rows; ++r)
    {
      const uint16_t *it = ir.ptr<uint16_t>(r);

      for(size_t c = 0; c < (size_t)ir.cols; ++c, ++it)
      {
        minIr = std::min(minIr, (int) * it);
        maxIr = std::max(maxIr, (int) * it);
      }
    }
  }

  void callback(const sensor_msgs::Image::ConstPtr imageColor, const sensor_msgs::Image::ConstPtr imageIr)
  {
    std::vector<cv::Point2f> pointsColor, pointsIr;
    cv::Mat color, ir, irGrey, irScaled;
    bool foundColor = false;
    bool foundIr = false;

    if(mode == COLOR || mode == SYNC)
    {
      readImage(imageColor, color);
    }
    if(mode == IR || mode == SYNC)
    {
      readImage(imageIr, ir);
      cv::resize(ir, irScaled, cv::Size(), 2.0, 2.0, cv::INTER_CUBIC);
      convertIr(irScaled, irGrey, minIr, maxIr);
      //ir.convertTo(irGrey, CV_8U, 255.0 / maxIr);

      clahe->apply(irGrey, irGrey);
    }

    if(circleBoard)
    {
      switch(mode)
      {
      case COLOR:
        foundColor = cv::findCirclesGrid(color, boardDims, pointsColor, circleFlags);
        break;
      case IR:
        foundIr = cv::findCirclesGrid(irGrey, boardDims, pointsIr, circleFlags);
        break;
      case SYNC:
        foundColor = cv::findCirclesGrid(color, boardDims, pointsColor, circleFlags);
        foundIr = cv::findCirclesGrid(irGrey, boardDims, pointsIr, circleFlags);
        break;
      }
    }
    else
    {
      const cv::TermCriteria termCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::COUNT, 100, DBL_EPSILON);
      switch(mode)
      {
      case COLOR:
        foundColor = cv::findChessboardCorners(color, boardDims, pointsColor, cv::CALIB_CB_FAST_CHECK);
        break;
      case IR:
        foundIr = cv::findChessboardCorners(irGrey, boardDims, pointsIr, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE);
        break;
      case SYNC:
        foundColor = cv::findChessboardCorners(color, boardDims, pointsColor, cv::CALIB_CB_FAST_CHECK);
        foundIr = cv::findChessboardCorners(irGrey, boardDims, pointsIr, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE);
        break;
      }
      if(foundColor)
      {
        cv::cornerSubPix(color, pointsColor, cv::Size(11, 11), cv::Size(-1, -1), termCriteria);
      }
      if(foundIr)
      {
        cv::cornerSubPix(irGrey, pointsIr, cv::Size(11, 11), cv::Size(-1, -1), termCriteria);
      }
    }

    if(foundIr)
    {
      // Update min and max ir value based on checkerboard values
      cv::RotatedRect rect = cv::minAreaRect(pointsIr);
      float dist = sqrt(rect.size.width * rect.size.width + rect.size.height + rect.size.height) * 0.9;
      cv::Rect roi(rect.center.x - dist / 2, rect.center.y - dist / 2, dist, dist);
      findMinMax(irScaled(roi));
    }

    lock.lock();
    this->color = color;
    this->ir = ir;
    this->irGrey = irGrey;
    this->foundColor = foundColor;
    this->foundIr = foundIr;
    this->pointsColor = pointsColor;
    this->pointsIr = pointsIr;
    update = true;
    lock.unlock();
  }

  void display()
  {
    std::vector<cv::Point2f> pointsColor, pointsIr;
    cv::Mat color, ir, irGrey;
    cv::Mat colorDisp, irDisp;
    bool foundColor = false;
    bool foundIr = false;
    bool save = false;

    while(!update)
    {
      usleep(1000);
    }

    for(; running;)
    {
      if(update)
      {
        lock.lock();
        color = this->color;
        ir = this->ir;
        irGrey = this->irGrey;
        foundColor = this->foundColor;
        foundIr = this->foundIr;
        pointsColor = this->pointsColor;
        pointsIr = this->pointsIr;
        update = false;
        lock.unlock();

        if(mode == COLOR || mode == SYNC)
        {
          cv::cvtColor(color, colorDisp, CV_GRAY2BGR);
          cv::drawChessboardCorners(colorDisp, boardDims, pointsColor, foundColor);
          //cv::resize(colorDisp, colorDisp, cv::Size(), 0.5, 0.5);
          //cv::flip(colorDisp, colorDisp, 1);
        }
        if(mode == IR || mode == SYNC)
        {
          cv::cvtColor(irGrey, irDisp, CV_GRAY2BGR);
          cv::drawChessboardCorners(irDisp, boardDims, pointsIr, foundIr);
          //cv::resize(irDisp, irDisp, cv::Size(), 0.5, 0.5);
          //cv::flip(irDisp, irDisp, 1);
        }
      }

      switch(mode)
      {
      case COLOR:
        cv::imshow("color", colorDisp);
        break;
      case IR:
        cv::imshow("ir", irDisp);
        break;
      case SYNC:
        cv::imshow("color", colorDisp);
        cv::imshow("ir", irDisp);
        break;
      }

      int key = cv::waitKey(10);
      switch(key & 0xFF)
      {
      case ' ':
      case 's':
        save = true;
        break;
      case 27:
      case 'q':
        running = false;
        break;
      }

      if(save && ((mode == COLOR && foundColor) || (mode == IR && foundIr) || (mode == SYNC && foundColor && foundIr)))
      {
        store(color, ir, irGrey, pointsColor, pointsIr);
        save = false;
      }
    }
    cv::destroyAllWindows();
    cv::waitKey(100);
  }

  void readImage(const sensor_msgs::Image::ConstPtr msgImage, cv::Mat &image) const
  {
    cv_bridge::CvImageConstPtr pCvImage;
    pCvImage = cv_bridge::toCvShare(msgImage, msgImage->encoding);
    pCvImage->image.copyTo(image);
  }

  void store(const cv::Mat &color, const cv::Mat &ir, const cv::Mat &irGrey, const std::vector<cv::Point2f> &pointsColor, std::vector<cv::Point2f> &pointsIr)
  {
    std::ostringstream oss;
    oss << std::setfill('0') << std::setw(4) << frame++;
    const std::string frameNumber(oss.str());
    std::cout << "storing frame: " << frameNumber << std::endl;
    std::string base = path + frameNumber;

    for(size_t i = 0; i < pointsIr.size(); ++i)
    {
      pointsIr[i].x /= 2.0;
      pointsIr[i].y /= 2.0;
    }

    if(mode == SYNC)
    {
      base += CALIB_SYNC;
    }

    if(mode == COLOR || mode == SYNC)
    {
      cv::imwrite(base + CALIB_FILE_COLOR, color, params);

      cv::FileStorage file(base + CALIB_POINTS_COLOR, cv::FileStorage::WRITE);
      file << "points" << pointsColor;
    }

    if(mode == IR || mode == SYNC)
    {
      cv::imwrite(base + CALIB_FILE_IR, ir, params);
      cv::imwrite(base + CALIB_FILE_IR_GREY, irGrey, params);

      cv::FileStorage file(base + CALIB_POINTS_IR, cv::FileStorage::WRITE);
      file << "points" << pointsIr;
    }
  }
};

class Calibrator
{
private:
  const bool circleBoard;
  const cv::Size boardDims;
  const float boardSize;

  const Source mode;
  const std::string path;

  std::vector<cv::Point3f> board;

  std::vector<std::vector<cv::Point3f>> pointsBoard;
  std::vector<std::vector<cv::Point2f>> pointsColor;
  std::vector<std::vector<cv::Point2f>> pointsIr;

  cv::Size sizeColor;
  cv::Size sizeIr;

  cv::Mat cameraMatrixColor, distortionColor, rotationColor, translationColor, projectionColor;
  cv::Mat cameraMatrixIr, distortionIr, rotationIr, translationIr, projectionIr;
  cv::Mat rotation, translation, essential, fundamental, disparity;

public:
  Calibrator(const std::string &path, const Source mode, const bool circleBoard, const cv::Size &boardDims, const float boardSize)
    : circleBoard(circleBoard), boardDims(boardDims), boardSize(boardSize), mode(mode), path(path), sizeColor(1920, 1080), sizeIr(512, 424)
  {
    board.resize(boardDims.width * boardDims.height);
    for(size_t r = 0, i = 0; r < (size_t)boardDims.height; ++r)
    {
      for(size_t c = 0; c < (size_t)boardDims.width; ++c, ++i)
      {
        board[i] = cv::Point3f(c * boardSize, r * boardSize, 0);
      }
    }
  }

  ~Calibrator()
  {
  }

  bool restore()
  {
    std::vector<std::string> filesSync;
    std::vector<std::string> filesColor;
    std::vector<std::string> filesIr;

    DIR *dp;
    struct dirent *dirp;
    size_t posColor, posIr, posSync;

    if((dp  = opendir(path.c_str())) ==  NULL)
    {
      std::cerr << "Error opening: " << path << std::endl;
      return false;
    }

    while((dirp = readdir(dp)) != NULL)
    {
      std::string filename = dirp->d_name;

      if(dirp->d_type != DT_REG)
      {
        continue;
      }

      posSync = filename.rfind(CALIB_SYNC);
      posColor = filename.rfind(CALIB_FILE_COLOR);

      if(posSync != std::string::npos)
      {
        if(posColor != std::string::npos)
        {
          std::string frameName = filename.substr(0, posColor);
          filesSync.push_back(frameName);
        }
        continue;
      }

      if(posColor != std::string::npos)
      {
        std::string frameName = filename.substr(0, posColor);
        filesColor.push_back(frameName);
        continue;
      }

      posIr = filename.rfind(CALIB_FILE_IR_GREY);
      if(posIr != std::string::npos)
      {
        std::string frameName = filename.substr(0, posIr);
        filesIr.push_back(frameName);
        continue;
      }
    }
    closedir(dp);

    std::sort(filesColor.begin(), filesColor.end());
    std::sort(filesIr.begin(), filesIr.end());
    std::sort(filesSync.begin(), filesSync.end());

    bool ret = true;
    switch(mode)
    {
    case COLOR:
      pointsColor.resize(filesColor.size());
      pointsBoard.resize(filesColor.size(), board);
      ret = ret && readFiles(filesColor, CALIB_POINTS_COLOR, pointsColor);
      break;
    case IR:
      pointsIr.resize(filesIr.size());
      pointsBoard.resize(filesIr.size(), board);
      ret = ret && readFiles(filesIr, CALIB_POINTS_IR, pointsIr);
      break;
    case SYNC:
      pointsIr.resize(filesSync.size());
      pointsColor.resize(filesSync.size());
      pointsBoard.resize(filesSync.size(), board);
      ret = ret && readFiles(filesSync, CALIB_POINTS_COLOR, pointsColor);
      ret = ret && readFiles(filesSync, CALIB_POINTS_IR, pointsIr);
      break;
    }

    loadCalibration();
    return ret;
  }

  void calibrate()
  {
    switch(mode)
    {
    case COLOR:
      calibrateIntrinsics(sizeColor, pointsBoard, pointsColor, cameraMatrixColor, distortionColor, rotationColor, projectionColor);
      break;
    case IR:
      calibrateIntrinsics(sizeIr, pointsBoard, pointsIr, cameraMatrixIr, distortionIr, rotationIr, projectionIr);
      break;
    case SYNC:
      calibrateExtrinsics();
      break;
    }
    storeCalibration();
  }

  /*void showResults()
  {
    const cv::Size sizeColor(imagesColor[0].cols, imagesColor[0].rows), sizeIr(imagesIr[0].cols, imagesIr[0].rows);
    cv::Mat map1Color, map2Color, map1UndistColor, map2UndistColor;
    cv::Mat map1Ir, map2Ir, map1UndistIr, map2UndistIr;
    cv::Mat color, ir, dispColor, dispIr;

    //cv::getOptimalNewCameraMatrix(cameraMatrixIr, distortionIr, sizeIr, -1, )
    //cv::initUndistortRectifyMap(cameraMatrixIr, distortionIr, cv::Mat(), cameraMatrixIr, sizeIr, CV_16SC2, map1UndistIr, map2UndistIr);
    cv::initUndistortRectifyMap(cameraMatrixColor, distortionColor, cv::Mat(), cameraMatrixColor, sizeColor, CV_16SC2, map1UndistColor, map2UndistColor);
    cv::initUndistortRectifyMap(cameraMatrixIr, distortionIr, cv::Mat(), cameraMatrixIr, sizeIr, CV_16SC2, map1UndistIr, map2UndistIr);

    //cv::initUndistortRectifyMap(cameraMatrixColor, distortionColor, rotationColor, projectionColor, sizeColor, CV_16SC2, map1Color, map2Color);
    //cv::initUndistortRectifyMap(cameraMatrixIr, distortionIr, rotationIr, projectionIr, sizeColor, CV_16SC2, map1Ir, map2Ir);
    cv::initUndistortRectifyMap(cameraMatrixColor, distortionColor, rotationColor, projectionColor, sizeIr, CV_16SC2, map1Color, map2Color);
    cv::initUndistortRectifyMap(cameraMatrixIr, distortionIr, rotationIr, projectionIr, sizeIr, CV_16SC2, map1Ir, map2Ir);

    size_t index = 0;
    const size_t size = imagesSyncColor.size();
    int mode = 1;

    for(bool run = true; run;)
    {
      color = imagesSyncColor[index];
      ir = imagesSyncIr[index];

      if(mode == 0)
      {
        cv::remap(color, dispColor, map1UndistColor, map2UndistColor, cv::INTER_LANCZOS4);
        cv::remap(ir, dispIr, map1UndistIr, map2UndistIr, cv::INTER_LANCZOS4);
      }
      else if(mode == 1)
      {
        cv::remap(color, dispColor, map1Color, map2Color, cv::INTER_LANCZOS4);
        cv::remap(ir, dispIr, map1UndistIr, map2UndistIr, cv::INTER_LANCZOS4);
        cv::remap(dispIr, dispIr, map1Ir, map2Ir, cv::INTER_LANCZOS4);
      }

      cv::imshow("color", dispColor);
      cv::imshow("ir", dispIr);

      int key = cv::waitKey();
      switch(key & 0xFF)
      {
      case 'u':
        mode = 0;
        break;
      case 'r':
        mode = 1;
        break;
      case 27:
        run = false;
        break;
      case 81: // right arrow
        index = (index + 1) % size;
        break;
      case 83: // left arrow
        index = (size + index - 1) % size;
        break;
      }
    }
    cv::destroyAllWindows();
    cv::waitKey(100);
  }*/

private:
  bool readFiles(const std::vector<std::string> &files, const std::string &ext, std::vector<std::vector<cv::Point2f>> &points) const
  {
    bool ret = true;
    #pragma omp parallel for
    for(size_t i = 0; i < files.size(); ++i)
    {
      std::string pointsname = path + files[i] + ext;

      #pragma omp critical
      std::cout << "restoring file: " << files[i] << ext << std::endl;

      cv::FileStorage file(pointsname, cv::FileStorage::READ);
      file["points"] >> points[i];
    }
    return ret;
  }

  void calibrateIntrinsics(const cv::Size &size, const std::vector<std::vector<cv::Point3f>> &pointsBoard, const  std::vector<std::vector<cv::Point2f>> &points,
                           cv::Mat &cameraMatrix, cv::Mat &distortion, cv::Mat &rotation, cv::Mat &projection)
  {
    std::vector<cv::Mat> rvecs, tvecs;
    const cv::TermCriteria termCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 50, DBL_EPSILON);
    double error;

    std::cout << "calibrating intrinsics..." << std::endl;
    error = cv::calibrateCamera(pointsBoard, points, size, cameraMatrix, distortion, rvecs, tvecs, 0, termCriteria);
    std::cout << "error: " << error << std::endl << std::endl;
    //projectionError(pointsColor, rvecs, tvecs, cameraMatrix, distortion);

    std::cout << "Camera Matrix:" << std::endl << cameraMatrix << std::endl;
    std::cout << "Distortion Coeeficients:" << std::endl << distortion << std::endl << std::endl;
    rotation = cv::Mat::eye(3, 3, CV_64F);
    projection = cv::Mat::eye(4, 4, CV_64F);
    cameraMatrix.copyTo(projection(cv::Rect(0, 0, 3, 3)));
  }

  void calibrateExtrinsics()
  {
    if(pointsColor.size() != pointsIr.size())
    {
      std::cerr << "not the same size!" << std::endl;
      return;
    }
    const cv::TermCriteria termCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 50, DBL_EPSILON);
    double error;

    std::cout << "Camera Matrix Color:" << std::endl << cameraMatrixColor << std::endl;
    std::cout << "Distortion Coeeficients Color:" << std::endl << distortionColor << std::endl << std::endl;
    std::cout << "Camera Matrix Ir:" << std::endl << cameraMatrixIr << std::endl;
    std::cout << "Distortion Coeeficients Ir:" << std::endl << distortionIr << std::endl << std::endl;

    std::cout << "calibrating Color and Ir extrinsics..." << std::endl;
    error = cv::stereoCalibrate(pointsBoard, pointsIr, pointsColor, cameraMatrixIr, distortionIr, cameraMatrixColor, distortionColor, sizeColor,
                                rotation, translation, essential, fundamental, termCriteria,
                                cv::CALIB_FIX_INTRINSIC);
    std::cout << "error: " << error << std::endl << std::endl;

    std::cout << "Rotation:" << std::endl << rotation << std::endl;
    std::cout << "Translation:" << std::endl << translation << std::endl;
    std::cout << "Essential:" << std::endl << essential << std::endl;
    std::cout << "Fundamental:" << std::endl << fundamental << std::endl << std::endl;
  }

  void projectionError(const std::vector<std::vector<cv::Point2f>> &points, const std::vector<cv::Mat> &rvecs, const std::vector<cv::Mat> &tvecs, const cv::Mat &cameraMatrix, const cv::Mat &distortion) const
  {
    std::vector<cv::Point2f> imgPoints;

    for(size_t i = 0; i < points.size(); ++i)
    {
      const std::vector<cv::Point2f> &relPoints = points[i];
      const cv::Mat &rvec = rvecs[i];
      const cv::Mat &tvec = tvecs[i];

      imgPoints.clear();
      cv::projectPoints(board, rvec, tvec, cameraMatrix, distortion, imgPoints);

      double error = 0;
      for(size_t j = 0; j < relPoints.size(); ++j)
      {
        const cv::Point2f diff = relPoints[j] - imgPoints[j];
        error += sqrt(diff.dot(diff));
      }
      error /= relPoints.size();
      std::cout << "image: " << i << " error: " << error << std::endl;
    }
  }

  void storeCalibration()
  {
    cv::FileStorage fs;

    if(fs.open(path + CALIB_POSE, cv::FileStorage::WRITE))
    {
      fs << CALIB_ROTATION << rotation;
      fs << CALIB_TRANSLATION << translation;
      fs << CALIB_ESSENTIAL << essential;
      fs << CALIB_FUNDAMENTAL << fundamental;
      fs.release();
    }

    if(fs.open(path + CALIB_COLOR, cv::FileStorage::WRITE))
    {
      fs << CALIB_CAMERA_MATRIX << cameraMatrixColor;
      fs << CALIB_DISTORTION << distortionColor;
      fs << CALIB_ROTATION << rotationColor;
      fs << CALIB_PROJECTION << projectionColor;
      fs.release();
    }

    if(fs.open(path + CALIB_IR, cv::FileStorage::WRITE))
    {
      fs << CALIB_CAMERA_MATRIX << cameraMatrixIr;
      fs << CALIB_DISTORTION << distortionIr;
      fs << CALIB_ROTATION << rotationIr;
      fs << CALIB_PROJECTION << projectionIr;
      fs.release();
    }
  }

  void loadCalibration()
  {
    cv::FileStorage fs;

    if(fs.open(path + CALIB_COLOR, cv::FileStorage::READ))
    {
      fs[CALIB_CAMERA_MATRIX] >> cameraMatrixColor;
      fs[CALIB_DISTORTION] >> distortionColor;
      fs[CALIB_ROTATION] >> rotationColor;
      fs[CALIB_PROJECTION] >> projectionColor;
      fs.release();
    }

    if(fs.open(path + CALIB_IR, cv::FileStorage::READ))
    {
      fs[CALIB_CAMERA_MATRIX] >> cameraMatrixIr;
      fs[CALIB_DISTORTION] >> distortionIr;
      fs[CALIB_ROTATION] >> rotationIr;
      fs[CALIB_PROJECTION] >> projectionIr;
      fs.release();
    }
  }
};

void help(const std::string &path)
{
  std::cout << path << " [options]" << std::endl
            << "  mode: 'record' or 'calibrate'" << std::endl
            << "  source: 'color', 'ir', 'sync'" << std::endl
            << "  board:" << std::endl
            << "    'circle<WIDTH>x<HEIGHT>x<SIZE>'  for symmentric cirle grid" << std::endl
            << "    'acircle<WIDTH>x<HEIGHT>x<SIZE>' for asymmentric cirle grid" << std::endl
            << "    'chess<WIDTH>x<HEIGHT>x<SIZE>'   for chessboard pattern" << std::endl
            << "  topics: '-color <TOPIC>' and/or '-ir <TOPIC>'" << std::endl
            << "  output path: '<PATH>'" << std::endl;
}

int main(int argc, char **argv)
{
  Mode mode = RECORD;
  Source source = SYNC;
  bool circleBoard = false;
  bool symmetric = true;
  cv::Size boardDims = cv::Size(7, 6);
  float boardSize = 0.108;
  std::string path = "./";
  std::string topicColor = K2_TOPIC_IMAGE_MONO K2_TOPIC_RAW;
  std::string topicIr = K2_TOPIC_IMAGE_IR K2_TOPIC_RAW;

  ros::init(argc, argv, "kinect2_calib");

  if(!ros::ok())
  {
    return 0;
  }

  for(int argI = 1; argI < argc; ++ argI)
  {
    std::string arg(argv[argI]);

    if(arg == "--help" || arg == "--h" || arg == "-h" || arg == "-?" || arg == "--?")
    {
      help(argv[0]);
      ros::shutdown();
      return 0;
    }
    else if(arg == "record")
    {
      mode = RECORD;
    }
    else if(arg == "calibrate")
    {
      mode = CALIBRATE;
    }
    else if(arg == "color")
    {
      source = COLOR;
    }
    else if(arg == "ir")
    {
      source = IR;
    }
    else if(arg == "sync")
    {
      source = SYNC;
    }
    else if(arg.find("circle") == 0 && arg.find('x') != arg.rfind('x') && arg.rfind('x') != std::string::npos)
    {
      circleBoard = true;
      const size_t start = 6;
      const size_t leftX = arg.find('x');
      const size_t rightX = arg.rfind('x');
      const size_t end = arg.size();

      int width = atoi(arg.substr(start, leftX - start).c_str());
      int height = atoi(arg.substr(leftX + 1, rightX - leftX + 1).c_str());
      boardSize = atof(arg.substr(rightX + 1, end - rightX + 1).c_str());
      boardDims = cv::Size(width, height);
    }
    else if((arg.find("circle") == 0 || arg.find("acircle") == 0) && arg.find('x') != arg.rfind('x') && arg.rfind('x') != std::string::npos)
    {
      symmetric = arg.find("circle") == 0;
      circleBoard = true;
      const size_t start = 6;
      const size_t leftX = arg.find('x');
      const size_t rightX = arg.rfind('x');
      const size_t end = arg.size();

      int width = atoi(arg.substr(start, leftX - start).c_str());
      int height = atoi(arg.substr(leftX + 1, rightX - leftX + 1).c_str());
      boardSize = atof(arg.substr(rightX + 1, end - rightX + 1).c_str());
      boardDims = cv::Size(width, height);
    }
    else if(arg.find("chess") == 0 && arg.find('x') != arg.rfind('x') && arg.rfind('x') != std::string::npos)
    {
      circleBoard = false;
      const size_t start = 5;
      const size_t leftX = arg.find('x');
      const size_t rightX = arg.rfind('x');
      const size_t end = arg.size();

      int width = atoi(arg.substr(start, leftX - start).c_str());
      int height = atoi(arg.substr(leftX + 1, rightX - leftX + 1).c_str());
      boardSize = atof(arg.substr(rightX + 1, end - rightX + 1).c_str());
      boardDims = cv::Size(width, height);
    }
    else if(arg == "-color" && argI + 1 < argc)
    {
      topicColor = std::string(argv[++argI]);
    }
    else if(arg == "-ir" && argI + 1 < argc)
    {
      topicIr = std::string(argv[++argI]);
    }
    else
    {
      struct stat fileStat;
      if(stat(arg.c_str(), &fileStat) == 0 && S_ISDIR(fileStat.st_mode))
      {
        path = arg;
      }
      else
      {
        std::cerr << "Unknown argument: " << arg << std::endl;
        help(argv[0]);
        ros::shutdown();
        return 0;
      }
    }
  }

  std::cout << "Start settings:" << std::endl
            << "Mode: " << (mode == RECORD ? "record" : "calibrate") << std::endl
            << "Source: " << (source == COLOR ? "color" : (source == IR ? "ir" : "sync")) << std::endl
            << "Board: " << (circleBoard ? "circles" : "chess") << std::endl
            << "Dimensions: " << boardDims.width << " x " << boardDims.height << std::endl
            << "Field size: " << boardSize << std::endl
            << "Topic color: " << topicColor << std::endl
            << "Topic ir: " << topicIr << std::endl
            << "Path: " << path << std::endl << std::endl;

  if(!ros::master::check())
  {
    std::cerr << "checking ros master failed." << std::endl;
    return -1;
  }
  if(mode == RECORD)
  {
    Recorder recorder(path, topicColor, topicIr, source, circleBoard, symmetric, boardDims, boardSize);

    std::cout << "starting recorder..." << std::endl;
    recorder.startRecord();

    std::cout << "starting loop..." << std::endl;
    while(ros::ok() && recorder.isRunning())
    {
      usleep(10000);
    }

    std::cout << "stopping recording..." << std::endl;
    recorder.stopRecord();
  }
  else
  {
    Calibrator calib(path, source, circleBoard, boardDims, boardSize);

    std::cout << "restoring files..." << std::endl;
    calib.restore();

    std::cout << "starting calibration..." << std::endl;
    calib.calibrate();
  }

  ros::shutdown();
  return 0;
}
