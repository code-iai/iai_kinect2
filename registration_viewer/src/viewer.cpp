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
#include <cmath>
#include <mutex>
#include <thread>
#include <chrono>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

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
#include <message_filters/sync_policies/approximate_time.h>

#include <kinect2_definitions.h>

class Receiver
{
public:
  enum Mode
  {
    IMAGE = 0,
    CLOUD,
    BOTH
  };

private:
  std::mutex lock;

  const std::string topicColor, topicDepth;
  const bool useExact, useCompressed;

  bool updateImage, updateCloud;
  bool save;
  bool running;
  size_t frame;
  const size_t queueSize;

  cv::Mat color, depth;
  cv::Mat cameraMatrixColor, cameraMatrixDepth;
  cv::Mat lookupX, lookupY;

  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ExactSyncPolicy;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ApproximateSyncPolicy;

  ros::NodeHandle nh;
  ros::AsyncSpinner spinner;
  image_transport::ImageTransport it;
  image_transport::SubscriberFilter *subImageColor, *subImageDepth;
  message_filters::Subscriber<sensor_msgs::CameraInfo> *subCameraInfoColor, *subCameraInfoDepth;

  message_filters::Synchronizer<ExactSyncPolicy> *syncExact;
  message_filters::Synchronizer<ApproximateSyncPolicy> *syncApproximate;

  std::thread imageViewerThread;
  Mode mode;

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;
  pcl::PCDWriter writer;
  std::ostringstream oss;
  std::vector<int> params;

public:
  Receiver(const std::string &topicColor, const std::string &topicDepth, const bool useExact, const bool useCompressed)
    : topicColor(topicColor), topicDepth(topicDepth), useExact(useExact), useCompressed(useCompressed),
      updateImage(false), updateCloud(false), save(false), running(false), frame(0), queueSize(5),
      nh("~"), spinner(0), it(nh), mode(CLOUD)
  {
    cameraMatrixColor = cv::Mat::zeros(3, 3, CV_64F);
    cameraMatrixDepth = cv::Mat::zeros(3, 3, CV_64F);
    params.push_back(cv::IMWRITE_JPEG_QUALITY);
    params.push_back(100);
    params.push_back(cv::IMWRITE_PNG_COMPRESSION);
    params.push_back(1);
    params.push_back(cv::IMWRITE_PNG_STRATEGY);
    params.push_back(cv::IMWRITE_PNG_STRATEGY_RLE);
    params.push_back(0);
  }

  ~Receiver()
  {
  }

  void run(const Mode mode)
  {
    start(mode);
    stop();
  }

private:
  void start(const Mode mode)
  {
    this->mode = mode;
    running = true;

    std::string topicCameraInfoColor = topicColor.substr(0, topicColor.rfind('/')) + "/camera_info";
    std::string topicCameraInfoDepth = topicDepth.substr(0, topicDepth.rfind('/')) + "/camera_info";

    image_transport::TransportHints hints(useCompressed ? "compressed" : "raw");
    image_transport::TransportHints hintsDepth(useCompressed ? "compressedDepth" : "raw");
    subImageColor = new image_transport::SubscriberFilter(it, topicColor, queueSize, hints);
    subImageDepth = new image_transport::SubscriberFilter(it, topicDepth, queueSize, hintsDepth);
    subCameraInfoColor = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh, topicCameraInfoColor, queueSize);
    subCameraInfoDepth = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh, topicCameraInfoDepth, queueSize);

    if(useExact)
    {
      syncExact = new message_filters::Synchronizer<ExactSyncPolicy>(ExactSyncPolicy(queueSize), *subImageColor, *subImageDepth, *subCameraInfoColor, *subCameraInfoDepth);
      syncExact->registerCallback(boost::bind(&Receiver::callback, this, _1, _2, _3, _4));
    }
    else
    {
      syncApproximate = new message_filters::Synchronizer<ApproximateSyncPolicy>(ApproximateSyncPolicy(queueSize), *subImageColor, *subImageDepth, *subCameraInfoColor, *subCameraInfoDepth);
      syncApproximate->registerCallback(boost::bind(&Receiver::callback, this, _1, _2, _3, _4));
    }

    spinner.start();

    std::chrono::milliseconds duration(1);
    while(!updateImage || !updateCloud)
    {
      if(!ros::ok())
      {
        return;
      }
      std::this_thread::sleep_for(duration);
    }
    cloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>());
    cloud->height = color.rows;
    cloud->width = color.cols;
    cloud->is_dense = false;
    cloud->points.resize(cloud->height * cloud->width);
    createLookup(this->color.cols, this->color.rows);

    switch(mode)
    {
    case CLOUD:
      cloudViewer();
      break;
    case IMAGE:
      imageViewer();
      break;
    case BOTH:
      imageViewerThread = std::thread(&Receiver::imageViewer, this);
      cloudViewer();
      break;
    }
  }

  void stop()
  {
    spinner.stop();

    if(useExact)
    {
      delete syncExact;
    }
    else
    {
      delete syncApproximate;
    }

    delete subImageColor;
    delete subImageDepth;
    delete subCameraInfoColor;
    delete subCameraInfoDepth;

    running = false;
    if(mode == BOTH)
    {
      imageViewerThread.join();
    }
  }

  void callback(const sensor_msgs::Image::ConstPtr imageColor, const sensor_msgs::Image::ConstPtr imageDepth,
                const sensor_msgs::CameraInfo::ConstPtr cameraInfoColor, const sensor_msgs::CameraInfo::ConstPtr cameraInfoDepth)
  {
    cv::Mat color, depth;

    readCameraInfo(cameraInfoColor, cameraMatrixColor);
    readCameraInfo(cameraInfoDepth, cameraMatrixDepth);
    readImage(imageColor, color);
    readImage(imageDepth, depth);

    // IR image input
    if(color.type() == CV_16U)
    {
      cv::Mat tmp;
      color.convertTo(tmp, CV_8U, 0.02);
      cv::cvtColor(tmp, color, CV_GRAY2BGR);
    }

    lock.lock();
    this->color = color;
    this->depth = depth;
    updateImage = true;
    updateCloud = true;
    lock.unlock();
  }

  void imageViewer()
  {
    cv::Mat color, depth, depthDisp, combined;
    std::chrono::time_point<std::chrono::high_resolution_clock> start, now;
    double fps = 0;
    size_t frameCount = 0;
    std::ostringstream oss;
    const cv::Point pos(5, 15);
    const cv::Scalar colorText = CV_RGB(255, 255, 255);
    const double sizeText = 0.5;
    const int lineText = 1;
    const int font = cv::FONT_HERSHEY_SIMPLEX;

    cv::namedWindow("Image Viewer");
    oss << "starting...";

    start = std::chrono::high_resolution_clock::now();
    for(; running && ros::ok();)
    {
      if(updateImage)
      {
        lock.lock();
        color = this->color;
        depth = this->depth;
        updateImage = false;
        lock.unlock();

        ++frameCount;
        now = std::chrono::high_resolution_clock::now();
        double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count() / 1000.0;
        if(elapsed >= 1.0)
        {
          fps = frameCount / elapsed;
          oss.str("");
          oss << "fps: " << fps << " ( " << elapsed / frameCount * 1000.0 << " ms)";
          start = now;
          frameCount = 0;
        }

        dispDepth(depth, depthDisp, 12000.0f);
        combine(color, depthDisp, combined);
        //combined = color;

        cv::putText(combined, oss.str(), pos, font, sizeText, colorText, lineText, CV_AA);
        cv::imshow("Image Viewer", combined);
      }

      int key = cv::waitKey(1);
      switch(key & 0xFF)
      {
      case 27:
      case 'q':
        running = false;
        break;
      case ' ':
      case 's':
        if(mode == IMAGE)
        {
          createCloud(depth, color, cloud);
          saveCloudAndImages(cloud, color, depth, depthDisp);
        }
        else
        {
          save = true;
        }
        break;
      }
    }
    cv::destroyAllWindows();
    cv::waitKey(100);
  }

  void cloudViewer()
  {
    cv::Mat color, depth;
    pcl::visualization::PCLVisualizer::Ptr visualizer(new pcl::visualization::PCLVisualizer("Cloud Viewer"));
    const std::string cloudName = "rendered";

    lock.lock();
    color = this->color;
    depth = this->depth;
    updateCloud = false;
    lock.unlock();

    createCloud(depth, color, cloud);

    visualizer->addPointCloud(cloud, cloudName);
    visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloudName);
    visualizer->initCameraParameters();
    visualizer->setBackgroundColor(0, 0, 0);
    visualizer->setPosition(mode == BOTH ? color.cols : 0, 0);
    visualizer->setSize(color.cols, color.rows);
    visualizer->setShowFPS(true);
    visualizer->setCameraPosition(0, 0, 0, 0, -1, 0);
    visualizer->registerKeyboardCallback(&Receiver::keyboardEvent, *this);

    for(; running && ros::ok();)
    {
      if(updateCloud)
      {
        lock.lock();
        color = this->color;
        depth = this->depth;
        updateCloud = false;
        lock.unlock();

        createCloud(depth, color, cloud);

        visualizer->updatePointCloud(cloud, cloudName);
      }
      if(save)
      {
        save = false;
        cv::Mat depthDisp;
        dispDepth(depth, depthDisp, 12000.0f);
        saveCloudAndImages(cloud, color, depth, depthDisp);
      }
      visualizer->spinOnce(10);
    }
    visualizer->close();
  }

  void keyboardEvent(const pcl::visualization::KeyboardEvent &event, void *)
  {
    if(event.keyUp())
    {
      switch(event.getKeyCode())
      {
      case 27:
      case 'q':
        running = false;
        break;
      case ' ':
      case 's':
        save = true;
        break;
      }
    }
  }

  void readImage(const sensor_msgs::Image::ConstPtr msgImage, cv::Mat &image) const
  {
    cv_bridge::CvImageConstPtr pCvImage;
    pCvImage = cv_bridge::toCvShare(msgImage, msgImage->encoding);
    pCvImage->image.copyTo(image);
  }

  void readCameraInfo(const sensor_msgs::CameraInfo::ConstPtr cameraInfo, cv::Mat &cameraMatrix) const
  {
    double *itC = cameraMatrix.ptr<double>(0, 0);
    for(size_t i = 0; i < 9; ++i, ++itC)
    {
      *itC = cameraInfo->K[i];
    }
  }

  void dispDepth(const cv::Mat &in, cv::Mat &out, const float maxValue)
  {
    cv::Mat tmp = cv::Mat(in.rows, in.cols, CV_8U);
    const uint32_t maxInt = 255;

    #pragma omp parallel for
    for(int r = 0; r < in.rows; ++r)
    {
      const uint16_t *itI = in.ptr<uint16_t>(r);
      uint8_t *itO = tmp.ptr<uint8_t>(r);

      for(int c = 0; c < in.cols; ++c, ++itI, ++itO)
      {
        *itO = (uint8_t)std::min((*itI * maxInt / maxValue), 255.0f);
      }
    }

    cv::applyColorMap(tmp, out, cv::COLORMAP_JET);
  }

  void combine(const cv::Mat &inC, const cv::Mat &inD, cv::Mat &out)
  {
    out = cv::Mat(inC.rows, inC.cols, CV_8UC3);

    #pragma omp parallel for
    for(int r = 0; r < inC.rows; ++r)
    {
      const cv::Vec3b
      *itC = inC.ptr<cv::Vec3b>(r),
       *itD = inD.ptr<cv::Vec3b>(r);
      cv::Vec3b *itO = out.ptr<cv::Vec3b>(r);

      for(int c = 0; c < inC.cols; ++c, ++itC, ++itD, ++itO)
      {
        itO->val[0] = (itC->val[0] + itD->val[0]) >> 1;
        itO->val[1] = (itC->val[1] + itD->val[1]) >> 1;
        itO->val[2] = (itC->val[2] + itD->val[2]) >> 1;
      }
    }
  }

  void createCloud(const cv::Mat &depth, const cv::Mat &color, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud) const
  {
    const float badPoint = std::numeric_limits<float>::quiet_NaN();

    #pragma omp parallel for
    for(int r = 0; r < depth.rows; ++r)
    {
      pcl::PointXYZRGBA *itP = &cloud->points[r * depth.cols];
      const uint16_t *itD = depth.ptr<uint16_t>(r);
      const cv::Vec3b *itC = color.ptr<cv::Vec3b>(r);
      const float y = lookupY.at<float>(0, r);
      const float *itX = lookupX.ptr<float>();

      for(size_t c = 0; c < (size_t)depth.cols; ++c, ++itP, ++itD, ++itC, ++itX)
      {
        register const float depthValue = *itD / 1000.0f;
        // Check for invalid measurements
        if(isnan(depthValue) || depthValue <= 0.001)
        {
          // not valid
          itP->x = itP->y = itP->z = badPoint;
          itP->rgba = 0;
          continue;
        }
        itP->z = depthValue;
        itP->x = *itX * depthValue;
        itP->y = y * depthValue;
        itP->b = itC->val[0];
        itP->g = itC->val[1];
        itP->r = itC->val[2];
        itP->a = 0;
      }
    }
  }

  void saveCloudAndImages(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud, const cv::Mat &color, const cv::Mat &depth, const cv::Mat &depthColored)
  {
    oss.str("");
    oss << "./" << std::setfill('0') << std::setw(4) << frame;
    const std::string baseName = oss.str();
    const std::string cloudName = baseName + "_cloud.pcd";
    const std::string colorName = baseName + "_color.jpg";
    const std::string depthName = baseName + "_depth.png";
    const std::string depthColoredName = baseName + "_depth_colored.png";

    std::cout << "saving cloud: " << cloudName << std::endl;
    writer.writeBinary(cloudName, *cloud);
    std::cout << "saving color: " << colorName << std::endl;
    cv::imwrite(colorName, color, params);
    std::cout << "saving depth: " << depthName << std::endl;
    cv::imwrite(depthName, depth, params);
    std::cout << "saving depth: " << depthColoredName << std::endl;
    cv::imwrite(depthColoredName, depthColored, params);
    std::cout << "saving complete!" << std::endl;
    ++frame;
  }

  void createLookup(size_t width, size_t height)
  {
    const float fx = 1.0f / cameraMatrixColor.at<double>(0, 0);
    const float fy = 1.0f / cameraMatrixColor.at<double>(1, 1);
    const float cx = cameraMatrixColor.at<double>(0, 2);
    const float cy = cameraMatrixColor.at<double>(1, 2);
    float *it;

    lookupY = cv::Mat(1, height, CV_32F);
    it = lookupY.ptr<float>();
    for(size_t r = 0; r < height; ++r, ++it)
    {
      *it = (r - cy) * fy;
    }

    lookupX = cv::Mat(1, width, CV_32F);
    it = lookupX.ptr<float>();
    for(size_t c = 0; c < width; ++c, ++it)
    {
      *it = (c - cx) * fx;
    }
  }
};

void help(const std::string &path)
{
  std::cout << path << " [options]" << std::endl
            << "  name: 'any string' equals to the kinect2_bridge topic base name" << std::endl
            << "  mode: 'sd', 'hd', or 'ir'" << std::endl
            << "  visualization: 'image', 'cloud' or 'both'" << std::endl
            << "  options:" << std::endl
            << "    'raw' use raw instead of compressed topics" << std::endl
            << "    'approx' use approximate time synchronization" << std::endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "viewer", ros::init_options::AnonymousName);

  if(!ros::ok())
  {
    return 0;
  }

  std::string ns = K2_DEFAULT_NS;
  std::string topicColor = K2_TOPIC_LORES_COLOR K2_TOPIC_RAW;
  std::string topicDepth = K2_TOPIC_LORES_DEPTH K2_TOPIC_RAW;
  bool useExact = true;
  bool useCompressed = true;
  Receiver::Mode mode = Receiver::CLOUD;

  for(size_t i = 1; i < (size_t)argc; ++i)
  {
    std::string param(argv[i]);

    if(param == "-h" || param == "--help" || param == "-?" || param == "--?")
    {
      help(argv[0]);
      ros::shutdown();
      return 0;
    }
    else if(param == "sd")
    {
      topicColor = K2_TOPIC_LORES_COLOR K2_TOPIC_RAW;
      topicDepth = K2_TOPIC_LORES_DEPTH K2_TOPIC_RAW;
    }
    else if(param == "hd")
    {
      topicColor = K2_TOPIC_RECT_COLOR K2_TOPIC_RAW;
      topicDepth = K2_TOPIC_HIRES_DEPTH K2_TOPIC_RAW;
    }
    else if(param == "ir")
    {
      topicColor = K2_TOPIC_RECT_IR K2_TOPIC_RAW;
      topicDepth = K2_TOPIC_RECT_DEPTH K2_TOPIC_RAW;
    }
    else if(param == "approx")
    {
      useExact = false;
    }
    else if(param == "raw")
    {
      useCompressed = false;
    }
    else if(param == "image")
    {
      mode = Receiver::IMAGE;
    }
    else if(param == "cloud")
    {
      mode = Receiver::CLOUD;
    }
    else if(param == "both")
    {
      mode = Receiver::BOTH;
    }
    else
    {
      ns = param;
    }
  }

  topicColor = "/" + ns + topicColor;
  topicDepth = "/" + ns + topicDepth;
  std::cout << "topic color: " << topicColor << std::endl;
  std::cout << "topic depth: " << topicDepth << std::endl;

  Receiver receiver(topicColor, topicDepth, useExact, useCompressed);

  std::cout << "starting receiver..." << std::endl;
  receiver.run(mode);

  ros::shutdown();
  return 0;
}
