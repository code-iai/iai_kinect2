/**
 * Copyright 2015 Kyushu Institute of Technology,
 * Author: Nishanth Koganti <buntyke@gmail.com>
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

// CPP headers
#include <cmath>
#include <string>
#include <vector>
#include <stdio.h>
#include <sstream>
#include <stdlib.h>
#include <iostream>

// ROS headers
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosbag/query.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

// OpenCV headers
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

// kinect2 bridge header
#include <kinect2_bridge/kinect2_definitions.h>

void readImage(const sensor_msgs::Image::ConstPtr msgImage, cv::Mat &image)
{
  // obtain image data and encoding from sensor msg
  cv_bridge::CvImageConstPtr pCvImage;
  pCvImage = cv_bridge::toCvShare(msgImage, msgImage->encoding);

  // copy data to the Mat image
  pCvImage->image.copyTo(image);
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

// help function
void help(const std::string &path)
{
  std::cout << path << " [options]" << std::endl
            << "  name: 'any string' equals to the file name" << std::endl
            << "  mode: 'qhd', 'hd', 'sd'" << std::endl;
}

int main(int argc, char **argv)
{
  // create ros node with a random name to avoid conflicts
  ros::init(argc, argv, "kinect2_processor", ros::init_options::AnonymousName);

  // check for failure
  if(!ros::ok())
  {
    return 0;
  }

  // ns = kinect2
  // selecting default topic names when the options are not provided
  std::string topicType = "sd";
  std::string ns = K2_DEFAULT_NS;
  std::string topicColor = K2_TOPIC_SD K2_TOPIC_IMAGE_COLOR K2_TOPIC_IMAGE_RECT;
  std::string topicDepth = K2_TOPIC_SD K2_TOPIC_IMAGE_DEPTH K2_TOPIC_IMAGE_RECT;

  std::string fileName = "kinect.bag";

  // parsing command line arguments
  for(size_t i = 1; i < (size_t)argc; ++i)
  {
    std::string param(argv[i]);

    // printing help information
    if(param == "-h" || param == "--help" || param == "-?" || param == "--?")
    {
      help(argv[0]);
      ros::shutdown();
      return 0;
    }

    // selecting quater hd topics
    else if(param == "qhd")
    {
      topicType = param;
      topicColor = K2_TOPIC_QHD K2_TOPIC_IMAGE_COLOR K2_TOPIC_IMAGE_RECT;
      topicDepth = K2_TOPIC_QHD K2_TOPIC_IMAGE_DEPTH K2_TOPIC_IMAGE_RECT;
    }

    // selecting hd topics
    else if(param == "hd")
    {
      topicType = param;
      topicColor = K2_TOPIC_HD K2_TOPIC_IMAGE_COLOR K2_TOPIC_IMAGE_RECT;
      topicDepth = K2_TOPIC_HD K2_TOPIC_IMAGE_DEPTH K2_TOPIC_IMAGE_RECT;
    }

    else if(param == "ir")
    {
      topicColor = K2_TOPIC_SD K2_TOPIC_IMAGE_IR K2_TOPIC_IMAGE_RECT;
      topicDepth = K2_TOPIC_SD K2_TOPIC_IMAGE_DEPTH K2_TOPIC_IMAGE_RECT;
    }

    // selecting depth topics
    else if(param == "sd")
    {
      topicType = param;
      topicColor = K2_TOPIC_SD K2_TOPIC_IMAGE_COLOR K2_TOPIC_IMAGE_RECT;
      topicDepth = K2_TOPIC_SD K2_TOPIC_IMAGE_DEPTH K2_TOPIC_IMAGE_RECT;
    }

    // other option can only be the fileName
    else
    {
      fileName = param;
    }
  }

  // initializing color and depth topic names
  topicColor = "/" + ns + topicColor;
  topicDepth = "/" + ns + topicDepth;
  std::string topicCameraInfo = topicDepth.substr(0, topicDepth.rfind('/')) + "/camera_info";

  // print the selected topic names
  std::cout << "topic color: " << topicColor << std::endl;
  std::cout << "topic depth: " << topicDepth << std::endl;
  std::cout << "topic camera info: " << topicCameraInfo << std::endl;

  // create a vector of topic names to query in rosbag
  std::vector<std::string> topics;
  topics.push_back(topicColor);
  topics.push_back(topicDepth);
  topics.push_back(topicCameraInfo);

  // create rosbag instance and open bag file
  rosbag::Bag bag;
  bag.open(fileName, rosbag::bagmode::Read);
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  // opencv variables
  cv::Mat color, depth, depthDisp;
  cv::namedWindow("Color",0);
  cv::namedWindow("Depth",0);

  // iterator variables
  int frame = 0;
  ros::Time time;
  std::string type;
  rosbag::View::iterator iter = view.begin();

  std::cout << "starting processor..." << std::endl;

  // looping through rosbag file
  while(iter != view.end())
  {
    time = (*iter).getTime();

    // process messages together as they are stored at same time instant
    for (int i = 0; i < topics.size(); i++)
    {
      rosbag::MessageInstance const m = *iter;

      // process as color or depth image
      type = m.getDataType();
      if (type == "sensor_msgs/Image")
      {
        sensor_msgs::Image::ConstPtr image = m.instantiate<sensor_msgs::Image>();
        if (image->encoding == "bgr8")
          readImage(image, color);
        else if (image->encoding == "16UC1")
          readImage(image, depth);
      }
      // process of camera info matrix
      else
        sensor_msgs::CameraInfo::ConstPtr cameraInfo = m.instantiate<sensor_msgs::CameraInfo>();

      ++iter;
    }

    // processing of color and depth images can be done here
    // camera info can also be used for computing point clouds

    dispDepth(depth, depthDisp, 12000.0f);

    cv::imshow("Color", color);
    cv::imshow("Depth", depthDisp);
    char c = cv::waitKey(30);

    if (c == 'q' || c == 27)
      break;

    std::cout << "Frame: " << frame << ", Time: " << time << std::endl;
    frame++;
  }

  bag.close();

  // clean shutdown
  ros::shutdown();
  return 0;
}
