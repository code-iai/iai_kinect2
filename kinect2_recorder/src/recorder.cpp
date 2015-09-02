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
#include <mutex>
#include <thread>
#include <chrono>
#include <string>
#include <vector>
#include <stdio.h>
#include <sstream>
#include <stdlib.h>
#include <iostream>

// ROS headers
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <ros/spinner.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

// ROS messaging filters
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

// kinect2 bridge header
#include <kinect2_bridge/kinect2_definitions.h>

class Recorder
{
private:
  bool useExact, useCompressed;

  // important variables to decide functionality
  const std::string fileName;
  const std::string topicColor, topicDepth;
  std::string topicCameraInfoColor, topicCameraInfoDepth;

  // flags for logic
  size_t frame;
  const size_t queueSize;

  // variable for the rosbag
  rosbag::Bag bag;

  // synchronization parameters
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ExactSyncPolicy;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ApproximateSyncPolicy;

  // ros node parameters
  ros::NodeHandle nh;

  // AsyncSpinner is used to synchronize multiple threads under the ros framework
  ros::AsyncSpinner spinner;

  image_transport::ImageTransport it;
  image_transport::SubscriberFilter *subImageColor, *subImageDepth;
  message_filters::Subscriber<sensor_msgs::CameraInfo> *subCameraInfoColor, *subCameraInfoDepth;

  // message filters
  message_filters::Synchronizer<ExactSyncPolicy> *syncExact;
  message_filters::Synchronizer<ApproximateSyncPolicy> *syncApproximate;

public:
  // class constructor
  // takes 3 input arguments topic color, topic depth and file name
  Recorder(const std::string &topicColor, const std::string &topicDepth, const std::string fileName, const bool useExact, const bool useCompressed)
    : topicColor(topicColor), topicDepth(topicDepth), fileName(fileName), useExact(useExact), useCompressed(useCompressed),
    queueSize(5), nh("~"), spinner(0), it(nh)
  {
  }

  ~Recorder()
  {
  }

  // run function
  void run()
  {
    // start the kinect depth sensor
    start();

    std::chrono::milliseconds duration(1);
    while(1)
    {
      if(!ros::ok())
      {
        break;
      }
      std::this_thread::sleep_for(duration);
    }

    // stop the kinect
    stop();
  }

private:
  // start function
  void start()
  {
    // get the camera info ros topics for color and depth
    topicCameraInfoColor = topicColor.substr(0, topicColor.rfind('/')) + "/camera_info";
    topicCameraInfoDepth = topicDepth.substr(0, topicDepth.rfind('/')) + "/camera_info";

    // TransportHints stores the transport settings for the image topic subscriber
    // here we are giving the setting of raw or compressed using the useCompressed variable
    image_transport::TransportHints hints(useCompressed ? "compressed" : "raw");

    // SubscriberFilters are used to subscribe to the kinect image topics
    subImageColor = new image_transport::SubscriberFilter(it, topicColor, queueSize, hints);
    subImageDepth = new image_transport::SubscriberFilter(it, topicDepth, queueSize, hints);

    // message filters are used to subscribe to the camera info topics
    subCameraInfoColor = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh, topicCameraInfoColor, queueSize);
    subCameraInfoDepth = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh, topicCameraInfoDepth, queueSize);

    if(useExact)
    {
      syncExact = new message_filters::Synchronizer<ExactSyncPolicy>(ExactSyncPolicy(queueSize), *subImageColor, *subImageDepth, *subCameraInfoColor, *subCameraInfoDepth);
      syncExact->registerCallback(boost::bind(&Recorder::callback, this, _1, _2, _3, _4));
    }
    else
    {
      syncApproximate = new message_filters::Synchronizer<ApproximateSyncPolicy>(ApproximateSyncPolicy(queueSize), *subImageColor, *subImageDepth, *subCameraInfoColor, *subCameraInfoDepth);
      syncApproximate->registerCallback(boost::bind(&Recorder::callback, this, _1, _2, _3, _4));
    }

    // open a new bag file
    bag.open(fileName, rosbag::bagmode::Write);
    std::cout << "Opening ROS bag" << std::endl;

    // start the recording
    spinner.start();
  }

  // stop function to have clean shutdown
  void stop()
  {
    // stop the spinner
    spinner.stop();

    // close rosbag
    bag.close();
    std::cout << "Saving ROS Bag" << std::endl;

    // clean up all variables
    delete syncExact;

    delete subImageColor;
    delete subImageDepth;
    delete subCameraInfoColor;
    delete subCameraInfoDepth;
  }

  // message filter callback function, all processing is done here
  void callback(const sensor_msgs::Image::ConstPtr imageColor, const sensor_msgs::Image::ConstPtr imageDepth,
                const sensor_msgs::CameraInfo::ConstPtr cameraInfoColor, const sensor_msgs::CameraInfo::ConstPtr cameraInfoDepth)
  {
    ros::Time time = ros::Time::now();

    // write point cloud message and camera info to the ros bag file
    bag.write(topicColor, time, *imageColor);
    bag.write(topicDepth, time, *imageDepth);
    bag.write(topicCameraInfoColor, time, *cameraInfoColor);
  }
};

void help(const std::string &path)
{
  std::cout << path << " [options]" << std::endl
            << "  name: 'any string' equals to the file name" << std::endl
            << "  mode: 'qhd', 'hd', 'sd' or 'ir'" << std::endl
            << "  options:" << std::endl
            << "    'compressed' use compressed instead of raw topics" << std::endl
            << "    'approx' use approximate time synchronization" << std::endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "kinect2_recorder", ros::init_options::AnonymousName);

  if(!ros::ok())
  {
    return 0;
  }

  std::string ns = K2_DEFAULT_NS;
  std::string fileName = "kinect.bag";
  std::string topicColor = K2_TOPIC_QHD K2_TOPIC_IMAGE_COLOR K2_TOPIC_IMAGE_RECT;
  std::string topicDepth = K2_TOPIC_QHD K2_TOPIC_IMAGE_DEPTH K2_TOPIC_IMAGE_RECT;

  bool useExact = true;
  bool useCompressed = false;

  for(size_t i = 1; i < (size_t)argc; ++i)
  {
    std::string param(argv[i]);

    if(param == "-h" || param == "--help" || param == "-?" || param == "--?")
    {
      help(argv[0]);
      ros::shutdown();
      return 0;
    }
    else if(param == "qhd")
    {
      topicColor = K2_TOPIC_QHD K2_TOPIC_IMAGE_COLOR K2_TOPIC_IMAGE_RECT;
      topicDepth = K2_TOPIC_QHD K2_TOPIC_IMAGE_DEPTH K2_TOPIC_IMAGE_RECT;
    }
    else if(param == "hd")
    {
      topicColor = K2_TOPIC_HD K2_TOPIC_IMAGE_COLOR K2_TOPIC_IMAGE_RECT;
      topicDepth = K2_TOPIC_HD K2_TOPIC_IMAGE_DEPTH K2_TOPIC_IMAGE_RECT;
    }
    else if(param == "ir")
    {
      topicColor = K2_TOPIC_SD K2_TOPIC_IMAGE_IR K2_TOPIC_IMAGE_RECT;
      topicDepth = K2_TOPIC_SD K2_TOPIC_IMAGE_DEPTH K2_TOPIC_IMAGE_RECT;
    }
    else if(param == "sd")
    {
      topicColor = K2_TOPIC_SD K2_TOPIC_IMAGE_COLOR K2_TOPIC_IMAGE_RECT;
      topicDepth = K2_TOPIC_SD K2_TOPIC_IMAGE_DEPTH K2_TOPIC_IMAGE_RECT;
    }
    else if(param == "approx")
    {
      useExact = false;
    }

    else if(param == "compressed")
    {
      useCompressed = true;
    }
    else
    {
      fileName = param;
    }
  }

  topicColor = "/" + ns + topicColor;
  topicDepth = "/" + ns + topicDepth;
  std::cout << "topic color: " << topicColor << std::endl;
  std::cout << "topic depth: " << topicDepth << std::endl;

  Recorder recorder(topicColor, topicDepth, fileName, useExact, useCompressed);

  std::cout << "starting recorder..." << std::endl;
  recorder.run();

  ros::shutdown();
  return 0;
}
