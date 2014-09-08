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

#pragma once
#ifndef __DEPTH_REGISTRATION_H__
#define __DEPTH_REGISTRATION_H__

#include <vector>

#include <opencv2/opencv.hpp>

class DepthRegistration
{
public:
  enum Method
  {
    DEFAULT = 0,
    CPU,
    OPENCL
  };

protected:
  cv::Mat cameraMatrixColor, cameraMatrixDepth, rotation, translation, mapX, mapY;

  DepthRegistration();

  virtual bool init() = 0;

public:
  virtual ~DepthRegistration();

  bool init(const cv::Mat &cameraMatrixColor, const cv::Mat &cameraMatrixDepth, const cv::Mat &rotation, const cv::Mat &translation, const cv::Mat &mapX, const cv::Mat &mapY);

  virtual void remapDepth(const cv::Mat &in, cv::Mat &out) const = 0;

  virtual void registerDepth(const cv::Mat &depth, cv::Mat &registered) = 0;

  virtual void depthToRGBResolution(const cv::Mat &registered, cv::Mat &upscaled) = 0;

  static DepthRegistration *New(const cv::Size &color, const cv::Size &depth, const cv::Size &raw, const float zNear = 0.5f, const float zFar = 12.0f, const float zDist = 0.015f, Method method = DEFAULT);
};

#endif //__DEPTH_REGISTRATION_H__
