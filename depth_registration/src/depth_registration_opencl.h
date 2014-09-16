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
#ifndef __DEPTH_REGISTRATION_OPENCL_H__
#define __DEPTH_REGISTRATION_OPENCL_H__

#include <depth_registration.h>

class DepthRegistrationOpenCL : public DepthRegistration
{
private:
  struct OCLData;

  const cv::Size sizeColor, sizeDepth, sizeRaw;

  const uint16_t zNear;
  const uint16_t zFar;
  const float zDist;

  OCLData *data;
  cv::Mat map1, map2;

public:
  DepthRegistrationOpenCL(const cv::Size &color, const cv::Size &depth, const cv::Size &raw, const float zNear = 0.5f, const float zFar = 12.0f, const float zDist = 0.015f);

  ~DepthRegistrationOpenCL();

  bool init();

  void remapDepth(const cv::Mat &in, cv::Mat &out) const;

  void registerDepth(const cv::Mat &depth, cv::Mat &registered);

  void depthToRGBResolution(const cv::Mat &registered, cv::Mat &upscaled);

private:
  void generateOptions(std::string &options) const;

  bool readProgram(std::string &source) const;
};

#endif //__DEPTH_REGISTRATION_OPENCL_H__
