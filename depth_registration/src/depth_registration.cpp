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

#include <depth_registration.h>

#ifdef DEPTH_REG_CPU
#include "depth_registration_cpu.h"
#endif

#ifdef DEPTH_REG_OPENCL
#include "depth_registration_opencl.h"
#endif

#define OUT_NAME(FUNCTION) "[DepthRegistration::" FUNCTION "] "

DepthRegistration::DepthRegistration()
{
}

DepthRegistration::~DepthRegistration()
{
}

bool DepthRegistration::init(const cv::Mat &cameraMatrixRegistered, const cv::Size &sizeRegistered, const cv::Mat &cameraMatrixDepth, const cv::Size &sizeDepth,
                             const cv::Mat &distortionDepth, const cv::Mat &rotation, const cv::Mat &translation,
                             const float zNear, const float zFar, const int deviceId)
{
  this->cameraMatrixRegistered = cameraMatrixRegistered;
  this->cameraMatrixDepth = cameraMatrixDepth;
  this->rotation = rotation;
  this->translation = translation;
  this->sizeRegistered = sizeRegistered;
  this->sizeDepth = sizeDepth;
  this->zNear = zNear;
  this->zFar = zFar;

  cv::initUndistortRectifyMap(cameraMatrixDepth, distortionDepth, cv::Mat(), cameraMatrixRegistered, sizeRegistered, CV_32FC1, mapX, mapY);

  return init(deviceId);
}

DepthRegistration *DepthRegistration::New(Method method)
{
  if(method == DEFAULT)
  {
#ifdef DEPTH_REG_OPENCL
    method = OPENCL;
#elif defined DEPTH_REG_CPU
    method = CPU;
#endif
  }

  switch(method)
  {
  case DEFAULT:
    std::cerr << OUT_NAME("New") "No default registration method available!" << std::endl;
    break;
  case CPU:
#ifdef DEPTH_REG_CPU
    std::cout << OUT_NAME("New") "Using CPU registration method!" << std::endl;
    return new DepthRegistrationCPU();
#else
    std::cerr << OUT_NAME("New") "CPU registration method not available!" << std::endl;
    break;
#endif
  case OPENCL:
#ifdef DEPTH_REG_OPENCL
    std::cout << OUT_NAME("New") "Using OpenCL registration method!" << std::endl;
    return new DepthRegistrationOpenCL();
#else
    std::cerr << OUT_NAME("New") "OpenCL registration method not available!" << std::endl;
    break;
#endif
  }
  return NULL;
}
