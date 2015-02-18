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

#include <fstream>

#define __CL_ENABLE_EXCEPTIONS
#ifdef __APPLE__
#include <OpenCL/cl.hpp>
#else
#include <CL/cl.hpp>
#endif

#ifndef REG_OPENCL_FILE
#define REG_OPENCL_FILE ""
#endif

#include "depth_registration_opencl.h"

#define OUT_NAME(FUNCTION) "[DepthRegistrationOpenCL::" FUNCTION "] "

struct DepthRegistrationOpenCL::OCLData
{
  cl::Context context;
  cl::Device device;

  cl::Program program;
  cl::CommandQueue queue;

  cl::Kernel kernelSetZero;
  cl::Kernel kernelProject;
  cl::Kernel kernelCheckDepth;
  cl::Kernel kernelRemap;

  size_t sizeDepth;
  size_t sizeRegistered;
  size_t sizeIndex;
  size_t sizeImgZ;
  size_t sizeDists;
  size_t sizeSelDist;
  size_t sizeMap;

  cl::Buffer bufferDepth;
  cl::Buffer bufferScaled;
  cl::Buffer bufferRegistered;
  cl::Buffer bufferIndex;
  cl::Buffer bufferImgZ;
  cl::Buffer bufferDists;
  cl::Buffer bufferSelDist;
  cl::Buffer bufferMapX;
  cl::Buffer bufferMapY;
};

DepthRegistrationOpenCL::DepthRegistrationOpenCL()
  : DepthRegistration()
{
  data = new OCLData;
}

DepthRegistrationOpenCL::~DepthRegistrationOpenCL()
{
  delete data;
}

void getDevices(const std::vector<cl::Platform> &platforms, std::vector<cl::Device> &devices)
{
  devices.clear();
  for(size_t i = 0; i < platforms.size(); ++i)
  {
    const cl::Platform &platform = platforms[i];

    std::vector<cl::Device> devs;
    if(platform.getDevices(CL_DEVICE_TYPE_ALL, &devs) != CL_SUCCESS)
    {
      continue;
    }

    devices.insert(devices.end(), devs.begin(), devs.end());
  }
}

void listDevice(std::vector<cl::Device> &devices)
{
  std::cout << OUT_NAME("listDevice") " devices:" << std::endl;
  for(size_t i = 0; i < devices.size(); ++i)
  {
    cl::Device &dev = devices[i];
    std::string devName, devVendor, devType;
    size_t devTypeID;
    dev.getInfo(CL_DEVICE_NAME, &devName);
    dev.getInfo(CL_DEVICE_VENDOR, &devVendor);
    dev.getInfo(CL_DEVICE_TYPE, &devTypeID);

    switch(devTypeID)
    {
    case CL_DEVICE_TYPE_CPU:
      devType = "CPU";
      break;
    case CL_DEVICE_TYPE_GPU:
      devType = "GPU";
      break;
    case CL_DEVICE_TYPE_ACCELERATOR:
      devType = "ACCELERATOR";
      break;
    case CL_DEVICE_TYPE_CUSTOM:
      devType = "CUSTOM";
      break;
    default:
      devType = "UNKNOWN";
    }

    std::cout << "  " << i << ": " << devName << " (" << devType << ")[" << devVendor << ']' << std::endl;
  }
}

bool selectDevice(std::vector<cl::Device> &devices, cl::Device &device, const int deviceId = -1)
{
  if(deviceId != -1 && devices.size() > (size_t)deviceId)
  {
    device = devices[deviceId];
    return true;
  }

  bool selected = false;
  size_t selectedType = 0;

  for(size_t i = 0; i < devices.size(); ++i)
  {
    cl::Device &dev = devices[i];
    size_t devTypeID;
    dev.getInfo(CL_DEVICE_TYPE, &devTypeID);

    if(!selected || (selectedType != CL_DEVICE_TYPE_GPU && devTypeID == CL_DEVICE_TYPE_GPU))
    {
      selectedType = devTypeID;
      selected = true;
      device = dev;
    }
  }
  return selected;
}

bool DepthRegistrationOpenCL::init(const int deviceId)
{
  std::string sourceCode;
  if(!readProgram(sourceCode))
  {
    return false;
  }

  cl_int err = CL_SUCCESS;
  try
  {
    std::vector<cl::Platform> platforms;
    if(cl::Platform::get(&platforms) != CL_SUCCESS)
    {
      std::cerr << OUT_NAME("init") "error while getting opencl platforms." << std::endl;
      return false;
    }
    if(platforms.empty())
    {
      std::cerr << OUT_NAME("init") "no opencl platforms found." << std::endl;
      return false;
    }

    std::vector<cl::Device> devices;
    getDevices(platforms, devices);
    listDevice(devices);
    if(selectDevice(devices, data->device, deviceId))
    {
      std::string devName, devVendor, devType;
      size_t devTypeID;
      data->device.getInfo(CL_DEVICE_NAME, &devName);
      data->device.getInfo(CL_DEVICE_VENDOR, &devVendor);
      data->device.getInfo(CL_DEVICE_TYPE, &devTypeID);

      switch(devTypeID)
      {
      case CL_DEVICE_TYPE_CPU:
        devType = "CPU";
        break;
      case CL_DEVICE_TYPE_GPU:
        devType = "GPU";
        break;
      case CL_DEVICE_TYPE_ACCELERATOR:
        devType = "ACCELERATOR";
        break;
      case CL_DEVICE_TYPE_CUSTOM:
        devType = "CUSTOM";
        break;
      default:
        devType = "UNKNOWN";
      }
      std::cout << OUT_NAME("init") " selected device: " << devName << " (" << devType << ")[" << devVendor << ']' << std::endl;
    }
    else
    {
      std::cerr << OUT_NAME("init") "could not find any suitable device" << std::endl;
      return false;
    }

    data->context = cl::Context(data->device);

    std::string options;
    generateOptions(options);

    cl::Program::Sources source(1, std::make_pair(sourceCode.c_str(), sourceCode.length()));
    data->program = cl::Program(data->context, source);
    data->program.build(options.c_str());

    data->queue = cl::CommandQueue(data->context, data->device, 0, &err);

    data->sizeDepth = sizeDepth.height * sizeDepth.width * sizeof(uint16_t);
    data->sizeRegistered = sizeRegistered.height * sizeRegistered.width * sizeof(uint16_t);
    data->sizeIndex = sizeRegistered.height * sizeRegistered.width * sizeof(cl_int4);
    data->sizeImgZ = sizeRegistered.height * sizeRegistered.width * sizeof(uint16_t);
    data->sizeDists = sizeRegistered.height * sizeRegistered.width * sizeof(cl_float4);
    data->sizeSelDist = sizeRegistered.height * sizeRegistered.width * sizeof(float);
    data->sizeMap = sizeRegistered.height * sizeRegistered.width * sizeof(float);

    data->bufferDepth = cl::Buffer(data->context, CL_READ_ONLY_CACHE, data->sizeDepth, NULL, &err);
    data->bufferScaled = cl::Buffer(data->context, CL_READ_WRITE_CACHE, data->sizeRegistered, NULL, &err);
    data->bufferRegistered = cl::Buffer(data->context, CL_READ_WRITE_CACHE, data->sizeRegistered, NULL, &err);
    data->bufferIndex = cl::Buffer(data->context, CL_READ_WRITE_CACHE, data->sizeIndex, NULL, &err);
    data->bufferImgZ = cl::Buffer(data->context, CL_READ_WRITE_CACHE, data->sizeImgZ, NULL, &err);
    data->bufferDists = cl::Buffer(data->context, CL_READ_WRITE_CACHE, data->sizeDists, NULL, &err);
    data->bufferSelDist = cl::Buffer(data->context, CL_READ_WRITE_CACHE, data->sizeSelDist, NULL, &err);
    data->bufferMapX = cl::Buffer(data->context, CL_READ_ONLY_CACHE, data->sizeMap, NULL, &err);
    data->bufferMapY = cl::Buffer(data->context, CL_READ_ONLY_CACHE, data->sizeMap, NULL, &err);

    data->kernelSetZero = cl::Kernel(data->program, "setZero", &err);
    data->kernelSetZero.setArg(0, data->bufferRegistered);
    data->kernelSetZero.setArg(1, data->bufferSelDist);

    data->kernelProject = cl::Kernel(data->program, "project", &err);
    data->kernelProject.setArg(0, data->bufferScaled);
    data->kernelProject.setArg(1, data->bufferIndex);
    data->kernelProject.setArg(2, data->bufferImgZ);
    data->kernelProject.setArg(3, data->bufferDists);
    data->kernelProject.setArg(4, data->bufferSelDist);
    data->kernelProject.setArg(5, data->bufferRegistered);

    data->kernelCheckDepth = cl::Kernel(data->program, "checkDepth", &err);
    data->kernelCheckDepth.setArg(0, data->bufferIndex);
    data->kernelCheckDepth.setArg(1, data->bufferImgZ);
    data->kernelCheckDepth.setArg(2, data->bufferDists);
    data->kernelCheckDepth.setArg(3, data->bufferSelDist);
    data->kernelCheckDepth.setArg(4, data->bufferRegistered);

    data->kernelRemap = cl::Kernel(data->program, "remapDepth", &err);
    data->kernelRemap.setArg(0, data->bufferDepth);
    data->kernelRemap.setArg(1, data->bufferScaled);
    data->kernelRemap.setArg(2, data->bufferMapX);
    data->kernelRemap.setArg(3, data->bufferMapY);

    data->queue.enqueueWriteBuffer(data->bufferMapX, CL_TRUE, 0, data->sizeMap, mapX.data);
    data->queue.enqueueWriteBuffer(data->bufferMapY, CL_TRUE, 0, data->sizeMap, mapY.data);
  }
  catch(cl::Error err)
  {
    std::cerr << OUT_NAME("init") "ERROR: " << err.what() << "(" << err.err() << ")" << std::endl;

    if(err.err() == CL_BUILD_PROGRAM_FAILURE)
    {
      std::cout << OUT_NAME("init") "Build Status: " << data->program.getBuildInfo<CL_PROGRAM_BUILD_STATUS>(data->device) << std::endl;
      std::cout << OUT_NAME("init") "Build Options:\t" << data->program.getBuildInfo<CL_PROGRAM_BUILD_OPTIONS>(data->device) << std::endl;
      std::cout << OUT_NAME("init") "Build Log:\t " << data->program.getBuildInfo<CL_PROGRAM_BUILD_LOG>(data->device) << std::endl;
    }

    return false;
  }

  return true;
}

void DepthRegistrationOpenCL::registerDepth(const cv::Mat &depth, cv::Mat &registered)
{
  if(registered.empty() || registered.rows != sizeRegistered.height || registered.cols != sizeRegistered.width || registered.type() != CV_16U)
  {
    registered = cv::Mat(sizeRegistered, CV_16U);
  }

  try
  {
    cl::Event eventKernel, eventZero;
    cl::NDRange range(sizeRegistered.height * sizeRegistered.width);

    data->queue.enqueueWriteBuffer(data->bufferDepth, CL_TRUE, 0, data->sizeDepth, depth.data);
    data->queue.enqueueNDRangeKernel(data->kernelSetZero, cl::NullRange, range, cl::NullRange, NULL, &eventZero);

    data->queue.enqueueNDRangeKernel(data->kernelRemap, cl::NullRange, range, cl::NullRange, NULL, &eventKernel);
    eventKernel.wait();
    eventZero.wait();

    data->queue.enqueueNDRangeKernel(data->kernelProject, cl::NullRange, range, cl::NullRange, NULL, &eventKernel);
    eventKernel.wait();

    data->queue.enqueueNDRangeKernel(data->kernelCheckDepth, cl::NullRange, range, cl::NullRange, NULL, &eventKernel);
    eventKernel.wait();

    data->queue.enqueueNDRangeKernel(data->kernelCheckDepth, cl::NullRange, range, cl::NullRange, NULL, &eventKernel);
    eventKernel.wait();

    data->queue.enqueueReadBuffer(data->bufferRegistered, CL_TRUE, 0, data->sizeRegistered, registered.data);
  }
  catch(cl::Error err)
  {
    std::cerr << OUT_NAME("registerDepth") "ERROR: " << err.what() << "(" << err.err() << ")" << std::endl;
    return;
  }
}

void DepthRegistrationOpenCL::generateOptions(std::string &options) const
{
  std::ostringstream oss;
  oss.precision(16);
  oss << std::scientific;

  // Rotation
  oss << " -D r00=" << rotation.at<double>(0, 0) << "f";
  oss << " -D r01=" << rotation.at<double>(0, 1) << "f";
  oss << " -D r02=" << rotation.at<double>(0, 2) << "f";
  oss << " -D r10=" << rotation.at<double>(1, 0) << "f";
  oss << " -D r11=" << rotation.at<double>(1, 1) << "f";
  oss << " -D r12=" << rotation.at<double>(1, 2) << "f";
  oss << " -D r20=" << rotation.at<double>(2, 0) << "f";
  oss << " -D r21=" << rotation.at<double>(2, 1) << "f";
  oss << " -D r22=" << rotation.at<double>(2, 2) << "f";

  // Translation
  oss << " -D tx=" << translation.at<double>(0, 0) << "f";
  oss << " -D ty=" << translation.at<double>(1, 0) << "f";
  oss << " -D tz=" << translation.at<double>(2, 0) << "f";

  // Camera parameter upscaled depth
  oss << " -D fxR=" << cameraMatrixRegistered.at<double>(0, 0) << "f";
  oss << " -D fyR=" << cameraMatrixRegistered.at<double>(1, 1) << "f";
  oss << " -D cxR=" << cameraMatrixRegistered.at<double>(0, 2) << "f";
  oss << " -D cyR=" << cameraMatrixRegistered.at<double>(1, 2) << "f";
  oss << " -D fxRInv=" << (1.0 / cameraMatrixRegistered.at<double>(0, 0)) << "f";
  oss << " -D fyRInv=" << (1.0 / cameraMatrixRegistered.at<double>(1, 1)) << "f";

  // Clipping distances
  oss << " -D zNear=" << (uint16_t)(zNear * 1000);
  oss << " -D zFar=" << (uint16_t)(zFar * 1000);

  // Size registered image
  oss << " -D heightR=" << sizeRegistered.height;
  oss << " -D widthR=" << sizeRegistered.width;

  // Size depth image
  oss << " -D heightD=" << sizeDepth.height;
  oss << " -D widthD=" << sizeDepth.width;

  options = oss.str();
}

bool DepthRegistrationOpenCL::readProgram(std::string &source) const
{
  std::ifstream file(REG_OPENCL_FILE);

  if(!file.is_open())
  {
    return false;
  }

  source = std::string((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
  file.close();

  //std::cout << OUT_NAME("readProgram") "source:" << std::endl source << std::endl;
  return true;
}
