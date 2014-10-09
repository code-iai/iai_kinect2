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
  cl::Kernel kernelRender;
  cl::Kernel kernelRemap;

  size_t sizeDepth;
  size_t sizeIndex;
  size_t sizeImgZ;
  size_t sizeDists;
  size_t sizeSelDist;
  size_t sizeRendered;
  size_t sizeRaw;
  size_t sizeMap;

  cl::Buffer bufferDepth;
  cl::Buffer bufferIndex;
  cl::Buffer bufferImgZ;
  cl::Buffer bufferDists;
  cl::Buffer bufferSelDist;
  cl::Buffer bufferRendered;
  cl::Buffer bufferRemapIn;
  cl::Buffer bufferRemapOut;
  cl::Buffer bufferMapX;
  cl::Buffer bufferMapY;
};

DepthRegistrationOpenCL::DepthRegistrationOpenCL(const cv::Size &color, const cv::Size &depth, const cv::Size &raw, const float zNear, const float zFar, const float zDist)
  : DepthRegistration(), sizeColor(color), sizeDepth(depth), sizeRaw(raw), zNear(zNear * 1000), zFar(zFar * 1000), zDist(zDist)
{
  data = new OCLData;
}

DepthRegistrationOpenCL::~DepthRegistrationOpenCL()
{
  delete data;
}

bool selectDevice(const std::vector<cl::Platform> &platforms, const int type, cl::Device &device)
{
  bool selected = false;

  for(size_t i = 0; i < platforms.size(); ++i)
  {
    const cl::Platform &platform = platforms[i];
    std::string platformName, platformVendor;
    platform.getInfo(CL_PLATFORM_NAME, &platformName);
    platform.getInfo(CL_PLATFORM_VENDOR, &platformVendor);

    std::cout << OUT_NAME("selectDevice") "found platform: " << platformName << " vendor: " << platformVendor << std::endl;

    std::vector<cl::Device> devices;
    if(platform.getDevices(type, &devices) != CL_SUCCESS)
    {
      std::cerr << OUT_NAME("selectDevice") "error while getting opencl devices." << std::endl;
      return false;
    }

    for(size_t j = 0; j < devices.size(); ++j)
    {
      cl::Device &dev = devices[i];
      std::string devName, devVendor;
      dev.getInfo(CL_DEVICE_NAME, &devName);
      dev.getInfo(CL_DEVICE_VENDOR, &devVendor);

      std::cout << OUT_NAME("selectDevice") "found device: " << devName << " vendor: " << devVendor << std::endl;

      if(!selected)
      {
        selected = true;
        device = dev;
      }
    }
  }

  return selected;
}

bool DepthRegistrationOpenCL::init()
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

    if(!selectDevice(platforms, CL_DEVICE_TYPE_GPU, data->device))
    {
      std::cout << OUT_NAME("init") "could not find any GPU device. trying CPU devices" << std::endl;

      if(!selectDevice(platforms, CL_DEVICE_TYPE_CPU, data->device))
      {
        std::cerr << OUT_NAME("init") "could not find any suitable device" << std::endl;
        return false;
      }
    }
    data->context = cl::Context(data->device);

    std::string options;
    generateOptions(options);

    cl::Program::Sources source(1, std::make_pair(sourceCode.c_str(), sourceCode.length()));
    data->program = cl::Program(data->context, source);
    data->program.build(options.c_str());

    data->queue = cl::CommandQueue(data->context, data->device, 0, &err);

    data->sizeDepth = sizeDepth.height * sizeDepth.width * sizeof(uint16_t);
    data->sizeIndex = sizeDepth.height * sizeDepth.width * sizeof(cl_int4);
    data->sizeImgZ = sizeDepth.height * sizeDepth.width * sizeof(uint16_t);
    data->sizeDists = sizeDepth.height * sizeDepth.width * sizeof(cl_float4);
    data->sizeSelDist = sizeDepth.height * sizeDepth.width * sizeof(float);
    data->sizeRendered = sizeDepth.height * sizeDepth.width * sizeof(uint16_t);
    data->sizeRaw = sizeRaw.height * sizeRaw.width * sizeof(uint16_t);
    data->sizeMap = sizeDepth.height * sizeDepth.width * sizeof(float);

    data->bufferDepth = cl::Buffer(data->context, CL_READ_ONLY_CACHE, data->sizeDepth, NULL, &err);
    data->bufferIndex = cl::Buffer(data->context, CL_READ_WRITE_CACHE, data->sizeIndex, NULL, &err);
    data->bufferImgZ = cl::Buffer(data->context, CL_READ_WRITE_CACHE, data->sizeImgZ, NULL, &err);
    data->bufferDists = cl::Buffer(data->context, CL_READ_WRITE_CACHE, data->sizeDists, NULL, &err);
    data->bufferSelDist = cl::Buffer(data->context, CL_READ_WRITE_CACHE, data->sizeSelDist, NULL, &err);
    data->bufferRendered = cl::Buffer(data->context, CL_READ_WRITE_CACHE, data->sizeRendered, NULL, &err);
    data->bufferRemapIn = cl::Buffer(data->context, CL_READ_ONLY_CACHE, data->sizeRaw, NULL, &err);
    data->bufferRemapOut = cl::Buffer(data->context, CL_READ_WRITE_CACHE, data->sizeDepth, NULL, &err);
    data->bufferMapX = cl::Buffer(data->context, CL_READ_ONLY_CACHE, data->sizeMap, NULL, &err);
    data->bufferMapY = cl::Buffer(data->context, CL_READ_ONLY_CACHE, data->sizeMap, NULL, &err);

    data->kernelSetZero = cl::Kernel(data->program, "setZero", &err);
    data->kernelSetZero.setArg(0, data->bufferRendered);
    data->kernelSetZero.setArg(1, data->bufferSelDist);

    data->kernelProject = cl::Kernel(data->program, "project", &err);
    data->kernelProject.setArg(0, data->bufferDepth);
    data->kernelProject.setArg(1, data->bufferIndex);
    data->kernelProject.setArg(2, data->bufferImgZ);
    data->kernelProject.setArg(3, data->bufferDists);
    data->kernelProject.setArg(4, data->bufferSelDist);
    data->kernelProject.setArg(5, data->bufferRendered);

    data->kernelRender = cl::Kernel(data->program, "render", &err);
    data->kernelRender.setArg(0, data->bufferIndex);
    data->kernelRender.setArg(1, data->bufferImgZ);
    data->kernelRender.setArg(2, data->bufferDists);
    data->kernelRender.setArg(3, data->bufferSelDist);
    data->kernelRender.setArg(4, data->bufferRendered);

    data->kernelRemap = cl::Kernel(data->program, "remapDepth", &err);
    data->kernelRemap.setArg(0, data->bufferRemapIn);
    data->kernelRemap.setArg(1, data->bufferRemapOut);
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

  cv::initUndistortRectifyMap(cameraMatrixDepth, cv::Mat(), cv::Mat(), cameraMatrixColor, sizeColor, CV_16SC2, map1, map2);
  return true;
}

void DepthRegistrationOpenCL::remapDepth(const cv::Mat &in, cv::Mat &out) const
{
  if(out.empty() || out.rows != sizeDepth.height || out.cols != sizeDepth.width || out.type() != CV_16U)
  {
    out = cv::Mat(sizeDepth, CV_16U);
  }

  try
  {
    cl::Event eventKernel;
    cl::NDRange range(sizeDepth.height * sizeDepth.width);

    data->queue.enqueueWriteBuffer(data->bufferRemapIn, CL_TRUE, 0, data->sizeRaw, in.data);

    data->queue.enqueueNDRangeKernel(data->kernelRemap, cl::NullRange, range, cl::NullRange, NULL, &eventKernel);
    eventKernel.wait();

    data->queue.enqueueReadBuffer(data->bufferRemapOut, CL_TRUE, 0, data->sizeDepth, out.data);
  }
  catch(cl::Error err)
  {
    std::cerr << OUT_NAME("remapDepth") "ERROR: " << err.what() << "(" << err.err() << ")" << std::endl;
    return;
  }
}

void DepthRegistrationOpenCL::registerDepth(const cv::Mat &depth, cv::Mat &registered)
{
  if(registered.empty() || registered.rows != sizeDepth.height || registered.cols != sizeDepth.width || registered.type() != CV_16U)
  {
    registered = cv::Mat(sizeDepth, CV_16U);
  }

  try
  {
    cl::Event eventBuffer, eventKernel;
    cl::NDRange range(sizeDepth.height * sizeDepth.width);

    data->queue.enqueueNDRangeKernel(data->kernelSetZero, cl::NullRange, range, cl::NullRange, NULL, &eventKernel);
    data->queue.enqueueWriteBuffer(data->bufferDepth, CL_FALSE, 0, data->sizeDepth, depth.data, NULL, &eventBuffer);
    eventBuffer.wait();
    eventKernel.wait();

    data->queue.enqueueNDRangeKernel(data->kernelProject, cl::NullRange, range, cl::NullRange, NULL, &eventKernel);
    eventKernel.wait();

    data->queue.enqueueNDRangeKernel(data->kernelRender, cl::NullRange, range, cl::NullRange, NULL, &eventKernel);
    eventKernel.wait();
    //data->queue.enqueueNDRangeKernel(data->kernelRender, cl::NullRange, range, cl::NullRange, NULL, &eventKernel);
    //eventKernel.wait();
    //data->queue.enqueueNDRangeKernel(data->kernelRender, cl::NullRange, range, cl::NullRange, NULL, &eventKernel);
    //eventKernel.wait();

    data->queue.enqueueReadBuffer(data->bufferRendered, CL_FALSE, 0, data->sizeRendered, registered.data, NULL, &eventBuffer);
    eventBuffer.wait();
  }
  catch(cl::Error err)
  {
    std::cerr << OUT_NAME("registerDepth") "ERROR: " << err.what() << "(" << err.err() << ")" << std::endl;
    return;
  }
}

void DepthRegistrationOpenCL::depthToRGBResolution(const cv::Mat &registered, cv::Mat &upscaled)
{
  cv::remap(registered, upscaled, map1, map2, cv::INTER_NEAREST);
  //cv::medianBlur(upscaled, upscaled, 5);
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
  oss << " -D fxD=" << cameraMatrixDepth.at<double>(0, 0) << "f";
  oss << " -D fyD=" << cameraMatrixDepth.at<double>(1, 1) << "f";
  oss << " -D cxD=" << cameraMatrixDepth.at<double>(0, 2) << "f";
  oss << " -D cyD=" << cameraMatrixDepth.at<double>(1, 2) << "f";
  oss << " -D fxDInv=" << (1.0 / cameraMatrixDepth.at<double>(0, 0)) << "f";
  oss << " -D fyDInv=" << (1.0 / cameraMatrixDepth.at<double>(1, 1)) << "f";

  // Camera parameter color
  oss << " -D fxC=" << cameraMatrixColor.at<double>(0, 0) << "f";
  oss << " -D fyC=" << cameraMatrixColor.at<double>(1, 1) << "f";
  oss << " -D cxC=" << cameraMatrixColor.at<double>(0, 2) << "f";
  oss << " -D cyC=" << cameraMatrixColor.at<double>(1, 2) << "f";

  // Clipping distances
  oss << " -D zNear=" << zNear;
  oss << " -D zFar=" << zFar;
  oss << " -D zDist=" << zDist << "f";

  // Size color image
  oss << " -D heightC=" << sizeColor.height;
  oss << " -D widthC=" << sizeColor.width;

  // Size depth image
  oss << " -D heightD=" << sizeDepth.height;
  oss << " -D widthD=" << sizeDepth.width;

  // Size raw depth image
  oss << " -D heightR=" << sizeRaw.height;
  oss << " -D widthR=" << sizeRaw.width;

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
