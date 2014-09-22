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
#ifndef __KINECT2_DEFINITIONS_H__
#define __KINECT2_DEFINITIONS_H__

#define K2_TOPIC_BASE          "kinect2_head/"
#define K2_TF_RGB_FRAME        "head_mount_kinect2_rgb_optical_frame"
#define K2_TF_IR_FRAME         "head_mount_kinect2_ir_optical_frame"

#define K2_TOPIC_IMAGE_IR      K2_TOPIC_BASE "ir/"
#define K2_TOPIC_RECT_IR       K2_TOPIC_BASE "ir_rect/"

#define K2_TOPIC_IMAGE_COLOR   K2_TOPIC_BASE "rgb/"
#define K2_TOPIC_RECT_COLOR    K2_TOPIC_BASE "rgb_rect/"
#define K2_TOPIC_REG_COLOR     K2_TOPIC_BASE "rgb_lowres/"

#define K2_TOPIC_IMAGE_MONO    K2_TOPIC_BASE "mono/"
#define K2_TOPIC_RECT_MONO     K2_TOPIC_BASE "mono_rect/"
#define K2_TOPIC_REG_MONO      K2_TOPIC_BASE "mono_lowres/"

#define K2_TOPIC_IMAGE_DEPTH   K2_TOPIC_BASE "depth/"
#define K2_TOPIC_RECT_DEPTH    K2_TOPIC_BASE "depth_rect/"
#define K2_TOPIC_REG_DEPTH     K2_TOPIC_BASE "depth_lowres/"
#define K2_TOPIC_HIRES_DEPTH   K2_TOPIC_BASE "depth_highres/"

#define K2_TOPIC_RAW           "image"
#define K2_TOPIC_IMAGE         "image"
#define K2_TOPIC_COMPRESSED    "/compressed"
#define K2_TOPIC_COMP_DEPTH    "/compressedDepth"
#define K2_TOPIC_INFO          "camera_info"

#define K2_CALIB_COLOR         "/calib_color.yaml"
#define K2_CALIB_IR            "/calib_ir.yaml"
#define K2_CALIB_POSE          "/calib_pose.yaml"

#define K2_CALIB_CAMERA_MATRIX "cameraMatrix"
#define K2_CALIB_DISTORTION    "distortionCoefficients"
#define K2_CALIB_ROTATION      "rotation"
#define K2_CALIB_PROJECTION    "projection"
#define K2_CALIB_TRANSLATION   "translation"
#define K2_CALIB_ESSENTIAL     "essential"
#define K2_CALIB_FUNDAMENTAL   "fundamental"

#endif //__KINECT2_DEFINITIONS_H__
