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
#ifndef __CAMERA_CALIBRATION_DEFINITIONS_H__
#define __CAMERA_CALIBRATION_DEFINITIONS_H__

#define CALIB_FILE_EXT      ".png"
#define CALIB_FILE_COLOR    "_color" CALIB_FILE_EXT
#define CALIB_FILE_IR       "_ir" CALIB_FILE_EXT
#define CALIB_FILE_IR_GREY  "_grey_ir" CALIB_FILE_EXT

#define CALIB_POINTS_COLOR  "_color_points.yaml"
#define CALIB_POINTS_IR     "_ir_points.yaml"

#define CALIB_SYNC          "_sync"
#define CALIB_SYNC_COLOR    CALIB_SYNC CALIB_FILE_COLOR
#define CALIB_SYNC_IR       CALIB_SYNC CALIB_FILE_IR
#define CALIB_SYNC_IR_GREY  CALIB_SYNC CALIB_FILE_IR_GREY

#define CALIB_COLOR         "calib_color.yaml"
#define CALIB_IR            "calib_ir.yaml"
#define CALIB_POSE          "calib_pose.yaml"

#define CALIB_CAMERA_MATRIX "cameraMatrix"
#define CALIB_DISTORTION    "distortionCoefficients"
#define CALIB_ROTATION      "rotation"
#define CALIB_PROJECTION    "projection"
#define CALIB_TRANSLATION   "translation"
#define CALIB_ESSENTIAL     "essential"
#define CALIB_FUNDAMENTAL   "fundamental"

#endif //__CAMERA_CALIBRATION_DEFINITIONS_H__
