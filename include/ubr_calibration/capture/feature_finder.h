/*
 * Copyright (C) 2013-2014 Unbounded Robotics Inc.
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

// Author: Michael Ferguson

#ifndef UBR_CALIBRATION_CAPTURE_FEATURE_FINDER_H_
#define UBR_CALIBRATION_CAPTURE_FEATURE_FINDER_H_

#include <ros/ros.h>
#include <ubr_calibration/CalibrationData.h>

namespace ubr_calibration
{

/**
 *  \brief Base class for a feature finder.
 */
class FeatureFinder
{
public:
  FeatureFinder(ros::NodeHandle & n) {};
  virtual ~FeatureFinder() {};

  virtual bool find(ubr_calibration::CalibrationData * msg) = 0;
};

}  // namespace ubr_calibration

#endif  // UBR_CALIBRATION_CAPTURE_FEATURE_FINDER_H_
