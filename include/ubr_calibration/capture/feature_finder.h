/*
 * Copyright 2014 Unbounded Robotics Inc.
 * Author: Michael Ferguson
 */

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
