/*
 * Copyright 2013 Unbounded Robotics Inc.
 * Author: Michael Ferguson
 */

#ifndef UBR_CALIBRATION_CAPTURE_CHECKERBOARD_FINDER_H_
#define UBR_CALIBRATION_CAPTURE_CHECKERBOARD_FINDER_H_

#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <ubr_calibration/capture/feature_finder.h>
#include <ubr_calibration/CalibrationData.h>

#include <opencv2/calib3d/calib3d.hpp>
#include <ubr_calibration/pcl_conversions.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cv_bridge/cv_bridge.h>

namespace ubr_calibration
{

/**
 *  \brief This class processes the point cloud input to find a checkerboard
 */
class CheckerboardFinder : public FeatureFinder
{
public:
  CheckerboardFinder(ros::NodeHandle & n);

  /**
   * \brief Attempts to find the checkerboard incoming data.
   * \param msg CalibrationData instance to fill in with point information.
   * \param points_x Number of checkerboard points in x
   * \param points_y Number of checkerboard points in y
   * \returns True if point has been filled in.
   */
  bool find(ubr_calibration::CalibrationData * msg);

private:
  bool findInternal(ubr_calibration::CalibrationData * msg);

  void cameraCallback(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
  bool waitForCloud();

  int points_x_;
  int points_y_;

  ros::Subscriber subscriber_;  /// Incoming sensor_msgs::PointCloud2

  bool waiting_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr_;
};

}  // namespace ubr_calibration

#endif  // UBR_CALIBRATION_CAPTURE_CHECKERBOARD_FINDER_H_
