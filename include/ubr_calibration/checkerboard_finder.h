/*
 * Copyright 2013 Unbounded Robotics Inc.
 * Author: Michael Ferguson
 */

#ifndef UBR_CALIBRATION_CHECKERBOARD_FINDER_H_
#define UBR_CALIBRATION_CHECKERBOARD_FINDER_H_

#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
//#include <tf/transform_listener.h>
//#include <sensor_msgs/PointCloud2.h>
//#include <geometry_msgs/PointStamped.h>
#include <ubr_calibration/CalibrationData.h>

#include <opencv2/calib3d/calib3d.hpp>

//#include <opencv/cv.h>
//#include <opencv/highgui.h>

#include <ubr_calibration/pcl_conversions.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cv_bridge/cv_bridge.h>

class CheckerboardFinder
{
public:
  CheckerboardFinder(ros::NodeHandle & n) : waiting_(false)
  {
    subscriber_ = n.subscribe("/head_camera/depth_registered/points",
                              1,
                              &CheckerboardFinder::cameraCallback,
                              this);
  }

  /**
   * \brief Attempts to find the checkerboard incoming data.
   * \param msg CalibrationData instance to fill in with point information.
   * \param points_x Number of checkerboard points in x
   * \param points_y Number of checkerboard points in y
   * \returns True if point has been filled in.
   */
  bool findCheckerboard(ubr_calibration::CalibrationData * msg,
                        int points_x = 4, int points_y = 5);

private:
  bool findInternal(ubr_calibration::CalibrationData * msg,
                    int points_x, int points_y);

  void cameraCallback(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
  bool waitForCloud();

  ros::Subscriber subscriber_;  /// Incoming sensor_msgs::PointCloud2

  bool waiting_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr_;
};

#endif  // UBR_CALIBRATION_CHECKERBOARD_FINDER_H_
