/*
 * Copyright 2013 Unbounded Robotics Inc.
 * Author: Michael Ferguson
 */

#include <ubr_calibration/checkerboard_finder.h>

void CheckerboardFinder::cameraCallback(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
  if (waiting_)
  {
    cloud_ptr_ = cloud;
    waiting_ = false;
  }
}

/* Returns true if we got a message, false if we timeout. */
bool CheckerboardFinder::waitForCloud()
{
  waiting_ = true;
  int count = 0;
  while (waiting_ && count < 20)
    ros::Duration(0.1).sleep();
  return !waiting_;
}

bool CheckerboardFinder::findCheckerboard(ubr_calibration::CalibrationData * msg,
                                          int points_x,
                                          int points_y)
{
  /* Try up to 50 frames */
  for (int i = 0; i < 50; ++i)
  {
    if (findInternal(msg, points_x, points_y))
      return true;
  }
  return false;
}

bool CheckerboardFinder::findInternal(ubr_calibration::CalibrationData * msg,
                                      int points_x,
                                      int points_y)
{
  geometry_msgs::PointStamped rgbd;
  geometry_msgs::PointStamped world;

  /* Get cloud */
  if(!waitForCloud())
    return false;

  /* Get an OpenCV image from the cloud */
  cv_bridge::CvImagePtr bridge;
  sensor_msgs::ImagePtr image_msg(new sensor_msgs::Image);
  pcl_broke_again::toROSMsg (*cloud_ptr_, *image_msg);
  try
  {
    bridge = cv_bridge::toCvCopy(image_msg, "mono8");  // TODO: was rgb8? does this work?
  }
  catch(cv_bridge::Exception& e)
  {
    ROS_ERROR("Conversion failed");
    return false;
  }

  /* Find checkerboard */
  std::vector<cv::Point2f> points;
  points.resize(points_x * points_y);
  cv::Size checkerboard_size(points_x, points_y);
  int found = cv::findChessboardCorners(bridge->image, checkerboard_size,
                                        points, CV_CALIB_CB_ADAPTIVE_THRESH);

  if (found)
  {
    ROS_INFO("Found the checkboard");

    /* Set msg size */
    msg->rgbd_observations.resize(points_x * points_y);
    msg->world_observations.resize(points_x * points_y);
    
    /* Fill in the headers */
    rgbd.header.seq = cloud_ptr_->header.seq;
    rgbd.header.frame_id = cloud_ptr_->header.frame_id;
    rgbd.header.stamp.fromNSec(cloud_ptr_->header.stamp * 1e3);  // from pcl_conversion

    world.header.frame_id = "checkerboard";

    /* Fill in message */
    for (size_t i = 0; i < points.size(); ++i)
    {
      world.point.x = i / points_x;
      world.point.y = i % points_x;

      /* Get 3d point */
      int index = points[i].y * cloud_ptr_->width + points[i].x;
      rgbd.point.x = cloud_ptr_->points[index].x;
      rgbd.point.y = cloud_ptr_->points[index].y;
      rgbd.point.z = cloud_ptr_->points[index].z;

      /* Do not accept NANs */
      if (isnan(rgbd.point.x) ||
          isnan(rgbd.point.y) ||
          isnan(rgbd.point.z))
      {
        ROS_ERROR("NAN point");
        return false;
      }
      
      msg->rgbd_observations[i] = rgbd;
      msg->world_observations[i] = world;
    }

    /* Found all points */
    return true;
  }

  return false;
}
