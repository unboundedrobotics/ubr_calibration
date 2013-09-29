/*
 * Copyright 2013 Unbounded Robotics Inc.
 * Author: Michael Ferguson
 */

#include <ubr_calibration/led_finder.h>

void LedFinder::cameraCallback(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
  if (waiting_)
  {
    cloud_ptr_ = cloud;
    waiting_ = false;
  }
}

/* Returns true if we got a message, false if we timeout. */
bool LedFinder::waitForCloud()
{
  /* Initial wait cycle so that camera is definitely up to date. */
  ros::Duration(0.25).sleep();

  waiting_ = true;
  int count = 0;
  while (waiting_ && count < 20)
    ros::Duration(0.1).sleep();
  return !waiting_;
}

bool LedFinder::findLed(ubr_calibration::CalibrationData * msg)
{
  geometry_msgs::PointStamped rgbd;
  geometry_msgs::PointStamped world;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr prev_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

  ubr_msgs::GripperLedCommandGoal command;
  command.enable = false;
  client_.sendGoal(command);
  client_.waitForResult(ros::Duration(10.0));

  /* Get initial cloud */
  if(!waitForCloud())
    return false;
  *prev_cloud = *cloud_ptr_;

  /* Differences */
  std::vector<double> diff;
  diff.resize(cloud_ptr_->size());
  double diff_max = -1000;
  int diff_max_idx;
  for (std::vector<double>::iterator it = diff.begin(); it != diff.end(); ++it)
    *it = 0.0;

  int cycles = 0;
  while (true)
  {
    /* Toggle LED */
    command.enable = !command.enable;
    client_.sendGoal(command);
    client_.waitForResult(ros::Duration(10.0));

    /* Get a point cloud */
    if(!waitForCloud())
      return false;

    /* We want to break when the LED is off, so that the pixel is not washed
        out, but want the last update to have been from an on-state. */
    if (diff_max > threshold_ && !command.enable)
      break;

    double sign = command.enable ? 1: -1;

    /* Compare prev_cloud and cloud_ptr_ */
    for (size_t i = 0; i < cloud_ptr_->size(); i++)
    {
      diff[i] += ((double)(cloud_ptr_->points[i].b) - (double)(prev_cloud->points[i].b)) * sign;
      if (diff[i] > diff_max)
      {
        diff_max = diff[i];
        diff_max_idx = i;
      }
    }

    cycles++;
    if (cycles > max_iterations_)
      return false;

    *prev_cloud = *cloud_ptr_;
  }

  if (output_debug_image_)
  {
    /* Convert point cloud into a color image. */
    cv::Mat img(cloud_ptr_->height, cloud_ptr_->width, CV_8UC3);
    int k = 0;
    for (size_t i = 0; i < cloud_ptr_->height; ++i)
    {
      char * s = img.ptr<char>(i);
      for (size_t j = 0; j < cloud_ptr_->width; ++j)
      {
        s[j*3+0] = cloud_ptr_->points[k].b;
        s[j*3+1] = cloud_ptr_->points[k].g;
        s[j*3+2] = cloud_ptr_->points[k].r;
        ++k;
      }
    }

    /* Color any points that are probably part of the LED with small red circles. */
    for (size_t i = 0; i < cloud_ptr_->size(); ++i)
    {
      if (diff[i] > (threshold_ * 0.75))
      {
        cv::Point p(i%cloud_ptr_->width, i/cloud_ptr_->width);
        cv::circle(img, p, 2, cv::Scalar(0,0,255), -1);
      }
    }

    /* Color the center of the LED with a big yellow circle. */
    cv::Point p(diff_max_idx%cloud_ptr_->width, diff_max_idx/cloud_ptr_->width);
    cv::circle(img, p, 5, cv::Scalar(0,255,255), -1);

    /* Show the image */
    cv::imshow("led_finder", img);
    cv::waitKey(3);
  }

  /* diff_max_idx is now the most likely pixel. */
  rgbd.point.x = cloud_ptr_->points[diff_max_idx].x;
  rgbd.point.y = cloud_ptr_->points[diff_max_idx].y;
  rgbd.point.z = cloud_ptr_->points[diff_max_idx].z;

  /* Get a better centroid */
  int centroid_points = 0;
  double centroid_sum_x = 0.0;
  double centroid_sum_y = 0.0;
  double centroid_sum_z = 0.0;
  for (size_t i = 0; i < cloud_ptr_->size(); ++i)
  {
    /* Using highly likely points */
    if (diff[i] > (threshold_ * 0.75))
    {
      double dx = cloud_ptr_->points[i].x - rgbd.point.x;
      double dy = cloud_ptr_->points[i].y - rgbd.point.y;
      double dz = cloud_ptr_->points[i].z - rgbd.point.z;

      /* That are less than 1cm from the max point */
      if ( (dx*dx) + (dy*dy) + (dz*dz) < 0.1 )
      {
        centroid_sum_x += cloud_ptr_->points[i].x;
        centroid_sum_y += cloud_ptr_->points[i].y;
        centroid_sum_z += cloud_ptr_->points[i].z;
        ++centroid_points;
      }
    }
  }

  if (centroid_points == 0)
  {
    ROS_ERROR("No centroid found?");
    return false;
  }

  /* Update from centroid */
  rgbd.point.x = centroid_sum_x/centroid_points;
  rgbd.point.y = centroid_sum_y/centroid_points;
  rgbd.point.z = centroid_sum_z/centroid_points;

  /* Fill in the headers */
#if PCL_VERSION_COMPARE(<,1,7,0)
  rgbd.header = cloud_ptr_->header;
#else
  rgbd.header.seq = cloud_ptr_->header.seq;
  rgbd.header.frame_id = cloud_ptr_->header.frame_id;
  rgbd.header.stamp.fromNSec(cloud_ptr_->header.stamp * 1e3);  // from pcl_conversion
#endif

  /* Do not accept NANs */
  if (isnan(rgbd.point.x) ||
      isnan(rgbd.point.y) ||
      isnan(rgbd.point.z))
    return false;

  /* Transform to led frame */
  try
  {
    listener_.transformPoint("gripper_led_link", ros::Time(0), rgbd,
                             rgbd.header.frame_id, world);
  }
  catch(const tf::TransformException &ex)
  {
    ROS_ERROR("Failed to transform point to gripper_led_link");
    return false;
  }

  /* Compute distance */
  double distance = (world.point.x * world.point.x) +
                    (world.point.y * world.point.y) +
                    (world.point.z * world.point.z);

  /* Point must be within a certain distance of gripper_led_link */
  if (distance > 0.1)
  {
    ROS_ERROR_STREAM("Point was too far away from gripper_led_link: " << distance);
    return false;
  }

  /* Publish point. */
  publisher_.publish(rgbd);

  /* Push back observations */
  msg->rgbd_observations.push_back(rgbd);
  msg->world_observations.push_back(world);

  /* We found a point. */
  return true;
}

bool LedFinder::findGroundPlane(ubr_calibration::CalibrationData* msg)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr prev_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

  /* Get cloud */
  if(!waitForCloud())
    return false;

  /* 6 potential points */
  double x[] = {0.5, 0.5, 1.0, 1.0, 1.5, 1.5};
  double y[] = {-0.25, 0.25, -0.5, 0.5, -0.65, 0.65};

  /* For each potential point, find closest point */
  for (size_t i = 0; i < 6; ++i)
  {
    geometry_msgs::PointStamped rgbd;
    geometry_msgs::PointStamped world;

    double dist = 0.11;

    /* End up cycling through the cloud 6 times, but makes code
       cleaner, and we only do this twice */
    for (size_t j = 0; j < cloud_ptr_->size(); ++j)
    {
      /* No nans */
      if (isnan(cloud_ptr_->points[j].x) ||
          isnan(cloud_ptr_->points[j].y) ||
          isnan(cloud_ptr_->points[j].z))
        continue;

      /* Transform cloud point to base frame */
      geometry_msgs::PointStamped p, p_base;
      #if PCL_VERSION_COMPARE(<,1,7,0)
      p.header = cloud_ptr_->header;  // groovy still uses std_msgs::header in pcl
      #else
      p.header.seq = cloud_ptr_->header.seq;
      p.header.frame_id = cloud_ptr_->header.frame_id;
      p.header.stamp.fromNSec(cloud_ptr_->header.stamp * 1e3);  // from pcl_conversion
      #endif
      p.point.x = cloud_ptr_->points[j].x;
      p.point.y = cloud_ptr_->points[j].y;
      p.point.z = cloud_ptr_->points[j].z;
      try
      {
        listener_.transformPoint("base_link", p, p_base);
      }
      catch(const tf::TransformException &ex)
      {
        ROS_ERROR("Failed to transform point to base_link");
        continue;
      }

      /* Get distance from desired to candidate point */
      double dx = p_base.point.x - x[i];
      double dy = p_base.point.y - y[i];
      double dz = p_base.point.z;
      double d = (dx*dx) + (dy*dy) + (dz*dz);

      if (d < dist)
      {
        /* Set rgbd observation */
        rgbd = p;

        /* Set world observation */
        world.header.frame_id = "ground_plane";
        world.point.x = p_base.point.x;
        world.point.y = p_base.point.y;
        world.point.z = p_base.point.z;

        dist = d;
      }
    }

    if (dist < 0.1)
    {
      /* We found a point, push back observations */
      msg->rgbd_observations.push_back(rgbd);
      msg->world_observations.push_back(world);

      /* Publish point. */
      publisher_.publish(rgbd);
    }
  }

  /* Require 4 points */
  if (msg->rgbd_observations.size() >= 4)
    return true;

  /* Not enough points found */
  return false;
}
