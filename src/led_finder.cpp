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

/*
 * TODO: Future Improvements
 *  Accuracy might be improved by finding a plane around the gripper LED.
 *  Actually finding the real centroid of the max diff region may also improve calibration.
 */
bool LedFinder::findLed(geometry_msgs::PointStamped * point_stamped)
{
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
    for (size_t i = 0; i < cloud_ptr_->size(); i++)
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
  point_stamped->point.x = cloud_ptr_->points[diff_max_idx].x;
  point_stamped->point.y = cloud_ptr_->points[diff_max_idx].y;
  point_stamped->point.z = cloud_ptr_->points[diff_max_idx].z;
#if PCL_VERSION_COMPARE(<,1,7,0)
  point_stamped->header = cloud_ptr_->header;
#else
  point_stamped->header.seq = cloud_ptr_->header.seq;
  point_stamped->header.frame_id = cloud_ptr_->header.frame_id;
  point_stamped->header.stamp.fromNSec(cloud_ptr_->header.stamp * 1e3);  // from pcl_conversion
#endif

  /* Publish point. */
  publisher_.publish(*point_stamped);

  /* We found a point. */
  return true;
}
