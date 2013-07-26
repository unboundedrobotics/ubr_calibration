/*
 * Copyright 2013 Unbounded Robotics Inc.
 * Author: Michael Ferguson
 */

#ifndef UBR_CALIBRATION_LED_FINDER_H_
#define UBR_CALIBRATION_LED_FINDER_H_

#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <ubr_msgs/GripperLedCommandAction.h>
#include <actionlib/client/simple_action_client.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>

typedef actionlib::SimpleActionClient<ubr_msgs::GripperLedCommandAction> LedClient;

/**
 *  \brief This class processes the point cloud input to find the LED
 */
class LedFinder
{
public:
  LedFinder(ros::NodeHandle & n) :
    client_("/gripper_led_command", true),
    waiting_(false)
  {
    subscriber_ = n.subscribe("/camera/depth_registered/points", 1, &LedFinder::cameraCallback, this);
    client_.waitForServer();

    publisher_ = n.advertise<geometry_msgs::PointStamped>("led_point", 10);

    // TODO: load these from params
    threshold_ = 1000;
    max_iterations_ = 50;

    output_debug_image_ = true;
    if (output_debug_image_)
      cv::namedWindow("led_finder");
  }

  /**
   * \brief Attempts to find the led in incoming data.
   * \param point_stamped This will be filled in with point data, if any.
   * \returns True if point has been filled in.
   */
  bool findLed(geometry_msgs::PointStamped * point_stamped);

private:
  void cameraCallback(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
  bool waitForCloud();

  ros::Subscriber subscriber_;  /// Incoming sensor_msgs::Image
  ros::Publisher publisher_;  /// Outgoing geometry_msgs::PointStamped
  LedClient client_;

  bool waiting_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr_;

  double threshold_;  /// Minimum value of diffs in order to trigger that this is an LED
  int max_iterations_;  /// Maximum number of cycles before we abort finding the LED

  bool output_debug_image_;
};

#endif  // UBR_CALIBRATION_LED_FINDER_H_
