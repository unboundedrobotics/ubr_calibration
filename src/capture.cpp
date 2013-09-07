/*
 * Copyright 2013 Unbounded Robotics Inc.
 * Author: Michael Ferguson
 */

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <ubr_calibration/CalibrationData.h>
#include <ubr_calibration/led_finder.h>
#include <ubr_calibration/chain_manager.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosbag/query.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv,"ubr_calibration_capture");
  ros::NodeHandle nh;

  LedFinder led_finder_(nh);
  ChainManager chain_manager_(nh);

  ros::Publisher pub = nh.advertise<ubr_calibration::CalibrationData>("calibration_data", 10);
  ros::Publisher urdf_pub = nh.advertise<std_msgs::String>("robot_description", 1, true);

  /* Get the robot_description and republish it */
  std_msgs::String description_msg;
  if (!nh.getParam("robot_description", description_msg.data))
  {
    ROS_FATAL("robot_description not set!");
    return -1;
  }
  urdf_pub.publish(description_msg);

  /* Take name of poses from command line if supplied */
  std::string pose_bag_name("calibration_poses.bag");
  if (argc > 1)
    pose_bag_name = argv[1];

  /* Load a set of calibration poses, if not manually calibrating. */
  std::vector<sensor_msgs::JointState> poses;
  if (pose_bag_name.compare("--manual") != 0)
  {
    rosbag::Bag bag;
    try
    {
      bag.open(pose_bag_name, rosbag::bagmode::Read);
    }
    catch (rosbag::BagException)
    {
      ROS_FATAL_STREAM("Cannot open " << pose_bag_name);
      return -1;
    }
    rosbag::View data_view(bag, rosbag::TopicQuery("calibration_joint_states"));

    BOOST_FOREACH (rosbag::MessageInstance const m, data_view)
    {
      sensor_msgs::JointState::ConstPtr msg = m.instantiate<sensor_msgs::JointState>();
      poses.push_back(*msg);
    }
  }
  else
  {
    ROS_INFO("Using manual calibration mode...");
  }

  ros::AsyncSpinner spinner(1);
  spinner.start();

  /* For each pose in the capture sequence. */
  for (std::vector<sensor_msgs::JointState>::iterator it = poses.begin();
       (it != poses.end()) || (poses.size() == 0); ++it)
  {
    ubr_calibration::CalibrationData msg;

    if (poses.size() == 0)
    {
      /* Manual calibration, wait for keypress */
      ROS_INFO("Press key when arm is ready...");
      std::string throwaway;
      std::getline(std::cin, throwaway);
      if (throwaway.compare("exit") == 0)
        break;
    }
    else
    {
      /* Move head/arm to pose */
      chain_manager_.moveToState(*it);
    }

    /* Regardless of manual vs. automatic, wait for joints to settle */
    chain_manager_.waitToSettle();

    /* Get pose of the LED */
    msg.rgbd_observations.resize(1);
    if (!led_finder_.findLed(&msg.rgbd_observations[0]))
    {
      ROS_WARN("Failed to capture sample.");
      continue;
    }

    /* Fill in joint values */
    chain_manager_.getState(&msg.joint_states);

    /* Publish calibration data message. */
    pub.publish(msg);
  }
  ROS_INFO("Done Capturing Samples");
}
