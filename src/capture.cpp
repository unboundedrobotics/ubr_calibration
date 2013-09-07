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
    exit(-1);
  }
  urdf_pub.publish(description_msg);

  /* Take name of poses from command line if supplied */
  std::string pose_bag_name("calibration_poses.bag");
  if (argc > 1)
    pose_bag_name = argv[1];

  /* Load a set of calibration poses */
  rosbag::Bag bag;
  try
  {
    bag.open(pose_bag_name, rosbag::bagmode::Read);
  }
  catch (rosbag::BagException)
  {
    ROS_FATAL_STREAM("Cannot open " << pose_bag_name);
    exit(-1);
  }
  rosbag::View data_view(bag, rosbag::TopicQuery("calibration_joint_states"));

  std::vector<sensor_msgs::JointState> poses;
  BOOST_FOREACH (rosbag::MessageInstance const m, data_view)
  {
    sensor_msgs::JointState::ConstPtr msg = m.instantiate<sensor_msgs::JointState>();
    poses.push_back(*msg);
  }

  ros::AsyncSpinner spinner(1);
  spinner.start();

  /* For each pose in the capture sequence. */
  for (std::vector<sensor_msgs::JointState>::iterator it = poses.begin();
       it != poses.end(); ++it)
  {
    ubr_calibration::CalibrationData msg;

    /* Move head/arm to pose */
    chain_manager_.moveToState(*it);
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
