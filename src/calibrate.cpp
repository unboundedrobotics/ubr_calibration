/*
 * Copyright 2013 Unbounded Robotics Inc.
 * Author: Michael Ferguson
 */

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosbag/query.h>

#include <std_msgs/String.h>
#include <ubr_calibration/CalibrationData.h>

#include <ubr_calibration/optimizer.h>
#include <ubr_calibration/update_urdf.h>

#include <boost/foreach.hpp>  // for rosbag iterator
#include <fstream>

/** \mainpage
 * \section parameters Parameters of the Optimization:
 *   - giant pile of 3d LED points (or maybe checkerboard points if we need to)
 *   - joint angle offsets R, camera pose XYZRPY
 *
 * \section residuals Residual Blocks:
 *   - reprojection of 3d point, through the arm
 *   - reprojection of 3d point, through the 3d Kinect
 *   - ground error, such that points from the ground are actually at z = 0.0
 *
 *  For each block, it will need to be able to reproject THROUGH a particular set of joint angles.
 *
 * \section modules Modules:
 *   - Capture:
 *     -# move joints to a particular place
 *     -# wait to settle
 *     -# find LED in the camera view
 *        -# find point where LED is flashing. (might be improved)
 *        -# create plane where point is. (not implemented, might not be needed)
 *        -# sample center point of LED.
 *     -# write sample to bag file: joint angles, position of LED in camera. Debug data?
 *   - Calibrate:
 *     -# load urdf, samples from bag file.
 *     -# create pile of 3d points by projecting through arm. Create
 *        joint angles, camera pose initial values.
 *     -# create residual blocks.
 *     -# run calibration.
 *     -# write results to URDF.
 */

int main(int argc, char** argv)
{
  /* Take name of poses from command line if supplied */
  std::string data_bag_name("calibration_data.bag");
  if (argc > 1)
    data_bag_name = argv[1];

  /* Load bagfile of calibration data. */
  rosbag::Bag bag_;
  try
  {
    bag_.open(data_bag_name, rosbag::bagmode::Read);
  }
  catch (rosbag::BagException)
  {
    ROS_FATAL_STREAM("Cannot open " << data_bag_name);
    return -1;
  }

  /* Get robot_description from bag file. */
  rosbag::View model_view_(bag_, rosbag::TopicQuery("robot_description"));
  if (model_view_.size() < 1)
  {
    std::cerr << "robot_description topic not found in bag file." << std::endl;
    return -1;
  }
  std_msgs::String::ConstPtr description_ = model_view_.begin()->instantiate<std_msgs::String>();

  /* Parse calibration_data topic */
  rosbag::View data_view_(bag_, rosbag::TopicQuery("calibration_data"));

  /* Find typical offset */
  double x = 0.0; double y = 0.0; double z = 0.0;
  int msg_count = 0;
  BOOST_FOREACH (rosbag::MessageInstance const m, data_view_)
  {
    ubr_calibration::CalibrationData::ConstPtr msg = m.instantiate<ubr_calibration::CalibrationData>();
    if (msg->world_observations[0].header.frame_id != "gripper_led_link")
      continue;
    x += msg->world_observations[0].point.x;
    y += msg->world_observations[0].point.y;
    z += msg->world_observations[0].point.z;
    ++msg_count;
  }
  x = x/msg_count;
  y = y/msg_count;
  z = z/msg_count;

  std::vector<ubr_calibration::CalibrationData> data;
  BOOST_FOREACH (rosbag::MessageInstance const m, data_view_)
  {
    ubr_calibration::CalibrationData::ConstPtr msg = m.instantiate<ubr_calibration::CalibrationData>();
    if (msg->world_observations[0].header.frame_id == "gripper_led_link")
    {
      /* Include only 'good' samples */
      double dx = msg->world_observations[0].point.x - x;
      double dy = msg->world_observations[0].point.y - y;
      double dz = msg->world_observations[0].point.z - z;
      double d = sqrt((dx*dx) + (dy*dy) + (dx*dx));
      if (d > 0.04)
      {
        continue;
      }
    }
    data.push_back(*msg);
  }

  /* Create instance of optimizer. */
  Optimizer opt(description_->data, "base_link", "gripper_led_link");
  opt.optimize(data, true);
  opt.printResult();

  /* Update the URDF. */
  std::map<std::string, double> offsets = opt.getCalibrationOffsets();
  std::string s = updateURDF(description_->data, offsets);

  /* Save updated URDF. */
  std::ofstream file;
  file.open("calibrated.urdf");
  file << s;
  file.close();

  return 0;
}
