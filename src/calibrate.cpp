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

#include <boost/foreach.hpp>  // for rosbag iterator

/** \mainpage
 * \section parameters Parameters of the Optimization:
 *   - giant pile of 3d LED points (or maybe checkerboard points if we need to)
 *   - joint angle offsets R, camera pose XYZRPY
 *
 * \section residuals Residual Blocks:
 *   - reprojection of 3d point, through the arm
 *   - reprojection of 3d point, through the 3d Kinect
 *
 *  For each block, it will need to be able to reproject THROUGH a particular set of joint angles.
 *
 * \section modules Modules:
 *   - Capture:
 *     -# move joints to a particular place
 *     -# wait to settle
 *     -# find LED in the camera view
 *        -# find point where LED is flashing.
 *        -# create plane where point is.
 *        -# sample center point of LED.
 *     -# write sample to bag file: joint angles, position of LED in camera. Debug data?
 *   - Calibrate:
 *     -# load urdf, samples from bag file. (implemented)
 *     -# create pile of 3d points by projecting through arm. Create
 *        joint angles, camera pose initial values. (implemented)
 *     -# create residual blocks. (implemented, might evolve some still)
 *     -# run calibration. (implemented)
 *     -# write results to URDF. (ewww)
 */

int main(int argc, char** argv)
{
  /* Load bagfile of calibration dat. */
  rosbag::Bag bag_;
  bag_.open("calibration_data.bag", rosbag::bagmode::Read);

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
  std::vector<ubr_calibration::CalibrationData> data;
  BOOST_FOREACH (rosbag::MessageInstance const m, data_view_)
  {
    ubr_calibration::CalibrationData::ConstPtr msg = m.instantiate<ubr_calibration::CalibrationData>();
    data.push_back(*msg);
  }

  /* Create instance of optimizer. */
  Optimizer opt(description_->data, "base_link", "gripper_link");
  opt.optimize(data, true);
  opt.printResult();
  // TODO: use free_params to update and export new URDF

  return 0;
}
