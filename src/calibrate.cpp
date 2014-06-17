/*
 * Copyright (C) 2013-2014 Unbounded Robotics Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// Author: Michael Ferguson

#include <fstream>
#include <ctime>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosbag/query.h>

#include <std_msgs/String.h>
#include <ubr_calibration/CalibrationData.h>

#include <ubr_calibration/capture/chain_manager.h>
#include <ubr_calibration/capture/checkerboard_finder.h>
#include <ubr_calibration/capture/led_finder.h>
#include <ubr_calibration/depth_camera.h>

#include <camera_calibration_parsers/parse.h>
#include <ubr_calibration/ceres/optimizer.h>
#include <ubr_calibration/depth_camera.h>

#include <boost/foreach.hpp>  // for rosbag iterator

/** \mainpage
 * \section parameters Parameters of the Optimization:
 *   - joint angle offsets
 *   - frame 6DOF corrections (currently head pan frame, and camera frame)
 *   - camera intrinsics (2d & 3d)
 *
 * \section residuals Residual Blocks:
 *   - difference of reprojection through the arm and camera
 *   - residual blocks that limit offsets from growing outrageously large
 *
 * \section modules Modules:
 *   - Capture:
 *     - move joints to a particular place
 *     - wait to settle
 *     - find LEDs in the camera view. This currently works, but might be
 *       be improved by sampling a plane in the local region of the point.
 *     - write sample to bag file: joint angles, position of LEDs in camera.
 *   - Calibrate:
 *     - load urdf, samples from bag file.
 *     - create arm and camera reprojection chains.
 *     - create residual blocks.
 *     - run calibration.
 *     - write results to URDF.
 */

/*
 * usage:
 *  calibrate --manual
 *  calibrate calibration_poses.bag
 *  calibrate --from-bag calibration_data.bag
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv,"calibration_node");
  ros::NodeHandle nh;

  // Should we be stupidly verbose?
  bool verbose;
  nh.param<bool>("verbose_calibration", verbose, false);

  // The calibration data
  std_msgs::String description_msg;
  std::vector<ubr_calibration::CalibrationData> data;

  // What bag to use to load calibration poses out of (for capture)
  std::string pose_bag_name("calibration_poses.bag");
  if (argc > 1)
    pose_bag_name = argv[1];

  if (pose_bag_name.compare("--from-bag") != 0)
  {
    // no name provided for a calibration bag file, must do capture
    ubr_calibration::ChainManager chain_manager_(nh);
    ubr_calibration::FeatureFinder * finder_;
    if (nh.hasParam("use_checkerboards"))
      finder_ = new ubr_calibration::CheckerboardFinder(nh);
    else
      finder_ = new ubr_calibration::LedFinder(nh);

    ros::Publisher pub = nh.advertise<ubr_calibration::CalibrationData>("calibration_data", 10);
    ros::Publisher urdf_pub = nh.advertise<std_msgs::String>("robot_description", 1, true);  // latched

    // Get the robot_description and republish it
    if (!nh.getParam("robot_description", description_msg.data))
    {
      ROS_FATAL("robot_description not set!");
      return -1;
    }
    urdf_pub.publish(description_msg);

    // Get the camera parameters
    ubr_calibration::DepthCameraInfoManager depth_camera_manager;
    if (!depth_camera_manager.init(nh))
      return -1;

    // Load a set of calibration poses
    std::vector<sensor_msgs::JointState> poses;
    if (pose_bag_name.compare("--manual") != 0)
    {
      if (verbose)
        ROS_INFO_STREAM("Opening " << pose_bag_name);
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

    // For each pose in the capture sequence.
    for (unsigned int pose_idx = 0; (pose_idx < poses.size()) || (poses.size() == 0); ++pose_idx)
    {
      ubr_calibration::CalibrationData msg;

      if (poses.size() == 0)
      {
        // Manual calibration, wait for keypress
        ROS_INFO("Press key when arm is ready...");
        std::string throwaway;
        std::getline(std::cin, throwaway);
        if (throwaway.compare("done") == 0)
          break;
        if (!ros::ok())
          break;
      }
      else
      {
        // Move head/arm to pose
        chain_manager_.moveToState(poses[pose_idx]);
      }

      // Regardless of manual vs. automatic, wait for joints to settle
      chain_manager_.waitToSettle();

      // Get pose of the features
      if (!finder_->find(&msg))
      {
        ROS_WARN("Failed to capture sample %u.", pose_idx);
        continue;
      }
      else
      {
        ROS_INFO("Captured pose %u", pose_idx);
      }

      // Fill in joint values
      chain_manager_.getState(&msg.joint_states);

      // Fill in camera info
      msg.rgbd_info = depth_camera_manager.getDepthCameraInfo();

      // Publish calibration data message.
      pub.publish(msg);

      // Add to samples
      data.push_back(msg);
    }

    ROS_INFO("Done capturing samples");
  }
  else
  {
    // Load calibration data from bagfile
    std::string data_bag_name("/tmp/calibration_data.bag");
    if (argc > 2)
      data_bag_name = argv[2];
    ROS_INFO_STREAM("Loading calibration data from " << data_bag_name);

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

    // Get robot_description from bag file
    rosbag::View model_view_(bag_, rosbag::TopicQuery("robot_description"));
    if (model_view_.size() < 1)
    {
      std::cerr << "robot_description topic not found in bag file." << std::endl;
      return -1;
    }
    std_msgs::String::ConstPtr description_ = model_view_.begin()->instantiate<std_msgs::String>();
    description_msg = *description_;

    // Parse calibration_data topic
    rosbag::View data_view_(bag_, rosbag::TopicQuery("calibration_data"));
    BOOST_FOREACH (rosbag::MessageInstance const m, data_view_)
    {
      ubr_calibration::CalibrationData::ConstPtr msg = m.instantiate<ubr_calibration::CalibrationData>();
      data.push_back(*msg);
    }
  }

  // Create instance of optimizer
  ubr_calibration::Optimizer opt(description_msg.data, "base_link", "wrist_roll_link");
  opt.optimize(data, verbose);
  if (verbose)
  {
    std::cout << "Parameter Offsets:" << std::endl;
    std::cout << opt.getOffsets().getOffsetYAML() << std::endl;
  }

  // Update the URDF
  std::string s = opt.getOffsets().updateURDF(description_msg.data);

  // TODO: generate datecode
  char datecode[80];
  {
    std::time_t t = std::time(NULL);
    std::strftime(datecode, 80, "%Y_%m_%d_%H_%M_%S", std::localtime(&t));
  }

  // Save updated URDF
  {
    std::stringstream urdf_name;
    urdf_name << "/tmp/ubr1_calibrated_" << datecode << ".urdf";
    std::ofstream file;
    file.open(urdf_name.str().c_str());
    file << s;
    file.close();
  }

  // Output camera calibration
  {
    std::stringstream depth_name;
    depth_name << "/tmp/depth_" << datecode << ".yaml";
    camera_calibration_parsers::writeCalibration(depth_name.str(), "",
        ubr_calibration::updateCameraInfo(
                         opt.getOffsets().get("camera_fx"),
                         opt.getOffsets().get("camera_fy"),
                         opt.getOffsets().get("camera_cx"),
                         opt.getOffsets().get("camera_cy"),
                         data[0].rgbd_info.camera_info));

    std::stringstream rgb_name;
    rgb_name << "/tmp/rgb_" << datecode << ".yaml";
    camera_calibration_parsers::writeCalibration(rgb_name.str(), "",
        ubr_calibration::updateCameraInfo(
                         opt.getOffsets().get("camera_fx"),
                         opt.getOffsets().get("camera_fy"),
                         opt.getOffsets().get("camera_cx"),
                         opt.getOffsets().get("camera_cy"),
                         data[0].rgbd_info.camera_info));
  }

  // Output the calibration yaml
  {
    std::ofstream file;
    file.open("/tmp/calibration.yaml");
    file << opt.getOffsets().getOffsetYAML();
    file << "depth_info: depth_" << datecode << ".yaml" << std::endl;
    file << "rgb_info: rgb_" << datecode << ".yaml" << std::endl;
    file << "urdf: ubr1_calibrated_" << datecode << ".urdf" << std::endl;
    file.close();
  }

  ROS_INFO("Done calibrating");

  return 0;
}
