/*
 * Copyright 2013 Unbounded Robotics Inc.
 * Author: Michael Ferguson
 */

#ifndef UBR_CALIBRATION_CHAIN_MANAGER_H_
#define UBR_CALIBRATION_CHAIN_MANAGER_H_

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <control_msgs/PointHeadAction.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<control_msgs::PointHeadAction> PointHeadClient;
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> TrajectoryClient;

/**
 *  \brief Manages moving joints to a new pose, determining when they
 *  are settled, and returning current joint_states.
 */
class ChainManager
{
public:
  ChainManager(ros::NodeHandle & n) :
    head_client_("/point_head", true),
    arm_client_("/arm_controller/follow_joint_trajectory", true)
  {
    subscriber_ = n.subscribe("/joint_states", 1, &ChainManager::stateCallback, this);

    head_client_.waitForServer();
    arm_client_.waitForServer();

    head_joints_.push_back("head_pan_joint");
    head_joints_.push_back("head_tilt_joint");

    arm_joints_.push_back("shoulder_pan_joint");
    arm_joints_.push_back("shoulder_lift_joint");
    arm_joints_.push_back("upperarm_roll_joint");
    arm_joints_.push_back("elbow_flex_joint");
    arm_joints_.push_back("forearm_roll_joint");
    arm_joints_.push_back("wrist_flex_joint");
    arm_joints_.push_back("wrist_roll_joint");
  }

  /**
   *  \brief Send commands to all managed joints. The ChainManager automatically figures out
   *         which controller to send these to.
   *  \returns False if failed.
   */
  bool moveToState(const sensor_msgs::JointState& state);

  /**
   *  \brief Wait for joints to settle.
   */
  bool waitToSettle();

  /**
   *  \brief Get the current JointState message.
   */
  bool getState(sensor_msgs::JointState* state);

private:
  void stateCallback(const sensor_msgs::JointStateConstPtr& msg);

  ros::Subscriber subscriber_;
  sensor_msgs::JointState state_;

  PointHeadClient head_client_;
  TrajectoryClient arm_client_;

  std::vector<std::string> head_joints_;
  std::vector<std::string> arm_joints_;
};

#endif  // UBR_CALIBRATION_CHAIN_MANAGER_H_
