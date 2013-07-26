/*
 * Copyright 2013 Unbounded Robotics Inc.
 * Author: Michael Ferguson
 */

#include <ubr_calibration/chain_manager.h>

// TODO: need mutex here?
void ChainManager::stateCallback(const sensor_msgs::JointStateConstPtr& msg)
{
  state_ = *msg;
}

bool ChainManager::getState(sensor_msgs::JointState* state)
{
  *state = state_;
}

bool ChainManager::moveToState(const sensor_msgs::JointState& state)
{
  control_msgs::PointHeadGoal head_goal;
  control_msgs::FollowJointTrajectoryGoal arm_goal;

  // split joints into head/arm


  // call head and arm actions, wait for results

  return true;
}

bool ChainManager::waitToSettle()
{
  return true;
}
