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

trajectory_msgs::JointTrajectoryPoint
ChainManager::makePoint(const sensor_msgs::JointState& state, const std::vector<std::string> joints)
{
  trajectory_msgs::JointTrajectoryPoint p;
  for (size_t i = 0; i < joints.size(); ++i)
  {
    for (size_t j = 0; j < state.name.size(); ++j)
      if (joints[i] == state.name[j])
      {
        p.positions.push_back(state.position[j]);
        break;
      }
    p.velocities.push_back(0.0);
    if (p.velocities.size() != p.positions.size())
    {
      ROS_ERROR_STREAM("Bad move to state, missing " << joints[i]);
      exit(-1);
    }
  }
  return p;
}

bool ChainManager::moveToState(const sensor_msgs::JointState& state)
{
  /* Split into head and arm */
  control_msgs::FollowJointTrajectoryGoal head_goal;
  head_goal.trajectory.joint_names = head_joints_;

  trajectory_msgs::JointTrajectoryPoint p = makePoint(state, head_joints_);
  p.time_from_start = ros::Duration(10.0);
  head_goal.trajectory.points.push_back(p);
  head_goal.goal_time_tolerance = ros::Duration(1.0);

  control_msgs::FollowJointTrajectoryGoal arm_goal;
  arm_goal.trajectory.joint_names = arm_joints_;

  p = makePoint(state, arm_joints_);
  p.time_from_start = ros::Duration(10.0);
  arm_goal.trajectory.points.push_back(p);
  arm_goal.goal_time_tolerance = ros::Duration(1.0);

  /* Call actions */
  head_client_.sendGoal(head_goal);
  arm_client_.sendGoal(arm_goal);

  /* Wait for results */
  head_client_.waitForResult(ros::Duration(15.0));
  arm_client_.waitForResult(ros::Duration(15.0));

  // TODO: catch errors with clients

  return true;
}

bool ChainManager::waitToSettle()
{
  sensor_msgs::JointState state;
  std::vector<double> last(head_joints_.size() + arm_joints_.size(), 0.0);
  std::vector<double> next(head_joints_.size() + arm_joints_.size(), 0.0);
  int stable = 0;

  // TODO: timeout?
  while (true)
  {
    getState(&state);

    /* Get head positions */
    for (size_t i = 0; i < head_joints_.size(); ++i)
    {
      for (size_t j = 0; j < state.name.size(); ++j)
        if (head_joints_[i] == state.name[j])
        {
          next[i] = state.position[j];
          break;
        }
    }

    /* Get arm positions */
    for (size_t i = 0; i < arm_joints_.size(); ++i)
    {
      for (size_t j = 0; j < state.name.size(); ++j)
        if (arm_joints_[i] == state.name[j])
        {
          next[head_joints_.size() + i] = state.position[j];
          break;
        }
    }

    /* Compare next to last */
    std::vector<double> diff(last.size(), 0.0);
    bool settled = true;
    for (size_t i = 0; i < last.size(); ++i)
    {
      diff[i] = next[i] - last[i];
      settled &= (fabs(diff[i]) < 0.001);  // TODO: set this value by parameter and tune
    }

    if (settled)
      break;
  }

  return true;
}
