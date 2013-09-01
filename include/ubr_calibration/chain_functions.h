/*
 * Copyright 2013 Unbounded Robotics Inc.
 * Author: Michael Ferguson
 */

#ifndef UBR_CALIBRATION_CHAIN_FUNCTIONS_H_
#define UBR_CALIBRATION_CHAIN_FUNCTIONS_H_

#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <sensor_msgs/JointState.h>

/**
 *  \brief Get a KDL::JntArray of joint positions from a sensor_msgs::JointState
 *         for a set of joints in a particular KDL::Chain. Yeah, that seems pretty specific.
 */
KDL::JntArray getChainPositionsFromMsg(const KDL::Chain& chain,
                                       const sensor_msgs::JointState& msg)
{
  // initialize with length
  KDL::JntArray positions(chain.getNrOfJoints());

  // iterate through segments
  int j = 0;
  for (int i = 0; i < chain.getNrOfSegments(); ++i)
  {
    // some segments are fixed joints, ignore these
    if (chain.getSegment(i).getJoint().getType()!=KDL::Joint::None)
    {
      // find joint in sensor message, by name
      std::string name = chain.getSegment(i).getJoint().getName();
      for (int k = 0; k < msg.name.size(); ++k)
      {
        if (msg.name[k] == name)
        {
          positions.data[j] = msg.position[k];
          break;
        }
      }
      ++j;
    }
  }

  return positions;
}

/**
 *  \brief Converts our angle-axis-with-integrated-magnitude representation to a KDL::Rotation
 */
KDL::Rotation rotation_from_axis_magnitude(const double x, const double y, const double z)
{
  double magnitude = sqrt(x*x + y*y + z*z);

  if (magnitude == 0.0)
    return KDL::Rotation::Quaternion(0.0, 0.0, 0.0, 1.0);

  return KDL::Rotation::Quaternion(x/magnitude * sin(magnitude/2.0),
                                   y/magnitude * sin(magnitude/2.0),
                                   z/magnitude * sin(magnitude/2.0),
                                   cos(magnitude/2.0));
}

#endif  // UBR_CALIBRATION_CHAIN_FUNCTIONS_H_
