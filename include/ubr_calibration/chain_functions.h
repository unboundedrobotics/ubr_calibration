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
  /* Check that message is filled in */
  if (msg.name.size() == 0)
  {
    std::cerr << "sensor_msgs::JointState is empty" << std::endl;
  }

  /* Initialize with length */
  KDL::JntArray positions(chain.getNrOfJoints());

  /* Iterate through segments */
  int j = 0;
  for (int i = 0; i < chain.getNrOfSegments(); ++i)
  {
    /* Some segments are fixed joints, ignore these */
    if (chain.getSegment(i).getJoint().getType()!=KDL::Joint::None)
    {
      /* Find joint in sensor message, by name */
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

/**
 *  \brief Data structure for generalizing specification of calibration.
 *         This maps free parameters to frame updates.
 *
 *  NOTE: If calibrating orientations, either 1 or 3 of the parameters should
 *        be filled in, as an angle-axis parameterization with 1 of 3 values
 *        locked to 0 does not mean what you want it to mean.
 */
struct FrameCalibrationData
{
  FrameCalibrationData (int x_, int y_, int z_, int roll_, int pitch_, int yaw_) :
    x(x_), y(y_), z(z_), roll(roll_), pitch(pitch_), yaw(yaw_), idx(-1), calibrate(true)
  {
  }

  explicit FrameCalibrationData (int idx_) :
    x(-1), y(-1), z(-1), roll(-1), pitch(-1), yaw(-1), idx(idx_), calibrate(true)
  {
  }

  FrameCalibrationData () : calibrate(false)
  {
  }

  /*
   * Will be stored in a std::map<std::string, boost::shared_ptr<FrameCalibrationInfo> >
   * so there is no need to store name here.
   */

  /** \brief The index of the x offset parameter, -1 if not calibrating x */
  int x;

  /** \brief The index of the y offset parameter, -1 if not calibrating y */
  int y;

  /** \brief The index of the z offset parameter, -1 if not calibrating z */
  int z;

  /** \brief The index of the roll offset parameter, -1 if not calibrating roll */
  int roll;

  /** \brief The index of the pitch offset parameter, -1 if not calibrating pitch */
  int pitch;

  /** \brief The index of the yaw offset parameter, -1 if not calibrating yaw */
  int yaw;

  /** \brief The index of the joint, if this is an active joint */
  int idx;

  /** \brief Should we calibrate this? */
  bool calibrate;
};

typedef std::map<std::string, FrameCalibrationData> FrameCalibrationInfo;
typedef std::map<std::string, FrameCalibrationData>::iterator FrameCalibrationInfoIterator;

#endif  // UBR_CALIBRATION_CHAIN_FUNCTIONS_H_
