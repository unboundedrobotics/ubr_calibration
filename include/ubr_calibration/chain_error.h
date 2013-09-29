/*
 * Copyright 2013 Unbounded Robotics Inc.
 * Author: Michael Ferguson
 */

#ifndef UBR_CALIBRATION_CHAIN_ERROR_H_
#define UBR_CALIBRATION_CHAIN_ERROR_H_

#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>

#include <ubr_calibration/chain_functions.h>

#include <ceres/ceres.h>

#include <string>

/**
 *  \brief Used to find error in reprojection using a chain of joints.
 */
struct ChainError
{
  /**
   *  \brief Standard construct for a residual block based on projection through a chain.
   *  \param chain The KDL chain for the robot.
   *  \param positions The position of the joints when this measurement was taken.
   *  \param info The frame calibration info.
   *  \param offset The offset applied to this block, which must be subtracted from indexes in info.
   *  \param root The name of the root link (typically 'base_link')
   *  \param tip The name of the tip of the chain to be used (typically the camera sensor frame
   *         or gripper LED frame).
   */
  ChainError(KDL::Chain& chain, KDL::JntArray& positions,
             FrameCalibrationInfo* info, int offset,
             std::string root, std::string tip) :
    chain_(chain), positions_(positions), info_(info), offset_(offset), root_(root), tip_(tip)
  {
  }

  /**
   *  \brief Operator called by CERES optimizer.
   *  \param free_params The joint angle offsets to be applied to each joint.
   *  \param point The 3D point estimate to be compared with observed value.
   *  \param residuals The residuals computed, to be returned to the optimizer.
   */
  bool operator()(const double* const free_params,
                  const double* const point,
                  double* residuals) const
  {
    double expected[3];
    double measurement[3];

    getExpected(free_params, point, &expected[0]);
    getMeasurement(free_params, &measurement[0]);

    residuals[0] = expected[0] - measurement[0];
    residuals[1] = expected[1] - measurement[1];
    residuals[2] = expected[2] - measurement[2];
    return true;  // always return true
  }
    
  /**
   *  \brief Get the expected position of the point, which simply the point
   *         itself, since measurement is in the root_frame_.
   */
  inline void getExpected(const double* const free_params,
                          const double* const point,
                          double* expected) const
  {
    expected[0] = point[0];
    expected[1] = point[1];
    expected[2] = point[2];
  }

  /**
   *  \brief Get the measurment, which is the position of led_frame_ in root_frame_,
   *         for a given set of chain correcting free parameters.
   */
  inline void getMeasurement(const double* const free_params,
                             double* measurement) const
  {
    KDL::Frame m = getChainFK(free_params);
    measurement[0] = m.p.x();
    measurement[1] = m.p.y();
    measurement[2] = m.p.z();
  }

  /** \brief Get the pose of the chain, given a set of parameters */
  inline KDL::Frame getChainFK(const double* const free_params) const
  {
    /* FK through chain. */
    KDL::Frame p_out = KDL::Frame::Identity();
    int joint = 0;
    for (int i = 0; i < chain_.getNrOfSegments(); ++i)
    {
      std::string name = chain_.getSegment(i).getJoint().getName();
      FrameCalibrationData d = (*info_)[name];

      /* Apply frame corrections, if any */
      if (d.calibrate)
      {
        KDL::Frame correction(KDL::Frame::Identity());
        if (free_params)
        {
          /* Create 6-dof correction from free_params */
          if (d.x > -1)
            correction.p.x(free_params[d.x-offset_]);
          if (d.y > -1)
            correction.p.y(free_params[d.y-offset_]);
          if (d.z > -1)
            correction.p.z(free_params[d.z-offset_]);

          /* Orientation is either 0, 1 or 3 parameters */
          if ( (d.roll > -1) &&
               (d.pitch > -1) &&
               (d.yaw > -1) )
          {
            /* These don't really correspond to rpy -- but call them that anyways */
            correction.M = rotation_from_axis_magnitude(free_params[d.roll-offset_],
                                                        free_params[d.pitch-offset_],
                                                        free_params[d.yaw-offset_]);
          }
          else if (d.roll > -1)
          {
            correction.M = rotation_from_axis_magnitude(free_params[d.roll-offset_],
                                                        0,
                                                        0);
          }
          else if (d.pitch > -1)
          {
            correction.M = rotation_from_axis_magnitude(0,
                                                        free_params[d.pitch-offset_],
                                                        0);
          }
          else if (d.yaw > -1)
          {
            correction.M = rotation_from_axis_magnitude(0,
                                                        0,
                                                        free_params[d.yaw-offset_]);
          }
        }
        p_out = p_out * correction;
      }

      if (chain_.getSegment(i).getJoint().getType() != KDL::Joint::None)
      {
        if (d.calibrate && free_params)
          p_out = p_out * chain_.getSegment(i).pose(positions_(joint) + static_cast<double>(free_params[d.idx-offset_]));
        else
          p_out = p_out * chain_.getSegment(i).pose(positions_(joint));
        ++joint;
      }
      else
      {
        p_out = p_out * chain_.getSegment(i).pose(0.0);
      }
    }
    return p_out;
  }

  /**
   *  \brief Helper factory function to create a chain error block. Parameters
   *         are described in the class constructor, which this function calls.
   */
  template <int num_free_params>
  static ceres::CostFunction* Create(KDL::Chain& chain,
                                     KDL::JntArray& positions,
                                     FrameCalibrationInfo* info,
                                     int offset,
                                     std::string root,
                                     std::string tip)
  {
    /*
     * This part seems poorly documented in Ceres docs. The 3 integers in the template
     * parameters to AutoDiff are the size of the data passed to the operator() function.
     * In the case of this ChainError block, the first parameter is the size of the residuals.
     * The second parameter is the size of the number of free_parameters, which for our chain
     * is the same size as the number of joints. The third parameter is the size of our point.
     */
    return ( new ceres::NumericDiffCostFunction<ChainError, ceres::CENTRAL, 3, num_free_params, 3>(
                 new ChainError(chain, positions, info, offset, root, tip)));
  }

  /** \brief Helper factory function to create a chain error block from existing ChainError. */
  template <int num_free_params>
  static ceres::CostFunction* Create(ChainError * chain_error)
  {
    return ( new ceres::NumericDiffCostFunction<ChainError, ceres::CENTRAL, 3, num_free_params, 3>(chain_error));
  }

  /* stored data */
  KDL::Chain chain_;
  KDL::JntArray positions_;
  FrameCalibrationInfo * info_;
  int offset_;
  std::string root_;
  std::string tip_;
};

#endif  // UBR_CALIBRATION_CHAIN_ERROR_H_
