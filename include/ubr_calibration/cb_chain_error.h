/*
 * Copyright 2013 Unbounded Robotics Inc.
 * Author: Michael Ferguson
 */

#ifndef UBR_CALIBRATION_CB_CHAIN_ERROR_H_
#define UBR_CALIBRATION_CB_CHAIN_ERROR_H_

#include <ubr_calibration/chain_functions.h>

#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

#include <ceres/ceres.h>

#include <string>

/**
 *  \brief Assumes a checkerboard at the end of a chain, compares measured
 *         pose of checkerboard to estimated pose.
 */
struct CbChainError
{
  /**
   *  \brief Residual block for a checkerboard in a gripper.
   *
   *  This uses a ChainError instance internally to handle the free_params
   *  for the joint offsets of the chain. It then applies a 6-dof
   *  gripper-to-checkerboard transform using free params as defined in info.
   *
   *  \param chain The KDL chain for the robot.
   *  \param positions The position of the joints when this measurement was taken.
   *  \param info The frame calibration info.
   *  \param offset The offset applied to this block, which must be subtracted from indexes in info.
   *  \param root The name of the root link (typically 'base_link')
   *  \param tip The name of the gripper link (typically 'gripper_link').
   */
  CbChainError(KDL::Chain& chain, KDL::JntArray& positions,
               FrameCalibrationInfo* info, int offset,
               std::string root, std::string tip)
    : chain_(chain, positions, info, offset, root, tip),
      info_(info), offset_(offset)
  {
    // TODO: gripper_cb should probably be set to tip.append('cb')
  }

  /**
   *  \brief Operator called by CERES optimizer.
   *  \param free_params The offsets to be applied to joints/transforms.
   *  \param pose The 6D pose estimate to be compared with observed value.
   *  \param residuals The residuals computed, to be returned to the optimizer.
   */
  bool operator()(const double* const free_params,
                  const double* const pose,
                  double* residuals) const
  {
    double expected[6];
    double measurement[6];

    getExpected(free_params, pose, &expected[0]);
    getMeasurement(free_params, &measurement[0]);

    for (int i = 0; i < 6; ++i)
      residuals[i] = expected[i] - measurement[i];
    return true;  // always return true
  }
    
  /**
   *  \brief Get the expected pose of the checkerboard given a 
   *         set of chain correcting free parameters.
   */
  inline void getExpected(const double* const free_params,
                          const double* const pose,
                          double* expected) const
  {
    /* Pose is already in root link */
    for (int i = 0; i < 6; ++i)
      expected[i] = pose[i];
  }

  /**
   *  \brief Get the measured pose of the checkerboard given a 
   *         set of chain correcting free parameters.
   */
  inline void getMeasurement(const double* const free_params,
                             double* measurement) const
  {
    /* Compute FK through the chain. */
    KDL::Frame p_out = chain_.getChainFK(free_params);

    /* Apply gripper link to checkerboard transform */
    KDL::Frame checkerboard(KDL::Frame::Identity());
    FrameCalibrationData d = (*info_)["gripper_cb"];
    /* Create 6-dof correction from free_params */
    checkerboard.p.x(free_params[d.x-offset_]);
    checkerboard.p.y(free_params[d.y-offset_]);
    checkerboard.p.z(free_params[d.z-offset_]);
    checkerboard.M = rotation_from_axis_magnitude(free_params[d.roll-offset_],
                                                  free_params[d.pitch-offset_],
                                                  free_params[d.yaw-offset_]);
    p_out = p_out * checkerboard;
    
    measurement[0] = p_out.p.x();
    measurement[1] = p_out.p.y();
    measurement[2] = p_out.p.z();
    axis_magnitude_from_rotation(p_out.M, measurement[3], measurement[4], measurement[5]);
  }

  inline KDL::Frame getChainFK(const double* const free_params) const
  {
    return chain_.getChainFK(free_params);
  }

  /**
   *  \brief Helper factory function to create a rgbd error block. Parameters
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
    /* See ChainError::Create for what these params are. */
    return ( new ceres::NumericDiffCostFunction<CbChainError, ceres::CENTRAL, 6, num_free_params, 6>(
                 new CbChainError(chain, positions, info, offset, root, tip)));
  }

  /** \brief Helper factory function to create an error block from existing CbChainError. */
  template <int num_free_params>
  static ceres::CostFunction* Create(CbChainError * error)
  {
    return ( new ceres::NumericDiffCostFunction<CbChainError, ceres::CENTRAL,
                                                6, /* residual size */
                                                num_free_params, /* free param size */
                                /* pose size */ 6>(error));
  }

  /* stored data */
  ChainError chain_;
  FrameCalibrationInfo * info_;
  int offset_;
};

#endif  // UBR_CALIBRATION_CB_CHAIN_ERROR_H_
