/*
 * Copyright 2013 Unbounded Robotics Inc.
 * Author: Michael Ferguson
 */

#ifndef UBR_CALIBRATION_RGBD_ERROR_H_
#define UBR_CALIBRATION_RGBD_ERROR_H_

#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

#include <ceres/ceres.h>

#include <string>

/** \brief Used to find error in reprojection using a chain of joints.
 *
 *  Currently, the only free parameters supported are the joint angle offsets
 *  of each joint in the chain.
 */
struct RgbdError
{
  /** \brief Standard construct for a residual block based on a camera attached to a chain.
   *
   *  This uses a ChainError instance internally to handle the free_params for the joint offsets
   *  of the chain. It then applies a 6-dof correction transform for the sensor frame using the
   *  last 6 free_params, which should be X, Y, Z, and an angle-axis rotation (where the angle is
   *  given by the combined magnitude of the 3 parameters).
   *
   *  \param chain The KDL chain for the robot.
   *  \param root The name of the root link (typically 'base_link')
   *  \param tip The name of the sensor frame.
   *  \param positions The position of the joints when this measurement was taken.
   *  \param observed_x The observed x coordinate of the point, when chain is the arm, this is 0.
   *  \param observed_y The observed y coordinate of the point, when chain is the arm, this is 0.
   *  \param observed_z The observed z coordinate of the point, when chain is the arm, this is 0.
   */
  RgbdError(KDL::Chain& chain, KDL::JntArray& positions, std::string root, std::string tip,
             double observed_x, double observed_y, double observed_z)
    : chain_(chain, positions, root, tip)
  {
    observation_[0] = observed_x;
    observation_[1] = observed_y;
    observation_[2] = observed_z;
  }

  /** \brief Operator called by CERES optimizer.
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
   *  \brief Get the expected position of the point in the camera_frame_
   *         for a given set of chain correcting free parameters.
   */
  inline void getExpected(const double* const free_params,
                          const double* const point,
                          double* expected) const
  {
    /* Turn point into a transform (in the root_frame_). */
    KDL::Frame point_ = KDL::Frame::Identity();
    point_.p.x(point[0]);
    point_.p.y(point[1]);
    point_.p.z(point[2]);

    /* Compute FK through the chain. */
    KDL::Frame p_out = chain_.getChainFK(free_params);

    /* Allow this to run without free_params being defined. */
    if (free_params)
    {
      int param_idx = chain_.getNumOfParams();

      /* Create camera 6-dof correction from free_params */
      KDL::Frame camera_correction(KDL::Frame::Identity());
      camera_correction.p.x(free_params[param_idx]);
      camera_correction.p.y(free_params[param_idx+1]);
      camera_correction.p.z(free_params[param_idx+2]);
      /* TODO: is this correct? do we have to normalize params. */
      camera_correction.M = KDL::Rotation::Quaternion(free_params[param_idx+3],
                                                      free_params[param_idx+4],
                                                      free_params[param_idx+5],
                                                      1.0);
      p_out = p_out*camera_correction;
    }

    /* Transform point_ from root_frame_ into camera_frame_ and return it. */
    point_ = p_out.Inverse() * point_;
    expected[0] = point_.p.x();
    expected[1] = point_.p.y();
    expected[2] = point_.p.z();
  }

  /**
   *  \brief Get the measured position of the point from the RGBD sensor.
   */
  inline void getMeasurement(const double* const free_params,
                             double* measurement) const
  {
    /* Observation is already in camera_frame_ */
    measurement[0] = observation_[0];
    measurement[1] = observation_[1];
    measurement[2] = observation_[2];
  }

  /** \brief Helper factory function to create a rgbd error block.
   *  Parameters are described in the class constructor, which this function calls.
   */
  template <int num_free_params>
  static ceres::CostFunction* Create(KDL::Chain& chain,
                                     KDL::JntArray& positions,
                                     std::string root,
                                     std::string tip,
                                     double x,
                                     double y,
                                     double z)
  {
    /* See ChainError::Create for what these params are. */
    return ( new ceres::NumericDiffCostFunction<RgbdError, ceres::CENTRAL, 3, num_free_params, 3>(
                 new RgbdError(chain, positions, root, tip, x, y, z)));
  }

  /** \brief Helper factory function to create a rgbd error block from existing RgbdError. */
  template <int num_free_params>
  static ceres::CostFunction* Create(RgbdError * rgbd_error)
  {
    return ( new ceres::NumericDiffCostFunction<RgbdError, ceres::CENTRAL, 3, num_free_params, 3>(rgbd_error));
  }

  /* stored data */
  ChainError chain_;
  double observation_[3];
};

#endif  // UBR_CALIBRATION_RGBD_ERROR_H_
