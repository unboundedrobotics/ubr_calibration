/*
 * Copyright 2013 Unbounded Robotics Inc.
 * Author: Michael Ferguson
 */

#ifndef UBR_CALIBRATION_CB_RGBD_ERROR_H_
#define UBR_CALIBRATION_CB_RGBD_ERROR_H_

#include <ubr_calibration/chain_functions.h>

#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

#include <ceres/ceres.h>

#include <string>

/**
 *  \brief Assumes a checkerboard seen by an rgbd camera. The camera measured
 *         the points, and we estimate the points based on the estimated pose
 *         of the checkerboard.
 */
struct CbRgbdError
{
  /**
   *  \brief Residual block for a checkerboard seen by an rgbd camera.
   *
   *  This uses a ChainError instance internally to handle the free_params
   *  for the joint offsets of the chain. 
   *
   *  \param chain The KDL chain for the robot.
   *  \param positions The position of the joints when this measurement was taken.
   *  \param info The frame calibration info.
   *  \param offset The offset applied to this block, which must be subtracted from indexes in info.
   *  \param root The name of the root link (typically 'base_link')
   *  \param tip The name of the sensor frame.
   *  \param positions The position of the joints when this measurement was taken.
   *  \param observations The point observations by the camera
   *  \param size The size of a checkerboard point.
   */
  CbRgbdError(KDL::Chain& chain, KDL::JntArray& positions,
              FrameCalibrationInfo* info, int offset,
              std::string root, std::string tip,
              std::vector<double>& observations,
              double size, int x)
    : chain_(chain, positions, info, offset, root, tip)
  {
    observations_ = observations;
    size_ = size;
    x_ = x;
  }

  /**
   *  \brief Operator called by CERES optimizer.
   *  \param free_params The offsets to be applied to joints/transforms.
   *  \param pose The 6D pose estimate to be compared with observed points.
   *  \param residuals The residuals computed, to be returned to the optimizer.
   */
  bool operator()(const double* const free_params,
                  const double* const pose,
                  double* residuals) const
  {
    double expected[observations_.size()];
    double measurement[observations_.size()];

    getExpected(free_params, pose, &expected[0]);
    getMeasurement(free_params, &measurement[0]);

    residuals[0] = 0.0;
    residuals[1] = 0.0;
    residuals[2] = 0.0;

    for (size_t i = 0; i < (observations_.size()/3); ++i)
    {
      /* We generate residuals for x/y/z */
      residuals[0] += fabs(expected[(3*i)+0] - measurement[(3*i)+0]);
      residuals[1] += fabs(expected[(3*i)+1] - measurement[(3*i)+1]);
      residuals[2] += fabs(expected[(3*i)+2] - measurement[(3*i)+2]);
    }

    return true;  // always return true
  }

  /**
   *  \brief Get the expected position of the points in the camera_frame_
   *         for a given pose and set of chain correcting free parameters.
   */
  inline void getExpected(const double* const free_params,
                          const double* const pose,
                          double* expected) const
  {
    /* Compute FK from root to camera through the chain. */
    KDL::Frame p_out = chain_.getChainFK(free_params);

    /* Build up the transform from root to checkerboard */
    KDL::Frame checkerboard(KDL::Frame::Identity());
    checkerboard.p.x(pose[0]);
    checkerboard.p.y(pose[1]);
    checkerboard.p.z(pose[2]);
    checkerboard.M = rotation_from_axis_magnitude(pose[3], pose[4], pose[5]);

    /* Project points from checkerboard frame to camera frame */
    int i = 0;
    for (size_t y = 0; y < observations_.size()/(3*x_); ++y)
    {
      for (size_t x = 0; x < x_; ++x)
      {
        /* Point in the checkerboard frame */
        KDL::Frame point_ = KDL::Frame::Identity();
        point_.p.x(x * size_);
        point_.p.y(0.0);
        point_.p.z(y * size_);

        /* Transform point to camera frame */
        point_ = p_out.Inverse() * checkerboard * point_;

        /* Fill in expected */
        expected[(3*i)+0] = point_.p.x();
        expected[(3*i)+1] = point_.p.y();
        expected[(3*i)+2] = point_.p.z();
        ++i;
      }
    }
  }

  /**
   *  \brief Get the measured points from the RGBD sensor.
   */
  inline void getMeasurement(const double* const free_params,
                             double* measurement) const
  {
    /* Observations are already in camera_frame_ */
    for (size_t i = 0; i < (observations_.size()/3); ++i)
    {
      measurement[(3*i)+0] = observations_[(3*i)+0];  // x
      measurement[(3*i)+1] = observations_[(3*i)+1];  // y
      measurement[(3*i)+2] = observations_[(3*i)+2];  // z
    }
  }

  /**
   *  \brief Used in analysis at end of program, find the position of
   *         the observed point in the root frame.
   */
  inline bool getEstimatedGlobal(const double* const free_params,
                                 double* estimate,
                                 int observation = 0)
  {
    if ((3*observation)+2 >= observations_.size())
      return false; 
    KDL::Frame point_ = KDL::Frame::Identity();
    point_.p.x(observations_[(3*observation)+0]);
    point_.p.y(observations_[(3*observation)+1]);
    point_.p.z(observations_[(3*observation)+2]);

    /* Compute FK through the chain. */
    KDL::Frame p_out = chain_.getChainFK(free_params);

    /* Transform point_ from camera_frame_ into root_frame_ and return it. */
    point_ = p_out * point_;
    estimate[0] = point_.p.x();
    estimate[1] = point_.p.y();
    estimate[2] = point_.p.z();
    return true;
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
                                     std::string tip,
                                     std::vector<double>& observations,
                                     double size,
                                     int x)
  {
    /* See ChainError::Create for what these params are. */
    return ( new ceres::NumericDiffCostFunction<CbRgbdError, ceres::CENTRAL, 3, num_free_params, 6>(
                 new CbRgbdError(chain, positions, info, offset, root, tip, observations, size, x)));
  }

  /** \brief Helper factory function to create a rgbd error block from existing CbRgbdError. */
  template <int num_free_params>
  static ceres::CostFunction* Create(CbRgbdError * error)
  {
    return ( new ceres::NumericDiffCostFunction<CbRgbdError, ceres::CENTRAL, 3, num_free_params, 6>(error));
  }

  /* stored data */
  ChainError chain_;
  std::vector<double> observations_;
  double size_;
  int x_;
};

#endif  // UBR_CALIBRATION_CB_RGBD_ERROR_H_
