/*
 * Copyright 2013 Unbounded Robotics Inc.
 * Author: Michael Ferguson
 */

#ifndef UBR_CALIBRATION_GROUND_ERROR_H_
#define UBR_CALIBRATION_GROUND_ERROR_H_

#include <ubr_calibration/chain_functions.h>
#include <ceres/ceres.h>
#include <string>

/**
 *  \brief Used to find error in reprojection of points on the ground.
 */
struct GroundError
{
  /**
   *  \brief Standard construct for a residual block of a point in the ground.
   *  \param cost The cost to apply to the Z error.
   */
  GroundError(double cost) : cost_(cost)
  {
  }

  /**
   *  \brief Operator called by CERES optimizer.
   *  \param free_params The joint angle offsets to be applied to each joint.
   *  \param residuals The residuals computed, to be returned to the optimizer.
   */
  bool operator()(const double* const point,
                  double* residuals) const
  {
    residuals[0] = cost_ * point[2] * point[2];
    return true;
  }

  /**
   *  \brief Helper factory function to create a ground error block. Parameters
   *         are described in the class constructor, which this function calls.
   */
  static ceres::CostFunction* Create(double cost)
  {
    /*
     * This part seems poorly documented in Ceres docs. The 3 integers in the template
     * parameters to AutoDiff are the size of the data passed to the operator() function.
     * In the case of this ChainError block, the first parameter is the size of the residuals.
     * The second parameter is the size of the number of free_parameters, which for our chain
     * is the same size as the number of joints. The third parameter is the size of our point.
     */
    return ( new ceres::NumericDiffCostFunction<GroundError, ceres::CENTRAL, 1, 3>(
                 new GroundError(cost)));
  }

  /* stored data */
  double cost_;
};

#endif  // UBR_CALIBRATION_GROUND_ERROR_H_
