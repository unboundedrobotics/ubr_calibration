/*
 * Copyright 2013 Unbounded Robotics Inc.
 * Author: Michael Ferguson
 */

#ifndef UBR_CALIBRATION_CHAIN_ERROR_H_
#define UBR_CALIBRATION_CHAIN_ERROR_H_

#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

#include <ceres/ceres.h>

#include <string>

/**
 *  \brief Used to find error in reprojection using a chain of joints.
 *
 *  Currently, the only free parameters supported are the joint angle offsets
 *  of each joint in the chain.
 */
struct ChainError
{
  /**
   *  \brief Standard construct for a residual block based on projection through a chain.
   *  \param chain The KDL chain for the robot.
   *  \param positions The position of the joints when this measurement was taken.
   *  \param root The name of the root link (typically 'base_link')
   *  \param tip The name of the tip of the chain to be used (typically the camera sensor frame
   *         or gripper LED frame).
   */
  ChainError(KDL::Chain& chain, KDL::JntArray& positions, std::string root, std::string tip) : 
    chain_(chain), positions_(positions), root_(root), tip_(tip)
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
    /* Create a temporary JntArray, update it with the joint offsets from free_params. */
    int chain_len = chain_.getNrOfJoints();
    KDL::JntArray pos(chain_len);

    /* Allow this to run without free_params being defined */
    if (free_params)
      for (int i = 0; i < chain_len; ++i)
        pos(i) = positions_(i) + static_cast<double>(free_params[i]);
    else
      for (int i = 0; i < chain_len; ++i)
        pos(i) = positions_(i);

    /* FK through chain. */
    KDL::ChainFkSolverPos_recursive fk(chain_);
    KDL::Frame p_out;
    if (fk.JntToCart(pos, p_out) < 0)
    {
      // TODO: how to handle error?
    }

    return p_out;
  }

  /** \brief Get the number of parameters needed by this chain. */
  int getNumOfParams() const
  {
    // TODO: this will change when we do more complex parameter block manipulation.
    return chain_.getNrOfJoints();
  }

  /**
   *  \brief Helper factory function to create a chain error block. Parameters
   *         are described in the class constructor, which this function calls.
   */
  template <int num_free_params>
  static ceres::CostFunction* Create(KDL::Chain& chain,
                                     KDL::JntArray& positions,
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
                 new ChainError(chain, positions, root, tip)));
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
  std::string root_;
  std::string tip_;
};

#endif  // UBR_CALIBRATION_CHAIN_ERROR_H_
