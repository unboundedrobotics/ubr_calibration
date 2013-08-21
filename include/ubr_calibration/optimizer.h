/*
 * Copyright 2013 Unbounded Robotics Inc.
 * Author: Michael Ferguson
 */

#ifndef UBR_CALIBRATION_OPTIMIZER_H_
#define UBR_CALIBRATION_OPTIMIZER_H_

#include <ceres/ceres.h>

#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <ubr_calibration/CalibrationData.h>

#include <ubr_calibration/chain_error.h>
#include <ubr_calibration/rgbd_error.h>
#include <ubr_calibration/chain_functions.h>

/** \brief Class to do optimization. */
class Optimizer
{
public:
  /** \brief Standard constructor */
  Optimizer(std::string robot_description, std::string root_frame, std::string led_frame) :
    root_frame_(root_frame), led_frame_(led_frame)
  {
    if (!model_.initString(robot_description))
      std::cerr << "Failed to parse URDF." << std::endl;
  }

  virtual ~Optimizer()
  {
    if (observations_)
      delete[] observations_;
    if (points_)
      delete[] points_;
    if (free_params_)
      delete[] free_params_;
    if (problem_)
      delete problem_;
  }

  /** \brief Run optimization.
   *  \param data The data to be used for the optimization. Typically parsed
   *         from bag file, or loaded over some topic subscriber.
   *  \param progress_to_stdout If true, Ceres optimizer will output info to
   *         stdout.
   */
  int optimize(std::vector<ubr_calibration::CalibrationData> data,
               bool progress_to_stdout = false)
  {
    /* Load KDL from urdf */
    if (!kdl_parser::treeFromUrdfModel(model_, tree_))
    {
      std::cerr << "Failed to construct KDL tree" << std::endl;
      return -1;
    }

    /* Populate the chains */
    if(!tree_.getChain(root_frame_, led_frame_, arm_chain_))
    {
      std::cerr << "Failed to get arm chain" << std::endl;
      return -1;
    }
    if(!tree_.getChain(root_frame_, data[0].rgbd_observations[0].header.frame_id, camera_chain_))
    {
      std::cerr << "Failed to get camera chain" << std::endl;
      return -1;
    }

    /* observations = points = [X, Y, Z] * number of samples. */
    num_observations_ = data.size();
    observations_ = new double[3 * num_observations_];
    points_ = new double[3 * num_observations_];

    /* free parameters = [arm joint angle offsets] + [camera joint angle offsets] + [camera pose offset] = ALL 0 */
    num_free_params_ = arm_chain_.getNrOfJoints() + camera_chain_.getNrOfJoints() + 6;
    free_params_ = new double[num_free_params_];
    for (int i = 0; i < num_free_params_; ++i)
      free_params_[i] = 0.0;

    /* Houston, we have a problem.. */
    problem_ = new ceres::Problem();

    /* For each observation: */
    for (size_t i = 0; i < data.size(); ++i)
    {
      if (progress_to_stdout)
        std::cout << "Adding observation:" << i << std::endl;

      /* Get joint positions from message. */
      KDL::JntArray arm_positions = getChainPositionsFromMsg(arm_chain_, data[i].joint_states);
      KDL::JntArray camera_positions = getChainPositionsFromMsg(camera_chain_, data[i].joint_states);

      /* Load observations_[i] from point data in rgbd_observations */
      observations_[(3*i)+0] = data[i].rgbd_observations[0].point.x;
      observations_[(3*i)+1] = data[i].rgbd_observations[0].point.y;
      observations_[(3*i)+2] = data[i].rgbd_observations[0].point.z;

      /* Create arm chain error block, project through arm to get initial points[i] */
      ChainError * arm_error = new ChainError(arm_chain_, arm_positions, root_frame_, led_frame_);
      KDL::Frame projected = arm_error->getChainFK(0);
      points_[(3*i)+0] = projected.p.x();
      points_[(3*i)+1] = projected.p.y();
      points_[(3*i)+2] = projected.p.z();
      if (progress_to_stdout)
        std::cout << "Initial estimate of point (via arm): " << projected.p.x() <<
                     "," << projected.p.y() << "," << projected.p.z() << std::endl;

      /* Create camera chain error block, project through to do a sanity check on reprojection */
      RgbdError * camera_error = new RgbdError(camera_chain_, camera_positions,
                                               root_frame_, data[i].rgbd_observations[0].header.frame_id,
                                               observations_[(i*3)+0],
                                               observations_[(i*3)+1],
                                               observations_[(i*3)+2]);
      double reprojection[3];
      camera_error->getMeasurement(0, &reprojection[0]);
      if (progress_to_stdout)
        std::cout << "Camera measurement:   " << reprojection[0] <<
                     "," << reprojection[1] << "," << reprojection[2] << std::endl;

      camera_error->getExpected(0, &points_[(3*i)], &reprojection[0]);
      if (progress_to_stdout)
        std::cout << "Reprojected estimate (via camera):   " << reprojection[0] <<
                     "," << reprojection[1] << "," << reprojection[2] << std::endl;

      /* Create arm residual */
      ceres::CostFunction* cost_function = ChainError::Create<7>(arm_error);
      problem_->AddResidualBlock(cost_function,
                                 NULL /* squared loss */,
                                 &free_params_[0],
                                 &points_[(3*i)]);

      /* Create camera residual */
      cost_function = RgbdError::Create<8>(camera_error);
      problem_->AddResidualBlock(cost_function,
                                 NULL /* squared loss */,
                                 &free_params_[arm_chain_.getNrOfJoints()],
                                 &points_[(3*i)]);

      /* Note: arm/camera error blocks will be managed by scoped_ptr in cost functor
               which takes ownership, and so we do not need to delete them here */
    }

    /* Setup the actual optimization */
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = progress_to_stdout;

    summary_ = new ceres::Solver::Summary();
    ceres::Solve(options, problem_, summary_);
    return 0;
  }

  /** \brief Debugging information output. */
  void printResult()
  {
    /* Print final estimates of points */
    for (int i = 0; i < num_observations_; ++i)
    {
      std::cout << "Final estimate: " << points_[(3*i)+0] << "," << 
                                         points_[(3*i)+1] << "," << 
                                         points_[(3*i)+2] << std::endl;
    }

    /* Print final calibration updates */
    std::cout << "Calibrated Params" << std::endl;
    std::map<std::string, double> offsets = getCalibrationOffsets();
    for (std::map<std::string, double>::const_iterator it = offsets.begin();
         it != offsets.end(); ++it)
    {
      std::cout << it->first << ": " <<
                   it->second << std::endl;
    }
  }

  /**
   *  \brief Returns the calibration data to apply to the URDF.
   *         in the form of a map of joint_name:calibration_offset
   */
  std::map<std::string, double> getCalibrationOffsets()
  {
    std::map<std::string, double> offsets;
    int p = 0;
    for (int i = 0; i < arm_chain_.getNrOfSegments(); ++i)
    {
      if (arm_chain_.getSegment(i).getJoint().getType()!=KDL::Joint::None)
      {
        offsets[arm_chain_.getSegment(i).getJoint().getName()] = free_params_[p];
        ++p;
      }
    }
    for (int i = 0; i < camera_chain_.getNrOfSegments(); ++i)
    {
      if (camera_chain_.getSegment(i).getJoint().getType()!=KDL::Joint::None)
      {
        offsets[camera_chain_.getSegment(i).getJoint().getName()] = free_params_[p];
        ++p;
      }
    }
    // TODO: make this generic
    offsets["head_camera_frame_x"] = free_params_[p++];
    offsets["head_camera_frame_y"] = free_params_[p++];
    offsets["head_camera_frame_z"] = free_params_[p++];
    offsets["head_camera_frame_rot_r"] = free_params_[p++];
    offsets["head_camera_frame_rot_p"] = free_params_[p++];
    offsets["head_camera_frame_rot_y"] = free_params_[p++];
    return offsets;
  }

  /** \brief Returns the summary of the optimization last run. */
  ceres::Solver::Summary* summary()
  {
    return summary_;
  }

private:
  urdf::Model model_;
  std::string root_frame_;
  std::string led_frame_;

  KDL::Tree tree_;
  KDL::Chain arm_chain_;
  KDL::Chain camera_chain_;

  int num_observations_;
  int num_free_params_;
  double * observations_;
  double * points_;
  double * free_params_;

  ceres::Problem* problem_;
  ceres::Solver::Summary* summary_;
};

#endif  // UBR_CALIBRATION_OPTIMIZER_H_
