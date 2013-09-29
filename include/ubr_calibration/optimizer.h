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
#include <ubr_calibration/ground_error.h>
#include <ubr_calibration/rgbd_error.h>
#include <ubr_calibration/chain_functions.h>

#include <boost/shared_ptr.hpp>
#include <string>
#include <map>

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
    if(!tree_.getChain(root_frame_, data[2].rgbd_observations[0].header.frame_id, camera_chain_))
    {
      std::cerr << "Failed to get camera chain" << std::endl;
      return -1;
    }

    /* observations = points = [X, Y, Z] * number of samples. */
    num_observations_ = 0;
    for (size_t i = 0; i < data.size(); ++i)
      num_observations_ += data[i].rgbd_observations.size();
    observations_ = new double[3 * num_observations_];
    points_ = new double[3 * num_observations_];

    /* free parameters = [arm joint angle offsets] + [camera joint angle offsets] + [free angles] */
    num_free_params_ = 0; //arm_chain_.getNrOfJoints() + camera_chain_.getNrOfJoints();

    /* TODO: in the future, parse this from a yaml or something */
    /* Although we set aside a free parameter for each arm/camera joint, we may not want to calibrate each */
    adjustments_["torso_lift_joint"] = FrameCalibrationData(num_free_params_++);
    adjustments_["shoulder_pan_joint"] = FrameCalibrationData(num_free_params_++);
    adjustments_["shoulder_lift_joint"] = FrameCalibrationData(num_free_params_++);
    adjustments_["upperarm_roll_joint"] = FrameCalibrationData(num_free_params_++);
    adjustments_["elbow_flex_joint"] = FrameCalibrationData(num_free_params_++);
    adjustments_["forearm_roll_joint"] = FrameCalibrationData(num_free_params_++);
    adjustments_["wrist_flex_joint"] = FrameCalibrationData(num_free_params_++);
    adjustments_["wrist_roll_joint"] = FrameCalibrationData(num_free_params_++);
    adjustments_["head_pan_joint"] = FrameCalibrationData(num_free_params_++);
    adjustments_["head_tilt_joint"] = FrameCalibrationData(num_free_params_++);
    adjustments_["head_camera_rgb_joint"] =
      FrameCalibrationData(num_free_params_++, num_free_params_++, num_free_params_++, -1, -1, num_free_params_++);
    adjustments_["head_camera_rgb_optical_joint"] = FrameCalibrationData(-1, -1, -1, -1, -1, num_free_params_++);

    /* disable torso lift and wrist roll */
    adjustments_["torso_lift_joint"].calibrate = true;
    adjustments_["wrist_roll_joint"].calibrate = false;

    /* free parameters = [arm joint angle offsets] + [camera joint angle offsets] + [free angles] = ALL 0 */
    free_params_ = new double[num_free_params_];
    for (int i = 0; i < num_free_params_; ++i)
      free_params_[i] = 0.0;

    /* Houston, we have a problem.. */
    problem_ = new ceres::Problem();

    /* For each observation: */
    int observation = 0;
    int ground_observations = 0;
    for (size_t i = 0; i < data.size(); ++i)
    {
      /* Get joint positions from message. */
      KDL::JntArray arm_positions = getChainPositionsFromMsg(arm_chain_, data[i].joint_states);
      KDL::JntArray camera_positions = getChainPositionsFromMsg(camera_chain_, data[i].joint_states);

      /* Add observations */
      for (size_t j = 0; j < data[i].rgbd_observations.size(); ++j)
      {
        if (progress_to_stdout)
          std::cout << "\nAdding observation:" << observation << std::endl;

        /* Load observations_ from point data in rgbd_observations */
        observations_[(3*observation)+0] = data[i].rgbd_observations[j].point.x;
        observations_[(3*observation)+1] = data[i].rgbd_observations[j].point.y;
        observations_[(3*observation)+2] = data[i].rgbd_observations[j].point.z;

        if (data[i].world_observations[j].header.frame_id == led_frame_)
        {
          /* Create an arm chain error block */
          ChainError * arm_error = new ChainError(arm_chain_, arm_positions, &adjustments_, 0, root_frame_, led_frame_);

          /* Point is a reprojection through arm */
          KDL::Frame projected = arm_error->getChainFK(0);
          points_[(3*observation)+0] = projected.p.x();
          points_[(3*observation)+1] = projected.p.y();
          points_[(3*observation)+2] = projected.p.z();

          /* Create arm residual */
          ceres::CostFunction * cost_function = ChainError::Create<8>(arm_error);
          problem_->AddResidualBlock(cost_function,
                                     NULL /* squared loss */,
                                     &free_params_[adjustments_["torso_lift_joint"].idx],
                                     &points_[(3*observation)]);

          if (progress_to_stdout)
            std::cout << "Initial estimate of point (via arm):   " << projected.p.x() <<
                         "," << projected.p.y() << "," << projected.p.z() << std::endl;
        }
        else
        {
          /* Currently ground is the only other option */
          ++ground_observations;

          /* Point is set directly from world observation */
          points_[(3*observation)+0] = data[i].world_observations[j].point.x;
          points_[(3*observation)+1] = data[i].world_observations[j].point.y;
          points_[(3*observation)+2] = data[i].world_observations[j].point.z;

          /* Create ground residual */
          ceres::CostFunction * cost_function = GroundError::Create(2.0);  // TODO set ground_error scale
          problem_->AddResidualBlock(cost_function,
                                     NULL /* squared loss */,
                                     &points_[(3*observation)]);

          if (progress_to_stdout)
            std::cout << "Initial estimate of point (ground):    " << points_[(3*observation)+0] <<
                         "," << points_[(3*observation)+1] << "," << points_[(3*observation)+2] << std::endl;
        }

        /* Create camera chain error block */
        RgbdError * camera_error = new RgbdError(camera_chain_, camera_positions, &adjustments_, 8,
                                                 root_frame_, data[i].rgbd_observations[j].header.frame_id,
                                                 observations_[(observation*3)+0],
                                                 observations_[(observation*3)+1],
                                                 observations_[(observation*3)+2]);

        /* Reproject camera error for comparison */
        {
          double reprojection[3];
          camera_error->getEstimatedGlobal(0, &reprojection[0]);

          if (progress_to_stdout)
            std::cout << "Intial estimate of point (via camera): " << reprojection[0] <<
                         "," << reprojection[1] << "," << reprojection[2] << std::endl;
        }

        /* Create camera residual */
        ceres::CostFunction * cost_function = RgbdError::Create<7>(camera_error);
        problem_->AddResidualBlock(cost_function,
                                   NULL /* squared loss */,
                                   &free_params_[adjustments_["head_pan_joint"].idx],
                                   &points_[(3*observation)]);

        ++observation;

        /* Note: the error blocks will be managed by scoped_ptr in cost functor
                 which takes ownership, and so we do not need to delete them here */
      }
    }

    /* Setup the actual optimization */
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = progress_to_stdout;

    std::cout << "\nSolver output:" << std::endl;
    summary_ = new ceres::Solver::Summary();
    ceres::Solve(options, problem_, summary_);

    /* For each observation: */
    if (progress_to_stdout)
    {
      observation = 0;

      for (size_t i = 0; i < data.size(); ++i)
      {
        /* Get joint positions from message. */
        KDL::JntArray arm_positions = getChainPositionsFromMsg(arm_chain_, data[i].joint_states);
        KDL::JntArray camera_positions = getChainPositionsFromMsg(camera_chain_, data[i].joint_states);

        /* Analyze Observation */
        for (size_t j = 0; j < data[i].rgbd_observations.size(); ++j)
        {
          std::cout << "\nAnalyzing observation: " << observation << std::endl;

          /* Create camera chain error block again */
          RgbdError * camera_error = new RgbdError(camera_chain_, camera_positions, &adjustments_, 7,
                                                   root_frame_, data[i].rgbd_observations[j].header.frame_id,
                                                   observations_[(observation*3)+0],
                                                   observations_[(observation*3)+1],
                                                   observations_[(observation*3)+2]);

          double measurement[3];
          double expected[3];

          camera_error->getMeasurement(0, &measurement[0]);
          std::cout << "Camera measurement:                   " << measurement[0] <<
                       "," << measurement[1] << "," << measurement[2] << std::endl;

          camera_error->getExpected(0, &points_[(3*observation)], &expected[0]);
          std::cout << "Reprojection estimate:                " << expected[0] <<
                       "," << expected[1] << "," << expected[2] << std::endl;

          /* Reproject camera */
          double cam_reprojection[3];
          camera_error->getEstimatedGlobal(&free_params_[7], &cam_reprojection[0]);
          std::cout << "Final estimate of point (via camera): " << cam_reprojection[0] <<
                       "," << cam_reprojection[1] << "," << cam_reprojection[2] << std::endl;

          if (observation < ground_observations)
          {
            std::cout << "  error (camera to ground):           " << cam_reprojection[2] << std::endl;
          }
          else
          {
            /* Create arm chain error block again */
            ChainError * arm_error = new ChainError(arm_chain_, arm_positions, &adjustments_, 0, root_frame_, led_frame_);

            /* Project through arm */
            KDL::Frame arm_reprojection;
            arm_reprojection = arm_error->getChainFK(&free_params_[0]);
            std::cout << "Final estimate of point (via arm):    " << arm_reprojection.p.x() <<
                         "," << arm_reprojection.p.y() << "," << arm_reprojection.p.z() << std::endl;

            double x = fabs(arm_reprojection.p.x() - cam_reprojection[0]);
            double y = fabs(arm_reprojection.p.y() - cam_reprojection[1]);
            double z = fabs(arm_reprojection.p.z() - cam_reprojection[2]);
            double t = sqrt((x * x) + (y * y) + (z * z));

            std::cout << "  error (x):                          " << x << std::endl;
            std::cout << "  error (y):                          " << y << std::endl;
            std::cout << "  error (z):                          " << z << std::endl;
            std::cout << "  error (total):                      " << t << std::endl;
          }
          ++observation;
        }
      }
    }

    return 0;
  }

  /** \brief Debugging information output. */
  void printResult()
  {
    /* Newline before results */
    std::cout << std::endl;

    /* Print final estimates of points */
    for (int i = 0; i < num_observations_; ++i)
    {
      std::cout << "Final estimate: " << points_[(3*i)+0] << "," <<
                                         points_[(3*i)+1] << "," <<
                                         points_[(3*i)+2] << std::endl;
    }

    /* Print final calibration updates */
    std::cout << "\nCalibrated Params" << std::endl;
    std::map<std::string, double> offsets = getCalibrationOffsets();
    for (std::map<std::string, double>::const_iterator it = offsets.begin();
         it != offsets.end(); ++it)
    {
      std::cout << it->first << ": " <<
                   it->second << std::endl;
    }

    std::cout << "\n" << summary_->BriefReport() << std::endl;
  }

  /**
   *  \brief Returns the calibration data to apply to the URDF.
   *         in the form of a map of joint_name:calibration_offset
   */
  std::map<std::string, double> getCalibrationOffsets()
  {
    std::map<std::string, double> offsets;
    for (FrameCalibrationInfoIterator it = adjustments_.begin();
         it != adjustments_.end();
         ++it)
    {
      FrameCalibrationData d = it->second;
      if (d.calibrate)
      {
        if (d.idx > -1)
        {
          /* This is a joint */
          offsets[it->first] = free_params_[d.idx];
        }
        if (d.x > -1)
        {
          std::string name = it->first;
          offsets[name.append("_x")] = free_params_[d.x];
        }
        if (d.y > -1)
        {
          std::string name = it->first;
          offsets[name.append("_y")] = free_params_[d.y];
        }
        if (d.z > -1)
        {
          std::string name = it->first;
          offsets[name.append("_z")] = free_params_[d.z];
        }
        /* Angles are either all 3, or just one */
        if ((d.roll > -1) && (d.pitch > -1) && (d.yaw > -1))
        {
          double roll, pitch, yaw;
          /* Again, the d.X values are not actually roll/pitch/yaw */
          KDL::Rotation rot = rotation_from_axis_magnitude(free_params_[d.roll],
                                                           free_params_[d.pitch],
                                                           free_params_[d.yaw]);
          /* Get roll, pitch, yaw about fixed axis */
          rot.GetRPY(roll, pitch, yaw);
          /* Fill in offsets */
          std::string name = it->first;
          offsets[name.append("_roll")] = roll;
          name = it->first;
          offsets[name.append("_pitch")] = pitch;
          name = it->first;
          offsets[name.append("_yaw")] = yaw;
        }
        else if (d.roll > -1)
        {
          std::string name = it->first;
          offsets[name.append("_roll")] = free_params_[d.roll];
        }
        else if (d.pitch > -1)
        {
          std::string name = it->first;
          offsets[name.append("_pitch")] = free_params_[d.pitch];
        }
        else if (d.yaw > -1)
        {
          std::string name = it->first;
          offsets[name.append("_yaw")] = free_params_[d.yaw];
        }
      }
    }
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
  FrameCalibrationInfo adjustments_;

  ceres::Problem* problem_;
  ceres::Solver::Summary* summary_;
};

#endif  // UBR_CALIBRATION_OPTIMIZER_H_
