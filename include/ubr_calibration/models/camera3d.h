/*
 * Copyright 2013-2014 Unbounded Robotics Inc.
 * Author: Michael Ferguson
 */

#ifndef UBR_CALIBRATION_MODELS_CAMERA3D_H_
#define UBR_CALIBRATION_MODELS_CAMERA3D_H_

#include <ubr_calibration/depth_camera.h>
#include <ubr_calibration/models/chain.h>

namespace ubr_calibration
{

/**
 *  \brief Model of a camera on a kinematic chain.
 */
class Camera3dModel : public ChainModel
{
public:
  /**
   *  \brief Create a new camera 3d model (Kinect/Primesense).
   *  \param model The KDL model of the robot's kinematics.
   *  \param root The name of the root link, must be consistent across all
   *         models used for error modeling. Usually 'base_link'.
   *  \param tip The tip of the chain.
   */
  Camera3dModel(KDL::Tree model, std::string root, std::string tip);
  virtual ~Camera3dModel() {}

  /**
   *  \brief Compute the updated positions of the observed points
   */
  virtual std::vector<geometry_msgs::PointStamped> project(
    const ubr_calibration::CalibrationData& data,
    const CalibrationOffsetParser& offsets);
};

}  // namespace ubr_calibration

#endif  // UBR_CALIBRATION_MODELS_CAMERA3D_H_
