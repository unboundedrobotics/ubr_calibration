/*
 * Copyright 2013 Unbounded Robotics Inc.
 * Author: Michael Ferguson
 */

#ifndef UBR_CALIBRATION_UPDATE_URDF_H_
#define UBR_CALIBRATION_UPDATE_URDF_H_

#include <string>
#include <map>
#include <tinyxml.h>

/**
 *  \brief Update the urdf with new calibration offsets.
 *  \param urdf A string holding the URDF file contents that are to be updated.
 *  \param offsets Offsets to apply to the URDF.
 */
std::string updateURDF(const std::string &urdf, std::map<std::string, double> &offsets);

#endif  // UBR_CALIBRATION_UPDATE_URDF_H_
