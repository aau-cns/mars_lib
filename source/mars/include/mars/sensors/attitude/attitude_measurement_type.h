// Copyright (C) 2021 Martin Scheiber, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <martin.scheiber@ieee.org>

#ifndef ATTITUDE_MEASUREMENT_TYPE_H
#define ATTITUDE_MEASUREMENT_TYPE_H

#include <mars/general_functions/utils.h>
#include <mars/sensors/attitude/attitude_conversion.h>

#include <eigen3/Eigen/Dense>
#include <iostream>
#include <utility>

namespace mars
{
class AttitudeMeasurementType
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  //  Eigen::Vector2d rp_vector_;  ///< Raw attitude measurement [roll pitch]
  Attitude attitude_;

  AttitudeMeasurementType() = default;

  //  AttitudeMeasurementType(Eigen::Vector2d rp_vector) : rp_vector_(std::move(rp_vector))
  //  {
  //  }

  // since we do not know what order the roll pitch rotation is, we expect a rotation matrix and extract this ourselves
  //  AttitudeMeasurementType(Eigen::Matrix3d rot_mat)
  //  {
  //    Eigen::Vector3d rpy = mars::Utils::RPYFromRotMat(rot_mat);
  //    rp_vector_ << rpy(0), rpy(1);

  //    // DEBUG
  //    std::cout << "[SensorType]: attitude with rpy [deg]: [" << rpy.transpose() * 180.0/M_PI << "]" << std::endl;
  //  }

  AttitudeMeasurementType(Eigen::Matrix3d rot_mat) : attitude_(rot_mat)
  {
  }
};
}  // namespace mars

#endif  // ATTITUDE_MEASUREMENT_TYPE_H
