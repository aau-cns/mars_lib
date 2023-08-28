// Copyright (C) 2021 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <christian.brommer@ieee.org>

#ifndef UPDATE_SENSOR_ABS_CLASS_H
#define UPDATE_SENSOR_ABS_CLASS_H

#include <mars/core_state.h>
#include <mars/ekf.h>
#include <mars/sensors/sensor_abs_class.h>
#include <mars/sensors/sensor_interface.h>
#include <mars/type_definitions/base_states.h>
#include <Eigen/Dense>

namespace mars
{
class UpdateSensorAbsClass : public SensorAbsClass
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  int aux_states_;
  int aux_error_states_;
  int ref_to_nav_;
  Eigen::MatrixXd residual_;
  Eigen::VectorXd R_;  ///< Measurement noise "squared"
  Eigen::MatrixXd F_;
  Eigen::MatrixXd H_;
  Eigen::MatrixXd Q_;

  std::shared_ptr<void> initial_calib_{ nullptr };
  bool initial_calib_provided_{ false };  ///< True if an initial calibration was provided
  bool const_ref_to_nav_{ true };         ///< True if the reference should not be estimated
  bool use_dynamic_meas_noise_{ false };  ///< True if dynamic noise values from measurements should be used

  Chi2 chi2_;

  std::shared_ptr<CoreState> core_states_;
};
}  // namespace mars

#endif  // UPDATE_SENSOR_ABS_CLASS_H
