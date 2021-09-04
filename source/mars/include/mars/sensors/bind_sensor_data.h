// Copyright (C) 2021 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <christian.brommer@ieee.org>

#ifndef BASESENSORDATA_H
#define BASESENSORDATA_H

#include <mars/type_definitions/base_states.h>
#include <mars/type_definitions/core_state_type.h>
#include <Eigen/Dense>

namespace mars
{
template <typename T>
///
/// \brief The BaseSensorData class binds the sensor state and covariance matrix.
///
/// The state is instanciated with the passed template (sensor state type)
/// The sensor state class needs to define the error state.
/// The BaseSensorData class initializes the covariance matrix based on this value.
///
class BindSensorData
{
  static_assert(std::is_base_of<BaseStates, T>::value, "Type T must inherit from Class BaseStates");

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  T state_;
  int full_cov_size_;                      ///< size of the full covariance
  Eigen::MatrixXd sensor_cov_;             ///< covariance of the sensor states
  Eigen::MatrixXd core_sensor_cross_cov_;  ///< cross-correlation between sensor states and the core

  BindSensorData()
  {
    core_sensor_cross_cov_ = Eigen::MatrixXd::Zero(CoreStateType::size_error_, state_.cov_size_);
    sensor_cov_ = Eigen::MatrixXd::Zero(state_.cov_size_, state_.cov_size_);
    full_cov_size_ = CoreStateType::size_error_ + state_.cov_size_;
  }

  ///
  /// \brief set_cov Takes a full covariance and separates sensor covariance and sensor-core cross-correlation
  /// \param cov
  ///
  void set_cov(const Eigen::MatrixXd& cov)
  {
    // TODO allow this with changing sensor state sizes
    sensor_cov_ = cov.block(CoreStateType::size_error_, CoreStateType::size_error_, state_.cov_size_, state_.cov_size_);
    core_sensor_cross_cov_ = cov.block(0, CoreStateType::size_error_, CoreStateType::size_error_, state_.cov_size_);
  }

  ///
  /// \brief get_full_cov builds the full covariance matrix
  ///
  /// \return sensor covariance and cross covariance, Core entrys are zero.
  ///
  Eigen::MatrixXd get_full_cov() const
  {
    // TODO allow this with changing sensor state sizes
    Eigen::MatrixXd full_cov;
    full_cov.resize(full_cov_size_, full_cov_size_);

    // Set core elements to zero
    full_cov.block(0, 0, CoreStateType::size_error_, CoreStateType::size_error_) =
        Eigen::MatrixXd::Zero(CoreStateType::size_error_, CoreStateType::size_error_);

    // Fill sensor covariance
    full_cov.block(CoreStateType::size_error_, CoreStateType::size_error_, state_.cov_size_, state_.cov_size_) =
        sensor_cov_;

    // Fill cross covariance
    full_cov.block(0, CoreStateType::size_error_, CoreStateType::size_error_, state_.cov_size_) =
        core_sensor_cross_cov_;
    full_cov.block(CoreStateType::size_error_, 0, state_.cov_size_, CoreStateType::size_error_) =
        core_sensor_cross_cov_.transpose();

    return full_cov;
  }
};
}

#endif  // BASESENSORDATA_H
