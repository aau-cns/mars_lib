// Copyright (C) 2021 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <christian.brommer@ieee.org>

#ifndef CORESTATE_H
#define CORESTATE_H

#include <mars/sensors/imu/imu_measurement_type.h>
#include <mars/sensors/sensor_abs_class.h>
#include <mars/time.h>
#include <mars/type_definitions/core_state_type.h>
#include <mars/type_definitions/core_type.h>
#include <Eigen/Dense>

namespace mars
{
class CoreState
{
private:
  bool fixed_acc_bias_{ false };   ///< bias are not estimated if fixed_bias = true
  bool fixed_gyro_bias_{ false };  ///< bias are not estimated if fixed_bias = true

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  CoreStateType state;  ///< Core State elements

  // Noise (STD)
  Eigen::Vector3d n_a_{ Eigen::Vector3d::Zero() };   ///< noise for linear acceleration measurement
  Eigen::Vector3d n_ba_{ Eigen::Vector3d::Zero() };  ///< random walk for linear acceleration bias
  Eigen::Vector3d n_w_{ Eigen::Vector3d::Zero() };   ///< noise for angular velocity measurement
  Eigen::Vector3d n_bw_{ Eigen::Vector3d::Zero() };  ///< random walk for angular velocity bias

  const Eigen::Vector3d g_{ 0, 0, 9.81 };  ///<defined gravity

  CoreStateMatrix initial_covariance_;

  std::shared_ptr<SensorAbsClass> propagation_sensor_{ nullptr };  ///< Reference to the propagation sensor

  bool test_state_transition_{ false };  ///< If true, the class performs tests on the state-transition properties
  bool verbose_{ false };                ///< increased output of information

  ///
  /// \brief CoreState Default constructor
  ///
  CoreState();

  ///
  /// \brief set_fixed_acc_bias disable or enable the estimation of the accelerometer bias
  /// \param value
  ///
  void set_fixed_acc_bias(const bool& value);

  ///
  /// \brief set_fixed_gyro_bias disable or enable the estimation of the gyro bias
  /// \param value
  ///
  void set_fixed_gyro_bias(const bool& value);

  ///
  /// \brief set_propagation_sensor Stores a reference to the propagation sensor
  /// \param propagation_sensor
  ///
  void set_propagation_sensor(std::shared_ptr<SensorAbsClass> propagation_sensor);

  ///
  /// \brief set_noise_std Sets the noise and bias of the propagation sensor
  /// \param n_w
  /// \param n_bw
  /// \param n_a
  /// \param n_ba
  ///
  void set_noise_std(const Eigen::Vector3d& n_w, const Eigen::Vector3d& n_bw, const Eigen::Vector3d& n_a,
                     const Eigen::Vector3d& n_ba);

  ///
  /// \brief InitializeState Initializes the core state
  /// \param ang_vel
  /// \param lin_acc
  /// \param p_wi
  /// \param v_wi
  /// \param q_wi
  /// \param b_w
  /// \param b_a
  /// \return
  ///
  CoreStateType InitializeState(const Eigen::Vector3d& ang_vel, const Eigen::Vector3d& lin_acc,
                                const Eigen::Vector3d& p_wi, const Eigen::Vector3d& v_wi,
                                const Eigen::Quaterniond& q_wi, const Eigen::Vector3d& b_w, const Eigen::Vector3d& b_a);

  ///
  /// \brief set_initial_covariance used to set the initial covariance of the core states
  /// \return
  ///
  void set_initial_covariance(const Eigen::Vector3d& p, const Eigen::Vector3d& v, const Eigen::Vector3d& q,
                              const Eigen::Vector3d& bw, const Eigen::Vector3d& ba);

  ///
  /// \brief InitializeCovariance Returnes the initialized core covariance
  /// \return
  ///
  CoreStateMatrix InitializeCovariance();

  ///
  /// \brief PropagateState Performs the state propagation
  /// \param prior_core_state
  /// \param measurement System input
  /// \param dt propagation timespan
  /// \return Propagated core state
  ///
  ///   \note Previous state is x_t-1 , Current state is x_t
  ///   ew and ea are the estimates of the inputs because the bias
  ///   are removed and the error state definition
  ///   \note first order Quaternion integration Sola, J. (2017).
  ///   Quaternion kinematics for the error-state Kalman
  ///   filter. arXiv preprint arXiv:1711.02508.
  CoreStateType PropagateState(const CoreStateType& prior_state, const IMUMeasurementType& measurement,
                               const double& dt);
  ///
  /// \brief PredictProcessCovariance Predicted core state covariance and generate the state transition matrix
  /// \param prior_core_state
  /// \param measurement System input
  /// \param dt propagation timespan
  /// \return Core state covariance and state transition matrix
  ///
  /// \note The Covariance is generated for the error state definition.
  CoreType PredictProcessCovariance(const CoreType& prior_core_state, const IMUMeasurementType& system_input,
                                    const double& dt);

  // Static
  ///
  /// \brief GenerateFdTaylor Generates the state-transition matrix with cut-off Taylor series
  /// \return state-transition matrix
  ///
  static CoreStateMatrix GenerateFdTaylor();

  ///
  /// \brief GenerateFdClosedForm Generates the state-transition matrix in closed-form
  /// \note Solar - Quaternion Kinematics Section B.3
  /// \return state-transition matrix
  ///
  static CoreStateMatrix GenerateFdClosedForm();

  ///
  /// \brief GenerateFdSmallAngleApprox Generates the state-transition matrix with small angle approximation
  ///
  /// \param q_wi   orientation of imu in world
  /// \param a_est  the estimate of the acceleration (a_m - b_a)
  /// \param w_est  the estimate of the angular velocity (w_m - b_w)
  /// \param dt     time step
  /// \param error_state_size
  ///
  /// \note Eq.(3.35) in S. Weiss, "Vision Based Navigation for Micro Helicopters (PhD Thesis - Weiss 2012)," Thesis.
  ///
  /// \return state-transition matrix
  ///
  static CoreStateMatrix GenerateFdSmallAngleApprox(const Eigen::Quaterniond& q_wi, const Eigen::Vector3d& a_est,
                                                    const Eigen::Vector3d& w_est, const double& dt);
  static CoreStateMatrix CalcQSmallAngleApprox(const double& dt, const Eigen::Quaterniond& q_wi,
                                               const Eigen::Vector3d& a_m, const Eigen::Vector3d& n_a,
                                               const Eigen::Vector3d& b_a, const Eigen::Vector3d& n_ba,
                                               const Eigen::Vector3d& w_m, const Eigen::Vector3d& n_w,
                                               const Eigen::Vector3d& b_w, const Eigen::Vector3d& n_bw);
};
}

#endif  // CORESTATE_H
