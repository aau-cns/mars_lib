// Copyright (C) 2021 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <christian.brommer@ieee.org>

#include <mars/core_state.h>
#include <mars/general_functions/utils.h>
#include <mars/time.h>
#include <mars/type_definitions/core_state_type.h>

#include <utility>

namespace mars
{
CoreState::CoreState()
{
  // Set default initial state covariance
  // Standard deviation
  Eigen::Vector3d position_std(1, 1, 1);
  Eigen::Vector3d velocity_std(0.5, 0.5, 0.5);
  Eigen::Vector3d orientation_std(30 * (M_PI / 180), 30 * (M_PI / 180), 30 * (M_PI / 180));
  Eigen::Vector3d bias_gyro_std(1 * (M_PI / 180), 1 * (M_PI / 180), 1 * (M_PI / 180));
  Eigen::Vector3d bias_acc_std(0.05, 0.05, 0.05);

  // Covariance
  Eigen::Matrix<double, CoreStateType::size_error_, 1> core_std;
  core_std << position_std, velocity_std, orientation_std, bias_gyro_std, bias_acc_std;

  initial_covariance_ = CoreStateMatrix(core_std.cwiseProduct(core_std).asDiagonal());
}

void CoreState::set_fixed_acc_bias(const bool& value)
{
  fixed_acc_bias_ = value;
}

void CoreState::set_fixed_gyro_bias(const bool& value)
{
  fixed_gyro_bias_ = value;
}

void CoreState::set_propagation_sensor(std::shared_ptr<SensorAbsClass> propagation_sensor)
{
  propagation_sensor_ = std::move(propagation_sensor);
}

CoreStateType CoreState::InitializeState(const Eigen::Vector3d& ang_vel, const Eigen::Vector3d& lin_acc,
                                         const Eigen::Vector3d& p_wi, const Eigen::Vector3d& v_wi,
                                         const Eigen::Quaterniond& q_wi, const Eigen::Vector3d& b_w,
                                         const Eigen::Vector3d& b_a)
{
  state.a_m_ = lin_acc;
  state.w_m_ = ang_vel;
  state.p_wi_ = p_wi;
  state.v_wi_ = v_wi;
  state.q_wi_ = q_wi;
  state.b_w_ = b_w;
  state.b_a_ = b_a;

  std::cout << "States Initialized:" << std::endl;
  std::cout << state << std::endl;

  return state;
}

void CoreState::set_noise_std(const Eigen::Vector3d& n_w, const Eigen::Vector3d& n_bw, const Eigen::Vector3d& n_a,
                              const Eigen::Vector3d& n_ba)
{
  n_w_ = n_w;
  n_bw_ = n_bw;
  n_a_ = n_a;
  n_ba_ = n_ba;
}

void CoreState::set_initial_covariance(const Eigen::Vector3d& p_cov, const Eigen::Vector3d& v_cov,
                                       const Eigen::Vector3d& q_cov, const Eigen::Vector3d& bw_cov,
                                       const Eigen::Vector3d& ba_cov)
{
  Eigen::Matrix<double, CoreStateType::size_error_, 1> core_cov;
  core_cov << p_cov, v_cov, q_cov, bw_cov, ba_cov;

  initial_covariance_ = CoreStateMatrix(core_cov.asDiagonal());
}

CoreStateMatrix CoreState::InitializeCovariance()
{
  return initial_covariance_;
}

CoreStateType CoreState::PropagateState(const CoreStateType& prior_state, const IMUMeasurementType& measurement,
                                        const double& dt)
{
  CoreStateType current_state;

  double delta_t = std::abs(dt);

  // Map System Input
  current_state.w_m_ = measurement.angular_velocity_;
  current_state.a_m_ = measurement.linear_acceleration_;

  // Zero propagation
  if (fixed_gyro_bias_)
  {
    current_state.b_w_.setZero();
  }
  else
  {
    current_state.b_w_ = prior_state.b_w_;
  }

  if (fixed_acc_bias_)
  {
    current_state.b_a_.setZero();
  }
  else
  {
    current_state.b_a_ = prior_state.b_a_;
  }

  // First order Quaternion integration
  const Eigen::Vector3d ew = current_state.w_m_ - current_state.b_w_;
  const Eigen::Vector3d ew_old = prior_state.w_m_ - prior_state.b_w_;
  const Eigen::Vector3d median_turn_rate = (ew_old + ew) / 2;

  // Quaternion right side multiplication matrix from angular turn rates
  const Eigen::Matrix4d omega = Utils::OmegaMat(ew);
  const Eigen::Matrix4d omega_old = Utils::OmegaMat(ew_old);
  const Eigen::Matrix4d omega_med = Utils::OmegaMat(median_turn_rate);

  // Matrix exponential approximation
  // Reference: Solar - Quaternion Kinematics - Equation(224b)
  const Eigen::Matrix4d omega_n = omega_med * 0.5 * delta_t;
  const Eigen::Matrix4d matexp = Utils::MatExp(omega_n, 4);

  // first order Quaternion integration matrix
  const Eigen::Matrix4d quat_int = matexp + ((omega * omega_old - omega_old * omega) * (delta_t * delta_t)) / 48.0;

  const Eigen::Vector4d prior_quat_coeffs(prior_state.q_wi_.w(), prior_state.q_wi_.x(), prior_state.q_wi_.y(),
                                          prior_state.q_wi_.z());

  const Eigen::Vector4d current_q_wi_coeffs = (quat_int * prior_quat_coeffs);

  current_state.q_wi_ = Eigen::Quaterniond(current_q_wi_coeffs.x(), current_q_wi_coeffs.y(), current_q_wi_coeffs.z(),
                                           current_q_wi_coeffs.w());
  current_state.q_wi_.normalize();

  // integrate linear acceleration
  const Eigen::Vector3d ea = current_state.a_m_ - current_state.b_a_;
  const Eigen::Vector3d ea_old = prior_state.a_m_ - prior_state.b_a_;

  const Eigen::Vector3d dv =
      (current_state.q_wi_.toRotationMatrix() * ea + prior_state.q_wi_.toRotationMatrix() * ea_old) / 2;
  current_state.v_wi_ = prior_state.v_wi_ + (dv - g_) * delta_t;

  // integrate velocity
  current_state.p_wi_ = prior_state.p_wi_ + ((current_state.v_wi_ + prior_state.v_wi_) / 2) * delta_t;

  return current_state;
}

CoreType CoreState::PredictProcessCovariance(const CoreType& prior_core_state, const IMUMeasurementType& system_input,
                                             const double& dt)
{
  const CoreStateMatrix P = prior_core_state.cov_;
  const Eigen::Quaterniond q_wi(prior_core_state.state_.q_wi_);
  const Eigen::Vector3d b_a = prior_core_state.state_.b_a_;
  const Eigen::Vector3d b_w = prior_core_state.state_.b_w_;

  const Eigen::Vector3d w_m = system_input.angular_velocity_;
  const Eigen::Vector3d a_m = system_input.linear_acceleration_;

  const Eigen::Vector3d w_est = w_m - b_w;
  const Eigen::Vector3d a_est = a_m - b_a;

  // State-Transition and Process-Noise
  CoreStateMatrix F_d = GenerateFdSmallAngleApprox(q_wi, a_est, w_est, dt);
  CoreStateMatrix Q_d =
      CalcQSmallAngleApprox(dt, q_wi, a_m, this->n_a_, b_a, this->n_ba_, w_m, this->n_w_, b_w, this->n_bw_);

  CoreStateMatrix P_predicted_raw = F_d * P * F_d.transpose() + Q_d;

  CoreStateMatrix propagated_state_covariance = Utils::EnforceMatrixSymmetry(P_predicted_raw);
  CoreStateMatrix state_transition = F_d;

  CoreType result;
  result.cov_ = propagated_state_covariance;
  result.state_transition_ = state_transition;
  return result;
}

CoreStateMatrix CoreState::GenerateFdSmallAngleApprox(const Eigen::Quaterniond& q_wi, const Eigen::Vector3d& a_est,
                                                      const Eigen::Vector3d& w_est, const double& dt)
{
  const Eigen::Matrix3d R = q_wi.toRotationMatrix();

  const Eigen::Matrix3d I(Eigen::Matrix3d::Identity());

  // Prepare dt powers (dt_p2 = dt power 2)
  const double dt_p2 = dt * dt;
  const double dt_p3 = dt_p2 * dt;
  const double dt_p4 = dt_p2 * dt_p2;
  const double dt_p5 = dt_p4 * dt;

  const Eigen::Matrix3d skew_w_est = Utils::Skew(w_est);
  const Eigen::Matrix3d skew_a_est = Utils::Skew(a_est);
  const Eigen::Matrix3d skew_w_est_p2 = skew_w_est * skew_w_est;

  const Eigen::Matrix3d A =
      -R * skew_a_est * (I * ((dt_p2) / 2) - ((dt_p3) / 6) * skew_w_est + ((dt_p4) / 24) * skew_w_est_p2);
  const Eigen::Matrix3d B =
      -R * skew_a_est * (I * ((-dt_p3) / 6) + ((dt_p4) / 24) * skew_w_est - ((dt_p5) / 120) * skew_w_est_p2);
  const Eigen::Matrix3d C = -R * skew_a_est * (I * dt - ((dt_p2) / 2) * skew_w_est + ((dt_p3) / 6) * skew_w_est_p2);
  const Eigen::Matrix3d D = -A;
  const Eigen::Matrix3d E = I - dt * skew_w_est + ((dt_p2) / 2) * skew_w_est_p2;
  const Eigen::Matrix3d F = -dt * I + ((dt_p2) / 2) * skew_w_est - (dt_p3) / 6 * skew_w_est_p2;

  // Map matrix components
  CoreStateMatrix F_d(CoreStateMatrix::Identity());
  F_d.block(0, 3, 3, 3) = I * dt;
  F_d.block(0, 6, 3, 3) = A;
  F_d.block(0, 9, 3, 3) = B;
  F_d.block(0, 12, 3, 3) = -R * ((dt_p2) / 2);

  F_d.block(3, 6, 3, 3) = C;
  F_d.block(3, 9, 3, 3) = D;
  F_d.block(3, 12, 3, 3) = -R * dt;

  F_d.block(6, 6, 3, 3) = E;
  F_d.block(6, 9, 3, 3) = F;
  return F_d;
}
}  // namespace mars
