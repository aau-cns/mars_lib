// Copyright (C) 2021 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <christian.brommer@ieee.org>

#ifndef CORESTATETYPE_H
#define CORESTATETYPE_H

#include <mars/general_functions/utils.h>
#include <Eigen/Dense>
#include <string>

namespace mars
{
class CoreStateType
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  CoreStateType() = default;

  Eigen::Vector3d p_wi_{ Eigen::Vector3d::Zero() };
  Eigen::Vector3d v_wi_{ Eigen::Vector3d::Zero() };
  Eigen::Quaternion<double> q_wi_{ Eigen::Quaternion<double>::Identity() };
  Eigen::Vector3d b_w_{ Eigen::Vector3d::Zero() };
  Eigen::Vector3d b_a_{ Eigen::Vector3d::Zero() };

  // will be removed in next version ISSUE: #14
  Eigen::Vector3d w_m_{ Eigen::Vector3d::Zero() };
  Eigen::Vector3d a_m_{ Eigen::Vector3d::Zero() };

  static constexpr int size_true_ = 16;
  static constexpr int size_error_ = 15;

  ///
  /// \brief ApplyCorrection
  /// \param state_prior
  /// \param correction order [p_wi(0:2), v_wi(3:5), q_wi(6:8), b_w(9:11), b_a(12:14)]
  /// \return Corrected state
  ///
  static CoreStateType ApplyCorrection(CoreStateType state_prior,
                                       Eigen::Matrix<double, CoreStateType::size_error_, 1> correction)
  {
    // APPLY_CORRECTION Applies the given correction to the provided state_prior
    // state + error state correction
    // with quaternion from small-angle approx -> new state

    CoreStateType corrected_state;

    corrected_state.p_wi_ = state_prior.p_wi_ + correction.block(0, 0, 3, 1);
    corrected_state.v_wi_ = state_prior.v_wi_ + correction.block(3, 0, 3, 1);

    // Attention: due to small-angle to quaternion conversion,
    // the index for the corrected state does not map 1:1 (7:9 to 7:10)
    // this is important for the mapping idx after the quaternion.
    corrected_state.q_wi_ = Utils::ApplySmallAngleQuatCorr(state_prior.q_wi_, correction.block(6, 0, 3, 1));

    //      %if obj.fixed_bias
    //      %    correction(10:12) = zeros(3,1);
    //      %    correction(13:15) = zeros(3,1);
    //      %end

    corrected_state.b_w_ = state_prior.b_w_ + correction.block(9, 0, 3, 1);
    corrected_state.b_a_ = state_prior.b_a_ + correction.block(12, 0, 3, 1);

    // Pass through IMU measurements
    corrected_state.a_m_ = state_prior.a_m_;
    corrected_state.w_m_ = state_prior.w_m_;

    return corrected_state;
  }

  friend std::ostream& operator<<(std::ostream& out, const CoreStateType& data)
  {
    out.precision(10);
    out << std::fixed;
    out << "p_wi:\t[ " << data.p_wi_.transpose() << " ]" << std::endl
        << "v_wi:\t[ " << data.v_wi_.transpose() << " ]" << std::endl
        << "q_wi:\t[ " << data.q_wi_.w() << " " << data.q_wi_.vec().transpose() << " ]" << std::endl
        << "b_w:\t[ " << data.b_w_.transpose() << " ]" << std::endl
        << "b_a:\t[ " << data.b_a_.transpose() << " ]" << std::endl
        << "w_m:\t[ " << data.w_m_.transpose() << " ]" << std::endl
        << "a_m:\t[ " << data.a_m_.transpose() << " ]" << std::endl;

    return out;
  }

  static std::string get_csv_state_header_string()
  {
    std::stringstream os;
    os << "t, ";
    os << "w_m_x, w_m_y, w_m_z, ";
    os << "a_m_x, a_m_y, a_m_z, ";
    os << "p_wi_x, p_wi_y, p_wi_z, ";
    os << "v_wi_x, v_wi_y, v_wi_z, ";
    os << "q_wi_w, q_wi_x, q_wi_y, q_wi_z, ";
    os << "b_w_x, b_w_y, b_w_z, ";
    os << "b_a_x, b_a_y, b_a_z";

    return os.str();
  }

  ///
  /// \brief to_csv_string export state to single csv string
  /// \param timestamp
  /// \return string format [p_wi v_wi q_wi b_w b_a]
  ///
  std::string to_csv_string(const double& timestamp) const
  {
    std::stringstream os;
    os.precision(17);
    os << timestamp;

    os << ", " << w_m_(0) << ", " << w_m_(1) << ", " << w_m_(2);
    os << ", " << a_m_(0) << ", " << a_m_(1) << ", " << a_m_(2);
    os << ", " << p_wi_(0) << ", " << p_wi_(1) << ", " << p_wi_(2);
    os << ", " << v_wi_(0) << ", " << v_wi_(1) << ", " << v_wi_(2);

    Eigen::Vector4d q_wi = q_wi_.coeffs();  // x y z w
    os << ", " << q_wi(3) << ", " << q_wi(0) << ", " << q_wi(1) << ", " << q_wi(2);

    os << ", " << b_w_(0) << ", " << b_w_(1) << ", " << b_w_(2);
    os << ", " << b_a_(0) << ", " << b_a_(1) << ", " << b_a_(2);

    return os.str();
  }
};

using CoreStateMatrix = Eigen::Matrix<double, CoreStateType::size_error_, CoreStateType::size_error_>;
using CoreStateVector = Eigen::Matrix<double, CoreStateType::size_error_, 1>;
}
#endif  // CORESTATETYPE_H
