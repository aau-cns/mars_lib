// Copyright (C) 2022 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <christian.brommer@ieee.org>

#include <mars/core_logic.h>
#include <mars/core_state.h>
#include <mars/data_utils/filesystem.h>
#include <mars/data_utils/read_baro_data.h>
#include <mars/data_utils/read_gps_w_vel_data.h>
#include <mars/data_utils/read_imu_data.h>
#include <mars/data_utils/read_mag_data.h>
#include <mars/data_utils/read_vision_data.h>
#include <mars/data_utils/write_csv.h>
#include <mars/general_functions/progress_indicator.h>
#include <mars/sensors/gps_w_vel/gps_w_vel_measurement_type.h>
#include <mars/sensors/gps_w_vel/gps_w_vel_sensor_class.h>
#include <mars/sensors/imu/imu_sensor_class.h>
#include <mars/sensors/mag/mag_measurement_type.h>
#include <mars/sensors/mag/mag_sensor_class.h>
#include <mars/sensors/mag/mag_utils.h>
#include <mars/sensors/pressure/pressure_measurement_type.h>
#include <mars/sensors/pressure/pressure_sensor_class.h>
#include <mars/sensors/vision/vision_measurement_type.h>
#include <mars/sensors/vision/vision_sensor_class.h>

#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <cmath>
#include <iomanip>
#include <iostream>
#include "include/mars_insane_dataset.h"
#include "include_local/insane_dataset_settings.h"

int main(int /*argc*/, char** /*argv[]*/)
{
  // Load configuration
  std::string settings_path = std::string(MARS_LIB_INSANE_DATASET_DATA_PATH);
  ParamLoad m_sett(settings_path + "insane_dataset.yaml");

  // Check source data directory
  std::cout << "[Info] Checking data path" << std::endl;

  if (!mars::filesystem::IsDir(m_sett.data_path_))
  {
    std::cout << "[Warning] Given data directory: " << m_sett.data_path_ << " does not exist" << std::endl;
    exit(EXIT_FAILURE);
  }

  // Initialize framework components
  std::cout << "[Info] Setup framework components" << std::endl;

  std::shared_ptr<mars::ImuSensorClass> imu_sensor_sptr_;  ///< Propagation sensor instance
  std::shared_ptr<mars::CoreState> core_states_sptr_;      ///< Core State instance
  mars::CoreLogic core_logic_;                             ///< Core Logic instance

  mars::MagnetometerInit mag_init_;

  // Sensor instances
  std::shared_ptr<mars::MagSensorClass> mag1_sensor_sptr_;        ///< MAG 1 sensor instance
  std::shared_ptr<mars::MagSensorClass> mag2_sensor_sptr_;        ///< MAG 2 sensor instance
  std::shared_ptr<mars::GpsVelSensorClass> gps1_sensor_sptr_;     ///< GPS 1 sensor instance
  std::shared_ptr<mars::GpsVelSensorClass> gps2_sensor_sptr_;     ///< GPS 2 sensor instance
  std::shared_ptr<mars::GpsVelSensorClass> gps3_sensor_sptr_;     ///< GPS 3 sensor instance
  std::shared_ptr<mars::VisionSensorClass> pose1_sensor_sptr_;    ///< POSE 1 sensor instance
  std::shared_ptr<mars::VisionSensorClass> pose2_sensor_sptr_;    ///< POSE 2 sensor instance
  std::shared_ptr<mars::PressureSensorClass> baro1_sensor_sptr_;  ///< BARO 1 sensor instance

  // Framework components
  imu_sensor_sptr_ = std::make_shared<mars::ImuSensorClass>("IMU");
  core_states_sptr_ = std::make_shared<mars::CoreState>();
  core_states_sptr_.get()->set_initial_covariance(m_sett.core_init_cov_p_, m_sett.core_init_cov_v_,
                                                  m_sett.core_init_cov_q_, m_sett.core_init_cov_bw_,
                                                  m_sett.core_init_cov_ba_);

  core_states_sptr_.get()->set_propagation_sensor(imu_sensor_sptr_);
  core_states_sptr_.get()->set_fixed_acc_bias(m_sett.disable_acc_bias_);
  core_states_sptr_.get()->set_fixed_gyro_bias(m_sett.disable_gyro_bias_);

  core_logic_ = mars::CoreLogic(core_states_sptr_);
  core_logic_.buffer_.set_max_buffer_size(m_sett.buffer_size_);

  core_logic_.verbose_ = m_sett.verbose_output_;
  core_logic_.verbose_out_of_order_ = m_sett.verbose_ooo_;
  core_logic_.discard_ooo_prop_meas_ = m_sett.discard_ooo_prop_meas_;

  core_states_sptr_->set_noise_std(Eigen::Vector3d(m_sett.g_rate_noise_, m_sett.g_rate_noise_, m_sett.g_rate_noise_),
                                   Eigen::Vector3d(m_sett.g_bias_noise_, m_sett.g_bias_noise_, m_sett.g_bias_noise_),
                                   Eigen::Vector3d(m_sett.a_noise_, m_sett.a_noise_, m_sett.a_noise_),
                                   Eigen::Vector3d(m_sett.a_bias_noise_, m_sett.a_bias_noise_, m_sett.a_bias_noise_));

  // Default initial condition
  Eigen::Vector3d p_wi_init(0, 0, 0);
  Eigen::Quaterniond q_wi_init = Eigen::Quaterniond::Identity();

  // Latest GNSS measurement for init
  Eigen::Vector3d p_wg_init(0, 0, 0);

  // Sensors
  bool common_gps_ref_is_set_ = false;

  // GPS1
  gps1_sensor_sptr_ = std::make_shared<mars::GpsVelSensorClass>("Gps1", core_states_sptr_);

  {  // Limit scope of temp variables
    Eigen::Matrix<double, 6, 1> gps_meas_std;
    gps_meas_std << m_sett.gps1_pos_meas_noise_, m_sett.gps1_vel_meas_noise_;

    gps1_sensor_sptr_->R_ = gps_meas_std.cwiseProduct(gps_meas_std);

    mars::GpsVelSensorData gps_calibration;
    gps_calibration.state_.p_ig_ = Eigen::Vector3d(m_sett.gps1_cal_ig_);

    Eigen::Matrix<double, 9, 9> gps_cov;
    gps_cov.setZero();
    gps_cov.diagonal() << m_sett.gps1_state_init_cov_, 1e-15, 1e-15, 1e-15, 1e-15, 1e-15, 1e-15;
    gps_calibration.sensor_cov_ = gps_cov;

    gps1_sensor_sptr_->set_initial_calib(std::make_shared<mars::GpsVelSensorData>(gps_calibration));

    gps1_sensor_sptr_->set_use_vel_rot(m_sett.gps1_use_vel_rot_);
    gps1_sensor_sptr_->set_v_rot_axis(m_sett.gps1_vel_rot_axis_);
    gps1_sensor_sptr_->set_vel_rot_thr(m_sett.gps1_vel_rot_thr_);

    // Chi2 settings
    gps1_sensor_sptr_->chi2_.set_chi_value(m_sett.gps1_chi2_value_);
    gps1_sensor_sptr_->chi2_.ActivateTest(m_sett.gps1_chi2_enable_);

    // TODO is set here for now, but will be managed by core logic in later versions
    gps1_sensor_sptr_->const_ref_to_nav_ = true;
  }

  // GPS2
  gps2_sensor_sptr_ = std::make_shared<mars::GpsVelSensorClass>("Gps2", core_states_sptr_);

  {  // Limit scope of temp variables
    Eigen::Matrix<double, 6, 1> gps_meas_std;
    gps_meas_std << m_sett.gps2_pos_meas_noise_, m_sett.gps2_vel_meas_noise_;

    gps2_sensor_sptr_->R_ = gps_meas_std.cwiseProduct(gps_meas_std);

    mars::GpsVelSensorData gps_calibration;
    gps_calibration.state_.p_ig_ = Eigen::Vector3d(m_sett.gps2_cal_ig_);

    Eigen::Matrix<double, 9, 9> gps_cov;
    gps_cov.setZero();
    gps_cov.diagonal() << m_sett.gps2_state_init_cov_, 1e-15, 1e-15, 1e-15, 1e-15, 1e-15, 1e-15;
    gps_calibration.sensor_cov_ = gps_cov;

    gps2_sensor_sptr_->set_initial_calib(std::make_shared<mars::GpsVelSensorData>(gps_calibration));

    gps2_sensor_sptr_->set_use_vel_rot(m_sett.gps2_use_vel_rot_);
    gps2_sensor_sptr_->set_v_rot_axis(m_sett.gps2_vel_rot_axis_);
    gps2_sensor_sptr_->set_vel_rot_thr(m_sett.gps2_vel_rot_thr_);

    // Chi2 settings
    gps2_sensor_sptr_->chi2_.set_chi_value(m_sett.gps2_chi2_value_);
    gps2_sensor_sptr_->chi2_.ActivateTest(m_sett.gps2_chi2_enable_);

    // TODO is set here for now, but will be managed by core logic in later versions
    gps2_sensor_sptr_->const_ref_to_nav_ = true;
  }

  // GPS3
  gps3_sensor_sptr_ = std::make_shared<mars::GpsVelSensorClass>("Gps3", core_states_sptr_);

  {  // Limit scope of temp variables
    Eigen::Matrix<double, 6, 1> gps_meas_std;
    gps_meas_std << m_sett.gps3_pos_meas_noise_, m_sett.gps3_vel_meas_noise_;

    gps3_sensor_sptr_->R_ = gps_meas_std.cwiseProduct(gps_meas_std);

    mars::GpsVelSensorData gps_calibration;
    gps_calibration.state_.p_ig_ = Eigen::Vector3d(m_sett.gps3_cal_ig_);

    Eigen::Matrix<double, 9, 9> gps_cov;
    gps_cov.setZero();
    gps_cov.diagonal() << m_sett.gps3_state_init_cov_, 1e-15, 1e-15, 1e-15, 1e-15, 1e-15, 1e-15;
    gps_calibration.sensor_cov_ = gps_cov;

    gps3_sensor_sptr_->set_initial_calib(std::make_shared<mars::GpsVelSensorData>(gps_calibration));

    gps3_sensor_sptr_->set_use_vel_rot(m_sett.gps3_use_vel_rot_);
    gps3_sensor_sptr_->set_v_rot_axis(m_sett.gps3_vel_rot_axis_);
    gps3_sensor_sptr_->set_vel_rot_thr(m_sett.gps3_vel_rot_thr_);

    // Chi2 settings
    gps3_sensor_sptr_->chi2_.set_chi_value(m_sett.gps3_chi2_value_);
    gps3_sensor_sptr_->chi2_.ActivateTest(m_sett.gps3_chi2_enable_);

    // TODO is set here for now, but will be managed by core logic in later versions
    gps3_sensor_sptr_->const_ref_to_nav_ = true;
  }

  // MAG1
  mag1_sensor_sptr_ = std::make_shared<mars::MagSensorClass>("Mag1", core_states_sptr_);

  {
    // Limit scope of temp variables
    Eigen::Matrix<double, 3, 1> mag_meas_std;
    mag_meas_std << m_sett.mag1_meas_noise_;
    mag1_sensor_sptr_->R_ = mag_meas_std.cwiseProduct(mag_meas_std);

    mars::MagSensorData mag_calibration;
    mag_calibration.state_.mag_ = mars::MagnetometerInit::mag_var_ang_to_vec(m_sett.mag1_decl_, m_sett.mag1_incl_);
    mag_calibration.state_.q_im_ = m_sett.mag1_cal_q_im_;

    Eigen::Matrix<double, 6, 6> mag_cov;
    mag_cov.setZero();
    mag_cov.diagonal() << m_sett.mag1_state_init_cov_;
    mag_calibration.sensor_cov_ = mag_cov;

    mag1_sensor_sptr_->set_initial_calib(std::make_shared<mars::MagSensorData>(mag_calibration));

    // TODO is set here for now, but will be managed by core logic in later versions
    mag1_sensor_sptr_->const_ref_to_nav_ = true;
    mag1_sensor_sptr_->set_normalize(m_sett.mag1_normalize_);
    mag1_sensor_sptr_->set_apply_intrinsic(m_sett.mag1_apply_intrinsic_);
    mag1_sensor_sptr_->set_intr_offset(m_sett.mag1_intr_offset_);
    mag1_sensor_sptr_->set_intr_transform(m_sett.mag1_intr_transform_);
  }

  // MAG2
  mag2_sensor_sptr_ = std::make_shared<mars::MagSensorClass>("Mag2", core_states_sptr_);

  {
    // Limit scope of temp variables
    Eigen::Matrix<double, 3, 1> mag_meas_std;
    mag_meas_std << m_sett.mag2_meas_noise_;
    mag2_sensor_sptr_->R_ = mag_meas_std.cwiseProduct(mag_meas_std);

    mars::MagSensorData mag_calibration;
    mag_calibration.state_.mag_ = mars::MagnetometerInit::mag_var_ang_to_vec(m_sett.mag2_decl_, m_sett.mag2_incl_);
    mag_calibration.state_.q_im_ = m_sett.mag2_cal_q_im_;

    Eigen::Matrix<double, 6, 6> mag_cov;
    mag_cov.setZero();
    mag_cov.diagonal() << m_sett.mag2_state_init_cov_;
    mag_calibration.sensor_cov_ = mag_cov;

    mag2_sensor_sptr_->set_initial_calib(std::make_shared<mars::MagSensorData>(mag_calibration));

    // TODO is set here for now, but will be managed by core logic in later versions
    mag2_sensor_sptr_->const_ref_to_nav_ = true;
    mag2_sensor_sptr_->set_normalize(m_sett.mag1_normalize_);
    mag2_sensor_sptr_->set_apply_intrinsic(m_sett.mag2_apply_intrinsic_);
    mag2_sensor_sptr_->set_intr_offset(m_sett.mag2_intr_offset_);
    mag2_sensor_sptr_->set_intr_transform(m_sett.mag2_intr_transform_);
  }

  // Pose1
  pose1_sensor_sptr_ = std::make_shared<mars::VisionSensorClass>("Pose1", core_states_sptr_, m_sett.pose1_use_scale_);

  {  // Limit scope of temp variables

    Eigen::Matrix<double, 6, 1> pose_meas_std;
    pose_meas_std << m_sett.pose1_pos_meas_noise_, m_sett.pose1_rot_meas_noise_;
    pose1_sensor_sptr_->R_ = pose_meas_std.cwiseProduct(pose_meas_std);

    mars::VisionSensorData pose_calibration;
    pose_calibration.state_.p_ic_ = m_sett.pose1_cal_p_ip_;
    pose_calibration.state_.q_ic_ = m_sett.pose1_cal_q_ip_;

    Eigen::Matrix<double, 13, 13> pose_cov;
    pose_cov.setZero();
    pose_cov.diagonal() << 0.025, 0.025, 0.025, 0.1218, 0.1218, 0.1218, m_sett.pose1_state_init_cov_, 0.0025;
    pose_calibration.sensor_cov_ = pose_cov;

    pose1_sensor_sptr_->set_initial_calib(std::make_shared<mars::VisionSensorData>(pose_calibration));

    pose1_sensor_sptr_->const_ref_to_nav_ = true;
  }

  // Pose2
  pose2_sensor_sptr_ = std::make_shared<mars::VisionSensorClass>("Pose2", core_states_sptr_, m_sett.pose2_use_scale_);

  {  // Limit scope of temp variables

    Eigen::Matrix<double, 6, 1> pose_meas_std;
    pose_meas_std << m_sett.pose1_pos_meas_noise_, m_sett.pose2_rot_meas_noise_;
    pose2_sensor_sptr_->R_ = pose_meas_std.cwiseProduct(pose_meas_std);

    mars::VisionSensorData pose_calibration;
    pose_calibration.state_.p_ic_ = m_sett.pose2_cal_p_ip_;
    pose_calibration.state_.q_ic_ = m_sett.pose2_cal_q_ip_;

    Eigen::Matrix<double, 13, 13> pose_cov;
    pose_cov.setZero();
    pose_cov.diagonal() << 0.01, 0.01, 0.01, 0.0305, 0.0305, 0.0305, m_sett.pose2_state_init_cov_, 1e-10;
    pose_calibration.sensor_cov_ = pose_cov;

    pose2_sensor_sptr_->set_initial_calib(std::make_shared<mars::VisionSensorData>(pose_calibration));

    pose2_sensor_sptr_->const_ref_to_nav_ = false;
  }

  // Baro1
  baro1_sensor_sptr_ = std::make_shared<mars::PressureSensorClass>("Baro1", core_states_sptr_);

  {  // Limit scope of temp variables

    Eigen::Matrix<double, 1, 1> baro_meas_std;
    baro_meas_std << m_sett.baro1_meas_noise_;
    baro1_sensor_sptr_->R_ = baro_meas_std.cwiseProduct(baro_meas_std);

    mars::PressureSensorData baro_calibration;
    baro_calibration.state_.p_ip_ = m_sett.baro1_cal_p_ip_;
    baro_calibration.state_.bias_p_ = 0;

    Eigen::Matrix<double, 4, 4> baro_cov;
    baro_cov.setZero();
    baro_cov.diagonal() << m_sett.baro1_state_init_cov_;
    baro_calibration.sensor_cov_ = baro_cov;

    baro1_sensor_sptr_->set_initial_calib(std::make_shared<mars::PressureSensorData>(baro_calibration));

    // TODO is set here for now, but will be managed by core logic in later versions
    baro1_sensor_sptr_->const_ref_to_nav_ = true;
  }

  // Load Sensor Data
  std::vector<mars::BufferEntryType> measurement_data;

  {  // keep individual measurement data limited to this scope

    // IMU
    {
      std::cout << "Reading measurement files:" << std::endl;
      std::cout << " - Reading IMU Measurement Data" << std::endl;
      std::vector<mars::BufferEntryType> measurement_data_imu;
      mars::ReadImuData(&measurement_data_imu, imu_sensor_sptr_, m_sett.data_path_ + m_sett.imu_file_name_);
      measurement_data.insert(measurement_data.end(), measurement_data_imu.begin(), measurement_data_imu.end());
    }

    // GPS1
    if (m_sett.enable_gps1_)
    {
      std::cout << " - Reading GPS1 Measurement Data" << std::endl;
      std::vector<mars::BufferEntryType> measurement_data_gps1;
      mars::ReadGpsWithVelData(&measurement_data_gps1, gps1_sensor_sptr_, m_sett.data_path_ + m_sett.gps1_file_name_,
                               m_sett.t_offset_gps1_);
      measurement_data.insert(measurement_data.end(), measurement_data_gps1.begin(), measurement_data_gps1.end());
    }

    // GPS2
    if (m_sett.enable_gps2_)
    {
      std::cout << " - Reading GPS2 Measurement Data" << std::endl;
      std::vector<mars::BufferEntryType> measurement_data_gps2;
      mars::ReadGpsWithVelData(&measurement_data_gps2, gps2_sensor_sptr_, m_sett.data_path_ + m_sett.gps2_file_name_,
                               m_sett.t_offset_gps2_);
      measurement_data.insert(measurement_data.end(), measurement_data_gps2.begin(), measurement_data_gps2.end());
    }

    // GPS3
    if (m_sett.enable_gps3_)
    {
      std::cout << " - Reading GPS3 Measurement Data" << std::endl;
      std::vector<mars::BufferEntryType> measurement_data_gps3;
      mars::ReadGpsWithVelData(&measurement_data_gps3, gps3_sensor_sptr_, m_sett.data_path_ + m_sett.gps3_file_name_,
                               m_sett.t_offset_gps3_);
      measurement_data.insert(measurement_data.end(), measurement_data_gps3.begin(), measurement_data_gps3.end());
    }

    // Mag1
    if (m_sett.enable_mag1_)
    {
      std::cout << " - Reading MAG1 Measurement Data" << std::endl;
      std::vector<mars::BufferEntryType> measurement_data_mag1;
      mars::ReadMagData(&measurement_data_mag1, mag1_sensor_sptr_, m_sett.data_path_ + m_sett.mag1_file_name_,
                        m_sett.t_offset_mag1_);
      measurement_data.insert(measurement_data.end(), measurement_data_mag1.begin(), measurement_data_mag1.end());
    }

    // Mag2
    if (m_sett.enable_mag2_)
    {
      std::cout << " - Reading MAG2 Measurement Data" << std::endl;
      std::vector<mars::BufferEntryType> measurement_data_mag2;
      mars::ReadMagData(&measurement_data_mag2, mag2_sensor_sptr_, m_sett.data_path_ + m_sett.mag2_file_name_,
                        m_sett.t_offset_mag2_);
      measurement_data.insert(measurement_data.end(), measurement_data_mag2.begin(), measurement_data_mag2.end());
    }

    // Pose1
    if (m_sett.enable_pose1_)
    {
      std::cout << " - Reading Pose1 Measurement Data" << std::endl;
      std::vector<mars::BufferEntryType> measurement_data_pose1;
      mars::ReadVisionData(&measurement_data_pose1, pose1_sensor_sptr_, m_sett.data_path_ + m_sett.pose1_file_name_,
                           m_sett.t_offset_pose1_);
      measurement_data.insert(measurement_data.end(), measurement_data_pose1.begin(), measurement_data_pose1.end());
    }

    // Pose2
    if (m_sett.enable_pose2_)
    {
      std::cout << " - Reading Pose2 Measurement Data" << std::endl;
      std::vector<mars::BufferEntryType> measurement_data_pose2;
      mars::ReadVisionData(&measurement_data_pose2, pose2_sensor_sptr_, m_sett.data_path_ + m_sett.pose2_file_name_,
                           m_sett.t_offset_pose2_);
      std::vector<mars::BufferEntryType> measurement_data_pose2_reduced =
          mars::Utils::VecExtractEveryNthElm(measurement_data_pose2, 6);
      measurement_data.insert(measurement_data.end(), measurement_data_pose2_reduced.begin(),
                              measurement_data_pose2_reduced.end());
    }

    // Baro1
    if (m_sett.enable_baro1_)
    {
      std::cout << " - Reading Baro1 Measurement Data" << std::endl;
      std::vector<mars::BufferEntryType> measurement_data_baro1;
      mars::ReadBarometerData(&measurement_data_baro1, baro1_sensor_sptr_, m_sett.data_path_ + m_sett.baro1_file_name_,
                              m_sett.t_offset_baro1_);
      measurement_data.insert(measurement_data.end(), measurement_data_baro1.begin(), measurement_data_baro1.end());
    }

    std::cout << " - Sort Measurement Data" << std::endl;
    std::sort(measurement_data.begin(), measurement_data.end());

    // Remove entries according to t_start
    if (m_sett.t_start_ > 0)
    {
      std::cout << " - Remove entries according to t_start" << std::endl;
      const double t_start_global = m_sett.t_start_ + measurement_data[0].timestamp_.get_seconds();
      for (size_t k = 0; k < measurement_data.size(); k++)
      {
        if (t_start_global < measurement_data[k].timestamp_.get_seconds())
        {
          std::cout << " - Removing: " << measurement_data.size() - k << " entries due to t_start setting" << std::endl;
          measurement_data.erase(measurement_data.begin(), measurement_data.begin() + k);
          break;
        }
      }
    }

    // Remove entries according to t_stop
    if (m_sett.t_stop_ > 0)
    {
      std::cout << " - Remove entries according to t_stop" << std::endl;
      const double t_stop_global = m_sett.t_stop_ + measurement_data[0].timestamp_.get_seconds();
      for (size_t k = 0; k < measurement_data.size(); k++)
      {
        if (measurement_data[k].timestamp_.get_seconds() > t_stop_global)
        {
          std::cout << " - Removing: " << measurement_data.size() - k << " entries due to t_stop setting" << std::endl;
          measurement_data.erase(measurement_data.begin() + k, measurement_data.end());
          break;
        }
      }
    }

    std::cout << " - DONE" << std::endl;
  }

  // Generate result directory
  std::string result_path(m_sett.data_path_ + m_sett.result_dir_pref_);
  mars::filesystem::MakeDir(result_path);

  // Open file for data export
  std::cout << "Open State Output Files" << std::endl;
  std::ofstream ofile_core;
  ofile_core.open(result_path + "/mars_core_state.csv", std::ios::out);
  ofile_core << std::setprecision(17);
  ofile_core << mars::CoreStateType::get_csv_state_header_string();
  ofile_core << mars::WriteCsv::get_cov_header_string(15) << std::endl;

  // GPS1 state output file
  std::ofstream ofile_gps1;
  ofile_gps1.open(result_path + "/mars_gps1_state.csv", std::ios::out);
  ofile_gps1 << std::setprecision(17);
  ofile_gps1 << mars::GpsVelSensorStateType::get_csv_state_header_string();
  ofile_gps1 << mars::WriteCsv::get_cov_header_string(9) << std::endl;
  // GPS1 ENU output file
  std::ofstream ofile_gps1_enu;
  ofile_gps1_enu.open(result_path + "/mars_gps1_enu.csv", std::ios::out);
  ofile_gps1_enu << std::setprecision(17);
  ofile_gps1_enu << "t, p_x, p_y, p_z" << std::endl;

  // GPS2 output file
  std::ofstream ofile_gps2;
  ofile_gps2.open(result_path + "/mars_gps2_state.csv", std::ios::out);
  ofile_gps2 << std::setprecision(17);
  ofile_gps2 << mars::GpsVelSensorStateType::get_csv_state_header_string();
  ofile_gps2 << mars::WriteCsv::get_cov_header_string(9) << std::endl;
  // GPS2 ENU output file
  std::ofstream ofile_gps2_enu;
  ofile_gps2_enu.open(result_path + "/mars_gps2_enu.csv", std::ios::out);
  ofile_gps2_enu << std::setprecision(17);
  ofile_gps2_enu << "t, p_x, p_y, p_z" << std::endl;

  // GPS3 output file
  std::ofstream ofile_gps3;
  ofile_gps3.open(result_path + "/mars_gps3_state.csv", std::ios::out);
  ofile_gps3 << std::setprecision(17);
  ofile_gps3 << mars::GpsVelSensorStateType::get_csv_state_header_string();
  ofile_gps3 << mars::WriteCsv::get_cov_header_string(9) << std::endl;
  // GPS3 ENU output file
  std::ofstream ofile_gps3_enu;
  ofile_gps3_enu.open(result_path + "/mars_gps3_enu.csv", std::ios::out);
  ofile_gps3_enu << std::setprecision(17);
  ofile_gps3_enu << "t, p_x, p_y, p_z" << std::endl;

  // MAG1 output file
  std::ofstream ofile_mag1;
  ofile_mag1.open(result_path + "mars_mag1_state.csv", std::ios::out);
  ofile_mag1 << std::setprecision(17);
  ofile_mag1 << mars::MagSensorStateType::get_csv_state_header_string();
  ofile_mag1 << mars::WriteCsv::get_cov_header_string(6) << std::endl;

  // MAG2 output file
  std::ofstream ofile_mag2;
  ofile_mag2.open(result_path + "mars_mag2_state.csv", std::ios::out);
  ofile_mag2 << std::setprecision(17);
  ofile_mag2 << mars::MagSensorStateType::get_csv_state_header_string();
  ofile_mag2 << mars::WriteCsv::get_cov_header_string(6) << std::endl;

  // POSE1 output file
  std::ofstream ofile_pose1;
  ofile_pose1.open(result_path + "mars_pose1_state.csv", std::ios::out);
  ofile_pose1 << std::setprecision(17);
  ofile_pose1 << mars::VisionSensorStateType::get_csv_state_header_string();
  ofile_pose1 << mars::WriteCsv::get_cov_header_string(13) << std::endl;

  // POSE2 output file
  std::ofstream ofile_pose2;
  ofile_pose2.open(result_path + "mars_pose2_state.csv", std::ios::out);
  ofile_pose2 << std::setprecision(17);
  ofile_pose2 << mars::VisionSensorStateType::get_csv_state_header_string();
  ofile_pose2 << mars::WriteCsv::get_cov_header_string(13) << std::endl;

  // Baro1 output file
  std::ofstream ofile_baro1;
  ofile_baro1.open(result_path + "mars_baro1_state.csv", std::ios::out);
  ofile_baro1 << std::setprecision(17);
  ofile_baro1 << mars::PressureSensorStateType::get_csv_state_header_string();
  ofile_baro1 << mars::WriteCsv::get_cov_header_string(4) << std::endl;

  std::cout << "Start Filtering Process..." << std::endl;
  mars::ProgressIndicator disp_prog(int(measurement_data.size()), 10);

  if (m_sett.use_manual_gps_ref_)
  {
    const double ref_lat = m_sett.manual_gps_ref_(0);
    const double ref_long = m_sett.manual_gps_ref_(1);
    const double ref_alt = m_sett.manual_gps_ref_(2);

    mars::GpsCoordinates reference(ref_lat, ref_long, ref_alt);
    gps1_sensor_sptr_->set_gps_reference_coordinates(reference);
    gps2_sensor_sptr_->set_gps_reference_coordinates(reference);
    gps3_sensor_sptr_->set_gps_reference_coordinates(reference);

    std::cout << "[Info] Using manual GPS reference:" << std::endl;
    std::cout << reference << std::endl;

    common_gps_ref_is_set_ = true;
  }

  if (m_sett.enable_manual_yaw_init_)
  {
    const double yaw = m_sett.yaw_init_deg_ * (M_PI / 180);
    Eigen::Matrix3d r;
    r << cos(yaw), -sin(yaw), 0, sin(yaw), cos(yaw), 0, 0, 0, 1;
    q_wi_init = Eigen::Quaterniond(r);

    std::cout << "Manual yaw initialization: " << yaw * (180 / M_PI) << "\n" << std::endl;
  }

  // Main processing Loop
  for (auto k : measurement_data)
  {
    // Display progress
    disp_prog.next_step();

    // Mag with IMU Rotation Initialization
    if (!m_sett.enable_manual_yaw_init_ && (k.sensor_ == mag1_sensor_sptr_) && !mag_init_.IsDone())
    {
      // Get last IMU measurement
      mars::BufferEntryType latest_imu_meas_entry;
      if (!core_logic_.buffer_prior_core_init_.get_latest_sensor_handle_measurement(imu_sensor_sptr_,
                                                                                    &latest_imu_meas_entry))
      {
        // Stop init routine if no previous IMU measurement was found
        continue;
      };
      mars::IMUMeasurementType imu_meas =
          *static_cast<mars::IMUMeasurementType*>(latest_imu_meas_entry.data_.sensor_.get());

      // Get current Magnetometer measurement
      mars::MagMeasurementType mag_meas = *static_cast<mars::MagMeasurementType*>(k.data_.sensor_.get());

      // Feed measurements to rotation init buffer
      mag_init_.AddElement(m_sett.mag1_cal_q_im_.toRotationMatrix() * mag_meas.mag_vector_,
                           imu_meas.linear_acceleration_);

      if (mag_init_.get_size() >= m_sett.auto_mag_init_samples_)
      {
        q_wi_init = mag_init_.get_quat();
        mag_init_.set_done();
      }
    }

    // Reject Mag1 measurement updates if deactivated
    if ((k.sensor_ == mag1_sensor_sptr_) && !m_sett.update_on_mag1_)
    {
      continue;
    }

    // Reject Mag2 measurement updates if deactivated
    if ((k.sensor_ == mag2_sensor_sptr_) && !m_sett.update_on_mag2_)
    {
      continue;
    }

    // GPS 1
    if (k.sensor_ == gps1_sensor_sptr_)
    {
      mars::GpsCoordinates measurement = static_cast<mars::GpsVelMeasurementType*>(k.data_.sensor_.get())->coordinates_;

      if (!common_gps_ref_is_set_)
      {
        std::cout << "Setting the common GPS reference to: \n" << measurement << std::endl;

        gps1_sensor_sptr_->set_gps_reference_coordinates(measurement);
        gps2_sensor_sptr_->set_gps_reference_coordinates(measurement);
        gps3_sensor_sptr_->set_gps_reference_coordinates(measurement);

        common_gps_ref_is_set_ = true;
      }
      else
      {
        p_wg_init = gps1_sensor_sptr_->gps_conversion_.get_enu(measurement);
      }
    }

    // GPS 2
    if (k.sensor_ == gps2_sensor_sptr_)
    {
      if (!common_gps_ref_is_set_)
      {
        mars::GpsCoordinates reference = static_cast<mars::GpsVelMeasurementType*>(k.data_.sensor_.get())->coordinates_;

        std::cout << "Setting the common GPS reference to: \n" << reference << std::endl;

        gps1_sensor_sptr_->set_gps_reference_coordinates(reference);
        gps2_sensor_sptr_->set_gps_reference_coordinates(reference);
        gps3_sensor_sptr_->set_gps_reference_coordinates(reference);

        common_gps_ref_is_set_ = true;
      }
    }

    // GPS 3
    if (k.sensor_ == gps3_sensor_sptr_)
    {
      if (!common_gps_ref_is_set_)
      {
        mars::GpsCoordinates reference = static_cast<mars::GpsVelMeasurementType*>(k.data_.sensor_.get())->coordinates_;

        std::cout << "Setting the common GPS reference to: \n" << reference << std::endl;

        gps1_sensor_sptr_->set_gps_reference_coordinates(reference);
        gps2_sensor_sptr_->set_gps_reference_coordinates(reference);
        gps3_sensor_sptr_->set_gps_reference_coordinates(reference);

        common_gps_ref_is_set_ = true;
      }
    }

    // Perform the sensor update
    core_logic_.ProcessMeasurement(k.sensor_, k.timestamp_, k.data_);

    // Initialize the core based on individual conditions
    if (!core_logic_.core_is_initialized_)
    {
      // Only initialize after orientation was determened by Magnetometer
      if (m_sett.enable_mag1_ && !mag_init_.IsDone() && !m_sett.enable_manual_yaw_init_)
      {
        continue;
      }

      // Initialize the first time at which the propagation sensor occures
      if (k.sensor_ == core_logic_.core_states_->propagation_sensor_)
      {
        // Get GPS calibration w.r.t. the IMU frame
        Eigen::Vector3d p_ig = gps1_sensor_sptr_->get_state(gps1_sensor_sptr_->initial_calib_).p_ig_;

        // Get IMU position w.r.t. world based on GPS and Mag information
        p_wi_init = Eigen::Vector3d(0.2, 0.6, 0);  // p_wg_init - (q_wi_init.toRotationMatrix() * p_ig);

        core_logic_.Initialize(p_wi_init, q_wi_init);
      }

      continue;
    }

    // Store results in a csv file
    if (k.sensor_ == core_logic_.core_states_->propagation_sensor_)
    {
      mars::BufferEntryType latest_result;
      core_logic_.buffer_.get_latest_state(&latest_result);
      mars::CoreType last_core_entry = *static_cast<mars::CoreType*>(latest_result.data_.core_.get());
      // Write Core State
      ofile_core << last_core_entry.state_.to_csv_string(latest_result.timestamp_.get_seconds());
      // Write Core State Covariance
      ofile_core << mars::WriteCsv::cov_mat_to_csv(last_core_entry.cov_);
      // Terminate line
      ofile_core << std::endl;
    }

    if (k.sensor_ == mag1_sensor_sptr_)
    {
      mars::BufferEntryType latest_result;
      if (!core_logic_.buffer_.get_latest_sensor_handle_state(mag1_sensor_sptr_, &latest_result))
      {
        continue;
      };

      mars::MagSensorData mag_data = *static_cast<mars::MagSensorData*>(latest_result.data_.sensor_.get());

      // Write Sensor State
      ofile_mag1 << mag_data.state_.to_csv_string(latest_result.timestamp_.get_seconds());

      // Write Core State Covariance
      ofile_mag1 << mars::WriteCsv::cov_mat_to_csv(mag_data.sensor_cov_);

      // Terminate line
      ofile_mag1 << std::endl;
    }

    if (k.sensor_ == mag2_sensor_sptr_)
    {
      mars::BufferEntryType latest_result;
      if (!core_logic_.buffer_.get_latest_sensor_handle_state(mag2_sensor_sptr_, &latest_result))
      {
        continue;
      };

      mars::MagSensorData mag_data = *static_cast<mars::MagSensorData*>(latest_result.data_.sensor_.get());

      // Write Sensor State
      ofile_mag2 << mag_data.state_.to_csv_string(latest_result.timestamp_.get_seconds());

      // Write Core State Covariance
      ofile_mag2 << mars::WriteCsv::cov_mat_to_csv(mag_data.sensor_cov_);

      // Terminate line
      ofile_mag2 << std::endl;
    }

    if (k.sensor_ == gps1_sensor_sptr_)
    {
      mars::BufferEntryType latest_result;
      if (!core_logic_.buffer_.get_latest_sensor_handle_state(gps1_sensor_sptr_, &latest_result))
      {
        continue;
      };

      // Write GPS State
      mars::GpsVelSensorData gps_data = *static_cast<mars::GpsVelSensorData*>(latest_result.data_.sensor_.get());

      // Write Sensor State
      ofile_gps1 << gps_data.state_.to_csv_string(latest_result.timestamp_.get_seconds());

      // Write Core State Covariance
      ofile_gps1 << mars::WriteCsv::cov_mat_to_csv(gps_data.sensor_cov_);

      // Terminate line
      ofile_gps1 << std::endl;

      // Write GPS ENU
      const Eigen::Vector3d gps_enu(gps1_sensor_sptr_->gps_conversion_.get_enu(
          static_cast<mars::GpsVelMeasurementType*>(k.data_.sensor_.get())->coordinates_));
      ofile_gps1_enu << k.timestamp_ << "," << gps_enu[0] << "," << gps_enu[1] << "," << gps_enu[2] << std::endl;
    }

    if (k.sensor_ == gps2_sensor_sptr_)
    {
      mars::BufferEntryType latest_result;
      if (!core_logic_.buffer_.get_latest_sensor_handle_state(gps2_sensor_sptr_, &latest_result))
      {
        continue;
      };

      mars::GpsVelSensorData gps_data = *static_cast<mars::GpsVelSensorData*>(latest_result.data_.sensor_.get());

      // Write Sensor State
      ofile_gps2 << gps_data.state_.to_csv_string(latest_result.timestamp_.get_seconds());

      // Write Core State Covariance
      ofile_gps2 << mars::WriteCsv::cov_mat_to_csv(gps_data.sensor_cov_);

      // Terminate line
      ofile_gps2 << std::endl;

      // Write GPS ENU
      const Eigen::Vector3d gps_enu(gps2_sensor_sptr_->gps_conversion_.get_enu(
          static_cast<mars::GpsVelMeasurementType*>(k.data_.sensor_.get())->coordinates_));
      ofile_gps2_enu << k.timestamp_ << "," << gps_enu[0] << "," << gps_enu[1] << "," << gps_enu[2] << std::endl;
    }

    if (k.sensor_ == gps3_sensor_sptr_)
    {
      mars::BufferEntryType latest_result;
      if (!core_logic_.buffer_.get_latest_sensor_handle_state(gps3_sensor_sptr_, &latest_result))
      {
        continue;
      };

      mars::GpsVelSensorData gps_data = *static_cast<mars::GpsVelSensorData*>(latest_result.data_.sensor_.get());

      // Write Sensor State
      ofile_gps3 << gps_data.state_.to_csv_string(latest_result.timestamp_.get_seconds());

      // Write Core State Covariance
      ofile_gps3 << mars::WriteCsv::cov_mat_to_csv(gps_data.sensor_cov_);

      // Terminate line
      ofile_gps3 << std::endl;

      // Write GPS ENU
      const Eigen::Vector3d gps_enu(gps3_sensor_sptr_->gps_conversion_.get_enu(
          static_cast<mars::GpsVelMeasurementType*>(k.data_.sensor_.get())->coordinates_));
      ofile_gps3_enu << k.timestamp_ << "," << gps_enu[0] << "," << gps_enu[1] << "," << gps_enu[2] << std::endl;
    }

    if (k.sensor_ == pose1_sensor_sptr_)
    {
      mars::BufferEntryType latest_result;
      if (!core_logic_.buffer_.get_latest_sensor_handle_state(pose1_sensor_sptr_, &latest_result))
      {
        continue;
      };

      mars::VisionSensorData pose_data = *static_cast<mars::VisionSensorData*>(latest_result.data_.sensor_.get());

      // Write Sensor State
      ofile_pose1 << pose_data.state_.to_csv_string(latest_result.timestamp_.get_seconds());

      // Write Core State Covariance
      ofile_pose1 << mars::WriteCsv::cov_mat_to_csv(pose_data.sensor_cov_);

      // Terminate line
      ofile_pose1 << std::endl;
    }

    if (k.sensor_ == pose2_sensor_sptr_)
    {
      mars::BufferEntryType latest_result;
      if (!core_logic_.buffer_.get_latest_sensor_handle_state(pose2_sensor_sptr_, &latest_result))
      {
        continue;
      };

      mars::VisionSensorData pose_data = *static_cast<mars::VisionSensorData*>(latest_result.data_.sensor_.get());

      // Write Sensor State
      ofile_pose2 << pose_data.state_.to_csv_string(latest_result.timestamp_.get_seconds());

      // Write Core State Covariance
      ofile_pose2 << mars::WriteCsv::cov_mat_to_csv(pose_data.sensor_cov_);

      // Terminate line
      ofile_pose2 << std::endl;
    }

    if (k.sensor_ == baro1_sensor_sptr_)
    {
      mars::BufferEntryType latest_result;
      if (!core_logic_.buffer_.get_latest_sensor_handle_state(baro1_sensor_sptr_, &latest_result))
      {
        continue;
      };

      mars::PressureSensorData baro_data = *static_cast<mars::PressureSensorData*>(latest_result.data_.sensor_.get());

      // Write Sensor State
      ofile_baro1 << baro_data.state_.to_csv_string(latest_result.timestamp_.get_seconds());

      // Write Core State Covariance
      ofile_baro1 << mars::WriteCsv::cov_mat_to_csv(baro_data.sensor_cov_);

      // Terminate line
      ofile_baro1 << std::endl;
    }
  }

  std::cout << "...Completed Filtering Process" << std::endl;

  std::cout << "Closing State Output Files..." << std::endl;
  ofile_core.close();
  ofile_gps1.close();
  ofile_gps1_enu.close();
  ofile_gps2.close();
  ofile_gps2_enu.close();
  ofile_gps3.close();
  ofile_gps3_enu.close();
  ofile_mag1.close();
  ofile_mag2.close();
  ofile_pose1.close();
  ofile_pose2.close();
  ofile_baro1.close();
  std::cout << "... Done" << std::endl;

  return 0;
}
