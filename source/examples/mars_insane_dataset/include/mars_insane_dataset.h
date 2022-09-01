#ifndef MARS_INSANE_DATASET_H
#define MARS_INSANE_DATASET_H

#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <cmath>
#include <iomanip>
#include <iostream>

class ParamLoad
{
public:
  std::string data_path_;
  std::string result_dir_pref_;
  std::string imu_file_name_;
  std::string mag1_file_name_;
  std::string mag2_file_name_;
  std::string gps1_file_name_;
  std::string gps2_file_name_;
  std::string gps3_file_name_;
  std::string pose1_file_name_;
  std::string pose2_file_name_;
  std::string baro1_file_name_;

  double t_start_{ 0 };
  double t_stop_{ 0 };

  bool enable_gps1_{ true };
  double t_offset_gps1_{ 0 };
  bool enable_gps2_{ true };
  double t_offset_gps2_{ 0 };
  bool enable_gps3_{ true };
  double t_offset_gps3_{ 0 };
  bool enable_mag1_{ true };
  double t_offset_mag1_{ 0 };
  bool update_on_mag1_{ false };
  bool enable_mag2_{ true };
  double t_offset_mag2_{ 0 };
  bool update_on_mag2_{ false };
  bool enable_pose1_{ true };
  double t_offset_pose1_{ 0 };
  bool enable_pose2_{ true };
  double t_offset_pose2_{ 0 };
  bool enable_baro1_{ true };
  double t_offset_baro1_{ 0 };

  bool publish_on_propagation_{ false };   ///< Set true to publish the core state on propagation
  bool use_ros_time_now_{ false };         ///< Set to true to use rostime now for all sensor updates
  bool verbose_output_{ false };           ///< If true, all verbose infos are printed
  bool verbose_ooo_{ false };              ///< If true, only out of order verbose msgs are printed
  bool discard_ooo_prop_meas_{ false };    ///< If true, all out of order propagation sensor meas are discarded
  bool use_common_gps_reference_{ true };  ///< Use a common GPS reference for all sensors
  bool cov_debug_{ false };
  int buffer_size_{ 2000 };  ///< Set mars buffersize

  bool enable_manual_yaw_init_{ false };
  double yaw_init_deg_{ 0 };
  int auto_mag_init_samples_{ 30 };

  bool publish_gps_enu_{ false };

  double g_rate_noise_;
  double g_bias_noise_;
  double a_noise_;
  double a_bias_noise_;

  bool disable_acc_bias_{ false };
  bool disable_gyro_bias_{ false };

  Eigen::Vector3d core_init_cov_p_;
  Eigen::Vector3d core_init_cov_v_;
  Eigen::Vector3d core_init_cov_q_;
  Eigen::Vector3d core_init_cov_bw_;
  Eigen::Vector3d core_init_cov_ba_;

  bool use_manual_gps_ref_{ false };
  Eigen::Vector3d manual_gps_ref_;

  bool gps1_chi2_enable_;
  double gps1_chi2_value_;
  Eigen::Vector3d gps1_pos_meas_noise_;
  Eigen::Vector3d gps1_vel_meas_noise_;
  Eigen::Vector3d gps1_cal_ig_;
  Eigen::Vector3d gps1_state_init_cov_;
  bool gps1_use_vel_rot_;
  Eigen::Vector3d gps1_vel_rot_axis_;
  double gps1_vel_rot_thr_;

  bool gps2_chi2_enable_;
  double gps2_chi2_value_;
  Eigen::Vector3d gps2_pos_meas_noise_;
  Eigen::Vector3d gps2_vel_meas_noise_;
  Eigen::Vector3d gps2_cal_ig_;
  Eigen::Vector3d gps2_state_init_cov_;
  bool gps2_use_vel_rot_;
  Eigen::Vector3d gps2_vel_rot_axis_;
  double gps2_vel_rot_thr_;

  bool gps3_chi2_enable_;
  double gps3_chi2_value_;
  Eigen::Vector3d gps3_pos_meas_noise_;
  Eigen::Vector3d gps3_vel_meas_noise_;
  Eigen::Vector3d gps3_cal_ig_;
  Eigen::Vector3d gps3_state_init_cov_;
  bool gps3_use_vel_rot_;
  Eigen::Vector3d gps3_vel_rot_axis_;
  double gps3_vel_rot_thr_;

  bool mag1_normalize_{ true };
  double mag1_decl_{ 0.0 };
  double mag1_incl_{ 0.0 };
  Eigen::Vector3d mag1_meas_noise_;
  Eigen::Quaterniond mag1_cal_q_im_{Eigen::Quaterniond::Identity()};
  Eigen::Matrix<double, 6, 1> mag1_state_init_cov_;
  bool mag1_apply_intrinsic_{ true };
  Eigen::Vector3d mag1_intr_offset_{ Eigen::Vector3d::Zero() };
  Eigen::Matrix3d mag1_intr_transform_{ Eigen::Matrix3d::Identity() };

  bool mag2_normalize_{ true };
  double mag2_decl_{ 0.0 };
  double mag2_incl_{ 0.0 };
  Eigen::Vector3d mag2_meas_noise_;
  Eigen::Quaterniond mag2_cal_q_im_{Eigen::Quaterniond::Identity()};
  Eigen::Matrix<double, 6, 1> mag2_state_init_cov_;
  bool mag2_apply_intrinsic_{ true };
  Eigen::Vector3d mag2_intr_offset_{ Eigen::Vector3d::Zero() };
  Eigen::Matrix3d mag2_intr_transform_{ Eigen::Matrix3d::Identity() };

  Eigen::Vector3d pose1_pos_meas_noise_;
  Eigen::Vector3d pose1_rot_meas_noise_;
  Eigen::Vector3d pose1_cal_p_ip_;
  Eigen::Quaterniond pose1_cal_q_ip_{Eigen::Quaterniond::Identity()};
  bool pose1_use_scale_{ false };
  Eigen::Matrix<double, 6, 1> pose1_state_init_cov_;

  Eigen::Vector3d pose2_pos_meas_noise_;
  Eigen::Vector3d pose2_rot_meas_noise_;
  Eigen::Vector3d pose2_cal_p_ip_;
  Eigen::Quaterniond pose2_cal_q_ip_{Eigen::Quaterniond::Identity()};
  bool pose2_use_scale_{ false };
  Eigen::Matrix<double, 6, 1> pose2_state_init_cov_;

  double baro1_meas_noise_;
  Eigen::Vector3d baro1_cal_p_ip_;
  Eigen::Matrix<double, 4, 1> baro1_state_init_cov_;

  Eigen::IOFormat HeavyFmt{ Eigen::FullPrecision, 0, ", ", ";\n", "[", "]", "[", "]" };

  ParamLoad(const std::string& test_data_path)
  {
    std::cout << "Read YAML Parameter from " << test_data_path << std::endl;
    YAML::Node config = YAML::LoadFile(test_data_path);

    read_yaml_str(&data_path_, "data_path", config);
    data_path_ = data_path_ + "/";
    read_yaml_str(&result_dir_pref_, "result_dir_pref", config);
    result_dir_pref_ = result_dir_pref_ + "/";

    read_yaml_str(&imu_file_name_, "imu_file_name", config);
    read_yaml_str(&mag1_file_name_, "mag1_file_name", config);
    read_yaml_str(&mag2_file_name_, "mag2_file_name", config);
    read_yaml_str(&gps1_file_name_, "gps1_file_name", config);
    read_yaml_str(&gps2_file_name_, "gps2_file_name", config);
    read_yaml_str(&gps3_file_name_, "gps3_file_name", config);
    read_yaml_str(&pose1_file_name_, "pose1_file_name", config);
    read_yaml_str(&pose2_file_name_, "pose2_file_name", config);
    read_yaml_str(&baro1_file_name_, "baro1_file_name", config);

    read_yaml_double(&t_start_, "t_start", config);
    read_yaml_double(&t_stop_, "t_stop", config);

    read_yaml_bool(&enable_gps1_, "enable_gps1", config);
    read_yaml_double(&t_offset_gps1_, "t_offset_gps1", config);
    read_yaml_bool(&enable_gps2_, "enable_gps2", config);
    read_yaml_double(&t_offset_gps2_, "t_offset_gps2", config);
    read_yaml_bool(&enable_gps3_, "enable_gps3", config);
    read_yaml_double(&t_offset_gps3_, "t_offset_gps3", config);
    read_yaml_bool(&enable_mag1_, "enable_mag1", config);
    read_yaml_double(&t_offset_mag1_, "t_offset_mag1", config);
    read_yaml_bool(&update_on_mag1_, "update_on_mag1", config);
    read_yaml_bool(&enable_mag2_, "enable_mag2", config);
    read_yaml_double(&t_offset_mag2_, "t_offset_mag2", config);
    read_yaml_bool(&update_on_mag2_, "update_on_mag2", config);
    read_yaml_bool(&enable_pose1_, "enable_pose1", config);
    read_yaml_double(&t_offset_pose1_, "t_offset_pose1", config);
    read_yaml_bool(&enable_pose2_, "enable_pose2", config);
    read_yaml_double(&t_offset_pose2_, "t_offset_pose2", config);
    read_yaml_bool(&enable_baro1_, "enable_baro1", config);
    read_yaml_double(&t_offset_baro1_, "t_offset_baro1", config);

    read_yaml_bool(&publish_on_propagation_, "pub_on_prop", config);
    read_yaml_bool(&use_ros_time_now_, "use_ros_time_now", config);
    read_yaml_bool(&verbose_output_, "verbose", config);
    read_yaml_bool(&verbose_ooo_, "verbose_out_of_order", config);
    read_yaml_bool(&discard_ooo_prop_meas_, "discard_ooo_prop_meas", config);
    read_yaml_bool(&cov_debug_, "cov_debug", config);
    read_yaml_int(&buffer_size_, "buffer_size", config);

    read_yaml_bool(&enable_manual_yaw_init_, "enable_manual_yaw_init", config);
    read_yaml_double(&yaw_init_deg_, "yaw_init_deg", config);
    read_yaml_int(&auto_mag_init_samples_, "auto_mag_init_samples", config);

    read_yaml_bool(&use_common_gps_reference_, "use_common_gps_reference", config);
    read_yaml_bool(&publish_gps_enu_, "publish_gps_enu", config);

    read_yaml_double(&g_rate_noise_, "gyro_rate_noise", config);
    read_yaml_double(&g_bias_noise_, "gyro_bias_noise", config);
    read_yaml_double(&a_noise_, "acc_noise", config);
    read_yaml_double(&a_bias_noise_, "acc_bias_noise", config);

    read_yaml_bool(&disable_acc_bias_, "disable_acc_bias", config);
    read_yaml_bool(&disable_gyro_bias_, "disable_gyro_bias", config);

    read_yaml_vec_3(&core_init_cov_p_, "core_init_cov_p", config);
    read_yaml_vec_3(&core_init_cov_v_, "core_init_cov_v", config);
    read_yaml_vec_3(&core_init_cov_q_, "core_init_cov_q", config);
    read_yaml_vec_3(&core_init_cov_bw_, "core_init_cov_bw", config);
    read_yaml_vec_3(&core_init_cov_ba_, "core_init_cov_ba", config);

    read_yaml_bool(&use_manual_gps_ref_, "use_manual_gps_ref", config);
    read_yaml_vec_3(&manual_gps_ref_, "manual_gps_ref", config);

    // GPS1
    read_yaml_bool(&gps1_chi2_enable_, "gps1_chi2_enable", config);
    read_yaml_double(&gps1_chi2_value_, "gps1_chi2_value", config);
    read_yaml_vec_3(&gps1_pos_meas_noise_, "gps1_pos_meas_noise", config);
    read_yaml_vec_3(&gps1_vel_meas_noise_, "gps1_vel_meas_noise", config);
    read_yaml_vec_3(&gps1_cal_ig_, "gps1_cal_ig", config);
    read_yaml_vec_3(&gps1_state_init_cov_, "gps1_state_init_cov", config);
    read_yaml_bool(&gps1_use_vel_rot_, "gps1_use_vel_rot", config);
    read_yaml_vec_3(&gps1_vel_rot_axis_, "gps1_vel_rot_axis", config);
    read_yaml_double(&gps1_vel_rot_thr_, "gps1_vel_rot_thr", config);

    // GPS2
    read_yaml_bool(&gps2_chi2_enable_, "gps2_chi2_enable", config);
    read_yaml_double(&gps2_chi2_value_, "gps2_chi2_value", config);
    read_yaml_vec_3(&gps2_pos_meas_noise_, "gps2_pos_meas_noise", config);
    read_yaml_vec_3(&gps2_vel_meas_noise_, "gps2_vel_meas_noise", config);
    read_yaml_vec_3(&gps2_cal_ig_, "gps2_cal_ig", config);
    read_yaml_vec_3(&gps2_state_init_cov_, "gps2_state_init_cov", config);
    read_yaml_bool(&gps2_use_vel_rot_, "gps2_use_vel_rot", config);
    read_yaml_vec_3(&gps2_vel_rot_axis_, "gps2_vel_rot_axis", config);
    read_yaml_double(&gps2_vel_rot_thr_, "gps2_vel_rot_thr", config);

    // GPS3
    read_yaml_bool(&gps3_chi2_enable_, "gps3_chi2_enable", config);
    read_yaml_double(&gps3_chi2_value_, "gps3_chi2_value", config);
    read_yaml_vec_3(&gps3_pos_meas_noise_, "gps3_pos_meas_noise", config);
    read_yaml_vec_3(&gps3_vel_meas_noise_, "gps3_vel_meas_noise", config);
    read_yaml_vec_3(&gps3_cal_ig_, "gps3_cal_ig", config);
    read_yaml_vec_3(&gps3_state_init_cov_, "gps3_state_init_cov", config);
    read_yaml_bool(&gps3_use_vel_rot_, "gps3_use_vel_rot", config);
    read_yaml_vec_3(&gps3_vel_rot_axis_, "gps3_vel_rot_axis", config);
    read_yaml_double(&gps3_vel_rot_thr_, "gps3_vel_rot_thr", config);

    // Mag1
    read_yaml_bool(&mag1_normalize_, "mag1_normalize", config);
    read_yaml_double(&mag1_decl_, "mag1_decl", config);
    read_yaml_double(&mag1_incl_, "mag1_incl", config);
    read_yaml_vec_3(&mag1_meas_noise_, "mag1_meas_noise", config);
    read_yaml_quaternion(&mag1_cal_q_im_, "mag1_cal_q_im", config);
    read_yaml_vec_6(&mag1_state_init_cov_, "mag1_state_init_cov", config);
    read_yaml_bool(&mag1_apply_intrinsic_, "mag1_apply_intrinsic", config);
    read_yaml_vec_3(&mag1_intr_offset_, "mag1_intr_offset", config);
    read_yaml_mat_3(&mag1_intr_transform_, "mag1_intr_transform", config);

    // Mag2
    read_yaml_bool(&mag2_normalize_, "mag2_normalize", config);
    read_yaml_double(&mag2_decl_, "mag2_decl", config);
    read_yaml_double(&mag2_incl_, "mag2_incl", config);
    read_yaml_vec_3(&mag2_meas_noise_, "mag2_meas_noise", config);
    read_yaml_quaternion(&mag2_cal_q_im_, "mag2_cal_q_im", config);
    read_yaml_vec_6(&mag2_state_init_cov_, "mag2_state_init_cov", config);
    read_yaml_bool(&mag2_apply_intrinsic_, "mag2_apply_intrinsic", config);
    read_yaml_vec_3(&mag2_intr_offset_, "mag2_intr_offset", config);
    read_yaml_mat_3(&mag2_intr_transform_, "mag2_intr_transform", config);

    // Pose1
    read_yaml_vec_3(&pose1_pos_meas_noise_, "pose1_pos_meas_noise", config);
    read_yaml_vec_3(&pose1_rot_meas_noise_, "pose1_rot_meas_noise", config);
    read_yaml_vec_3(&pose1_cal_p_ip_, "pose1_cal_p_ip", config);
    read_yaml_quaternion(&pose1_cal_q_ip_, "pose1_cal_q_ip", config);
    read_yaml_bool(&pose1_use_scale_, "pose1_use_scale", config);
    read_yaml_vec_6(&pose1_state_init_cov_, "pose1_state_init_cov", config);

    // Pose2
    read_yaml_vec_3(&pose2_pos_meas_noise_, "pose2_pos_meas_noise", config);
    read_yaml_vec_3(&pose2_rot_meas_noise_, "pose2_rot_meas_noise", config);
    read_yaml_vec_3(&pose2_cal_p_ip_, "pose2_cal_p_ip", config);
    read_yaml_quaternion(&pose2_cal_q_ip_, "pose2_cal_q_ip", config);
    read_yaml_bool(&pose2_use_scale_, "pose2_use_scale", config);
    read_yaml_vec_6(&pose2_state_init_cov_, "pose2_state_init_cov", config);

    // Baro1
    read_yaml_double(&baro1_meas_noise_, "baro1_meas_noise", config);
    read_yaml_vec_3(&baro1_cal_p_ip_, "baro1_cal_p_ip", config);
    read_yaml_vec_4(&baro1_state_init_cov_, "baro1_state_init_cov", config);
  }

private:
  bool read_yaml_vec_3(Eigen::Vector3d* value, const std::string& parameter, YAML::Node config)
  {
    if (config[parameter])
    {
      std::vector<double> vec;
      vec = config[parameter].as<std::vector<double>>();

      *value = Eigen::Vector3d(vec.data());

      std::cout << parameter << ": \n [ ";
      for (auto const& i : vec)
        std::cout << i << " ";

      std::cout << "]" << std::endl;
      return true;
    }
    return false;
  }

  bool read_yaml_vec_4(Eigen::Matrix<double, 4, 1>* value, const std::string& parameter, YAML::Node config)
  {
    if (config[parameter])
    {
      std::vector<double> vec;
      vec = config[parameter].as<std::vector<double>>();

      *value = Eigen::Matrix<double, 4, 1>(vec.data());

      std::cout << parameter << ": \n [ ";
      for (auto const& i : vec)
        std::cout << i << " ";

      std::cout << "]" << std::endl;
      return true;
    }
    return false;
  }


  bool read_yaml_vec_6(Eigen::Matrix<double, 6, 1>* value, const std::string& parameter, YAML::Node config)
  {
    if (config[parameter])
    {
      std::vector<double> vec;
      vec = config[parameter].as<std::vector<double>>();

      *value = Eigen::Matrix<double, 6, 1>(vec.data());

      std::cout << parameter << ": \n [ ";
      for (auto const& i : vec)
        std::cout << i << " ";

      std::cout << "]" << std::endl;
      return true;
    }
    return false;
  }

  bool read_yaml_vec_7(Eigen::Matrix<double, 7, 1>* value, const std::string& parameter, YAML::Node config)
  {
    if (config[parameter])
    {
      std::vector<double> vec;
      vec = config[parameter].as<std::vector<double>>();

      *value = Eigen::Matrix<double, 7, 1>(vec.data());

      std::cout << parameter << ": \n [ ";
      for (auto const& i : vec)
        std::cout << i << " ";

      std::cout << "]" << std::endl;
      return true;
    }
    return false;
  }

  bool read_yaml_mat_3(Eigen::Matrix3d* value, const std::string& parameter, YAML::Node config)
  {
    if (config[parameter])
    {
      std::vector<std::vector<double>> vec;
      vec = config[parameter].as<std::vector<std::vector<double>>>();

      Eigen::Matrix3d mat_conversion;
      mat_conversion << Eigen::Vector3d(vec[0].data()).transpose(), Eigen::Vector3d(vec[1].data()).transpose(),
          Eigen::Vector3d(vec[2].data()).transpose();

      *value = mat_conversion;

      std::cout << parameter << ": \n";
      std::cout << value->format(HeavyFmt) << std::endl;

      return true;
    }
    return false;
  }

  bool read_yaml_bool(bool* value, const std::string& parameter, YAML::Node config)
  {
    if (config[parameter])
    {
      *value = config[parameter].as<bool>();

      std::cout << parameter << ": \n [ ";
      if (*value)
      {
        std::cout << "True";
      }
      else
      {
        std::cout << "False";
      }
      std::cout << " ]" << std::endl;
      return true;
    }
    return false;
  }

  bool read_yaml_double(double* value, const std::string& parameter, YAML::Node config)
  {
    if (config[parameter])
    {
      *value = config[parameter].as<double>();
      std::cout << parameter << ": \n [ " << *value << " ]\n";
      return true;
    }
    return false;
  }

  bool read_yaml_int(int* value, const std::string& parameter, YAML::Node config)
  {
    if (config[parameter])
    {
      *value = config[parameter].as<int>();
      std::cout << parameter << ": \n [ " << *value << " ]\n";
      return true;
    }
    return false;
  }

  bool read_yaml_str(std::string* value, const std::string& parameter, YAML::Node config)
  {
    if (config[parameter])
    {
      *value = config[parameter].as<std::string>();

      std::cout << parameter << ": \n [ " << *value << " ]" << std::endl;
      return true;
    }
    return false;
  }

  ///
  /// \brief read_yaml_quaternion
  /// \param value Returning Quaternion
  /// \param parameter Order of the Quaternion entries in the YAML needs to be w,x,y,z
  /// \param config
  /// \return
  ///
  bool read_yaml_quaternion(Eigen::Quaterniond* value, const std::string& parameter, YAML::Node config)
  {
    if (config[parameter])
    {
      std::vector<double> vec;
      vec = config[parameter].as<std::vector<double>>();

      *value = Eigen::Quaterniond(vec[0], vec[1], vec[2], vec[3]).normalized();

      std::cout << parameter << "(w,x,y,z): \n [ " << value->w() << " " << value->x() << " " << value->y() << " "
                << value->z() << "]" << std::endl;

      return true;
    }

    *value = Eigen::Quaterniond::Identity();

    return false;
  }
};

#endif  // MARS_INSANE_DATASET_H
