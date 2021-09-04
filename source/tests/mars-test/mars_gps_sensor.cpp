
#include <gmock/gmock.h>
#include <mars/sensors/gps/gps_measurement_type.h>
#include <mars/sensors/gps/gps_sensor_class.h>
#include <mars/type_definitions/base_states.h>
#include <mars/type_definitions/buffer_data_type.h>
#include <mars/type_definitions/buffer_entry_type.h>
#include <Eigen/Dense>

class mars_gps_sensor_test : public testing::Test
{
public:
};

TEST_F(mars_gps_sensor_test, CTOR_GPS_SENSOR)
{
  mars::CoreState core_states;
  std::shared_ptr<mars::CoreState> core_states_sptr = std::make_shared<mars::CoreState>(core_states);
  mars::GpsSensorClass gps_sensor("GPS", core_states_sptr);
}

TEST_F(mars_gps_sensor_test, GPS_SENSOR_MEASUREMENT)
{
  Eigen::Vector3d position;  // Position [x y z]

  position << 1, 2, 3;
  mars::GpsMeasurementType b(position);

  Eigen::IOFormat OctaveFmt(Eigen::StreamPrecision, 0, ", ", ";\n", "", "", "[", "]");
  std::cout << b.position_.format(OctaveFmt) << std::endl;
}

TEST_F(mars_gps_sensor_test, GPS_SENSOR_INIT)
{
  Eigen::Vector3d position;  // Position [x y z]

  position << 1, 2, 3;
  mars::GpsMeasurementType measurement(position);

  mars::CoreState core_states;
  std::shared_ptr<mars::CoreState> core_states_sptr = std::make_shared<mars::CoreState>(core_states);
  mars::GpsSensorClass gps_sensor("GPS", core_states_sptr);

  gps_sensor.Initialize(1, std::make_shared<mars::GpsMeasurementType>(measurement),
                             std::make_shared<mars::CoreType>());
}

TEST_F(mars_gps_sensor_test, GPS_UPDATE)
{
  Eigen::Vector3d position;  // Position [x y z]

  position << 1, 2, 3;
  mars::GpsMeasurementType measurement(position);

  mars::CoreState core_states;
  std::shared_ptr<mars::CoreState> core_states_sptr = std::make_shared<mars::CoreState>(core_states);
  mars::GpsSensorClass gps_sensor("GPS", core_states_sptr);

  int timestamp = 1;
  mars::CoreStateType prior_core_state;
  mars::BufferDataType prior_sensor_buffer_data;
  Eigen::Matrix<double, prior_core_state.size_error_ + 3, prior_core_state.size_error_ + 3> prior_cov;
  prior_cov.setIdentity();

  mars::BufferDataType test;
  gps_sensor.CalcUpdate(timestamp, std::make_shared<mars::GpsMeasurementType>(measurement), prior_core_state,
                             prior_sensor_buffer_data.sensor_, prior_cov, &test);
}
