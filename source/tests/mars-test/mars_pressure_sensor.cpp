// Copyright (C) 2022 Martin Scheiber, Christian Brommer,
// Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the authors at <christian.brommer@ieee.org>
// and <martin.scheiber@ieee.org>

#include <gmock/gmock.h>
#include <mars/sensors/pressure/pressure_conversion.h>
#include <mars/sensors/pressure/pressure_measurement_type.h>
#include <mars/sensors/pressure/pressure_sensor_class.h>
#include <mars/type_definitions/base_states.h>
#include <mars/type_definitions/buffer_data_type.h>
#include <mars/type_definitions/buffer_entry_type.h>
#include <Eigen/Dense>

class mars_pressure_sensor_test : public testing::Test
{
public:
};

TEST_F(mars_pressure_sensor_test, CTOR_PRESSURE_SENSOR)
{
  mars::CoreState core_states;
  std::shared_ptr<mars::CoreState> core_states_sptr = std::make_shared<mars::CoreState>(core_states);
  mars::PressureSensorClass pressure_sensor("Pressure", core_states_sptr);
}

TEST_F(mars_pressure_sensor_test, PRESSURE_SENSOR_MEASUREMENT)
{
  double height = 1000.0;
  double pressure_liquid = 9806650;  // 9810 kPa = 98.1 bar approx 1000m depth
  double pressure_gas = 89874;       // 89.9 kPa approx 1000 m height
                                     // see also https://www.engineeringtoolbox.com/air-altitude-pressure-d_462.html
  double temperature = 293.15;       // standard temperature 20 degC

  // converter
  mars::PressureConversion pressure_conversion_;

  mars::PressureMeasurementType meas_height(height);
  mars::PressureMeasurementType meas_gas(pressure_gas, temperature);
  mars::PressureMeasurementType meas_liquid(pressure_liquid, temperature, mars::Pressure::Type::LIQUID);

  std::cout << "height: " << pressure_conversion_.get_height(meas_height.pressure_) << std::endl;
  std::cout << "gas:    " << pressure_conversion_.get_height(meas_gas.pressure_) << std::endl;
  std::cout << "liquid: " << pressure_conversion_.get_height(meas_liquid.pressure_) << std::endl;

  EXPECT_TRUE(meas_height.pressure_.type_ == mars::Pressure::Type::HEIGHT);
  EXPECT_EQ(static_cast<double>(pressure_conversion_.get_height(meas_height.pressure_)(0)), height);

  EXPECT_TRUE(meas_gas.pressure_.type_ == mars::Pressure::Type::GAS);
  EXPECT_NEAR(static_cast<double>(pressure_conversion_.get_height(meas_gas.pressure_)(0)), height, 40);

  EXPECT_TRUE(meas_liquid.pressure_.type_ == mars::Pressure::Type::LIQUID);
  EXPECT_NEAR(static_cast<double>(pressure_conversion_.get_height(meas_liquid.pressure_)(0)), height, 40);
}

TEST_F(mars_pressure_sensor_test, PRESSURE_SENSOR_INIT)
{
  double pressure_gas = 89874;  // 89.9 kPa approx 1000 m height
                                // see also https://www.engineeringtoolbox.com/air-altitude-pressure-d_462.html
  double temperature = 293.15;  // standard temperature 20 degC

  mars::PressureMeasurementType measurement(pressure_gas, temperature);

  mars::CoreState core_states;
  std::shared_ptr<mars::CoreState> core_states_sptr = std::make_shared<mars::CoreState>(core_states);
  mars::PressureSensorClass pressure_sensor("Pressure", core_states_sptr);

  EXPECT_DEATH(pressure_sensor.Initialize(1, std::make_shared<mars::PressureMeasurementType>(measurement),
                                          std::make_shared<mars::CoreType>()),
               "");
}

TEST_F(mars_pressure_sensor_test, PRESSURE_UPDATE)
{
  double pressure_gas = 89874;  // 89.9 kPa approx 1000 m height
                                // see also https://www.engineeringtoolbox.com/air-altitude-pressure-d_462.html
  double temperature = 293.15;  // standard temperature 20 degC

  mars::PressureMeasurementType measurement(pressure_gas, temperature);

  mars::CoreState core_states;
  std::shared_ptr<mars::CoreState> core_states_sptr = std::make_shared<mars::CoreState>(core_states);
  mars::PressureSensorClass pressure_sensor("Pressure", core_states_sptr);

  mars::CoreStateType prior_core_state;
  mars::BufferDataType prior_sensor_buffer_data;
  Eigen::Matrix<double, prior_core_state.size_error_ + 4, prior_core_state.size_error_ + 4> prior_cov;
  prior_cov.setIdentity();

  // TODO no update without init
  //  int timestamp = 1;
  //  mars::BufferDataType test =
  //      position_sensor.CalcUpdate(timestamp, std::make_shared<mars::PositionMeasurementType>(measurement),
  //                                 prior_core_state, prior_sensor_buffer_data.sensor_, prior_cov);
}
