// Copyright (C) 2024 Christian Brommer and Thomas Jantos, Control of Networked Systems, University of Klagenfurt,
// Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <christian.brommer@ieee.org>

#ifndef SENSORMANAGER_HPP
#define SENSORMANAGER_HPP

#include <mars/sensors/sensor_abs_class.h>
#include <memory>
#include <vector>

namespace mars
{
class SensorManager
{
public:
  std::vector<std::shared_ptr<SensorAbsClass>> sensor_list_;  ///< Vector containing all registered sensors
  SensorManager() = default;

  ///
  /// \brief register_sensor Register a sensor with the sensor manager
  /// \param sensor Sensor to be registered
  /// \return True if the sensor was registered, false if the sensor is already registered
  ///
  bool register_sensor(std::shared_ptr<SensorAbsClass> sensor)
  {
    // Check if sensor already exists
    if (std::find(sensor_list_.begin(), sensor_list_.end(), sensor) != sensor_list_.end())
    {
      // Sensor is already registered
      return false;
    }

    sensor_list_.push_back(sensor);
    std::cout << "Registered sensor [" << sensor->name_ << "] with Sensor Manager" << std::endl;
    return true;
  }

  ///
  /// \brief remove_sensor Remove a sensor from the sensor manager
  /// \param buffer Buffer to remove the sensor from
  /// \param sensor Sensor to be removed
  /// \return True if the sensor was removed, false if the sensor is not registered
  ///
  bool remove_sensor(Buffer& buffer, std::shared_ptr<SensorAbsClass> sensor)
  {
    if (!does_sensor_exist(sensor))
    {
      // Sensor is not registered
      return false;
    }
    // Deactive the sensor
    this->deactivate_sensor(buffer, sensor);
    // Remove the sensor from the list
    sensor_list_.erase(std::remove(sensor_list_.begin(), sensor_list_.end(), sensor), sensor_list_.end());
    std::cout << "Removed sensor [" << sensor->name_ << "] from Sensor Manager" << std::endl;
    return true;
  }

  ///
  /// \brief list_sensors Print the information of all registered sensors
  ///
  void list_sensors()
  {
    std::cout << "Sensor Manager contains " << sensor_list_.size() << " sensors" << std::endl;
    for (auto& sensor : sensor_list_)
    {
      std::cout << *sensor << std::endl;
    }
  }

  ///
  /// \brief does_sensor_exist Check if a sensor is registered
  /// \param sensor Sensor to be checked
  /// \return True if the sensor is registered, otherwise false
  ///
  bool does_sensor_exist(std::shared_ptr<SensorAbsClass> sensor)
  {
    if (std::find(sensor_list_.begin(), sensor_list_.end(), sensor) != sensor_list_.end())
    {
      return true;
    }
    return false;
  }

  ///
  /// \brief deactivate_sensor Deactivate a sensor
  /// \param buffer Buffer to remove the sensor from
  /// \param sensor Sensor to be deactivated
  /// \return False if the sensor is not registered, otherwise true
  ///
  bool deactivate_sensor(Buffer& buffer, std::shared_ptr<SensorAbsClass> sensor)
  {
    if (!does_sensor_exist(sensor))
    {
      // Sensor is not registered
      return false;
    }

    // Reset the sensor
    sensor->do_update_ = false;
    sensor->is_initialized_ = false;
    sensor->ref_to_nav_given_ = false;

    // Call buffer to clear all entries of the sensor
    if (buffer.RemoveSensorFromBuffer(sensor))
    {
      std::cout << "Removed sensor [" << sensor->name_ << "] from buffer" << std::endl;
    }
    else
    {
      std::cout << "Could not remove sensor [" << sensor->name_ << "] from buffer as buffer is empty" << std::endl;
      return false;
    }
    return true;
  }

  ///
  /// \brief activate_sensor Activate a sensor
  /// \param sensor Sensor to be activated
  /// \return False if the sensor is not registered, otherwise true
  ///
  bool activate_sensor(std::shared_ptr<SensorAbsClass> sensor)
  {
    if (!does_sensor_exist(sensor))
    {
      // Sensor is not registered
      return false;
    }

    sensor->do_update_ = true;
    std::cout << "Activated sensor [" << sensor->name_ << "]" << std::endl;
    return true;
  }
};
}  // namespace mars

#endif  // SENSORMANAGER_HPP
