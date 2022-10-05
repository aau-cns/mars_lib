// Copyright (C) 2021 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <christian.brommer@ieee.org>

#ifndef BUFFERDATATYPE_H
#define BUFFERDATATYPE_H

#include <memory>
#include <utility>

namespace mars
{
/// \brief The BufferDataType binds the core and sensor state in form of a shared void pointer.
///
/// This shared pointer is of type void to allow generalized storage of different sensor and core data types.
/// The need for the versatility of data types is caused by the fact that sensor measurements can not be stored with a
/// common base class since the input and output parameter differ for each class.
/// A high-level class such as the PoseSensorClass needs to keep track of the type such that it can cast the data before
/// it is used.
///
/// \attention Shared pointer should be passed by value to ensure the correct reference count. Further, using the move
/// operator saves two internal atomic copys of the shared pointer and is more efficient.
///
/// \remarks Implementation: Done, Testing: Open
///
class BufferDataType
{
public:
  std::shared_ptr<void> core_{ nullptr };    ///< Core data
  std::shared_ptr<void> sensor_{ nullptr };  ///< Sensor data

  BufferDataType() = default;

  ///
  /// \brief BufferDataType
  /// \param core this data field holds the core state data
  /// \param sensor this data field holds the sensor state or measurement data
  ///
  /// \note Return/pass smart pointers by value
  ///
  BufferDataType(std::shared_ptr<void> core, std::shared_ptr<void> sensor) : core_(move(core)), sensor_(move(sensor))
  {
  }

  void set_core_data(std::shared_ptr<void> core)
  {
    core_ = std::move(core);
  }

  void set_sensor_data(std::shared_ptr<void> sensor)
  {
    sensor_ = std::move(sensor);
  }
};
}  // namespace mars
#endif  // BUFFERDATATYPE_H
