// Copyright (C) 2021 Martin Scheiber, Christian Brommer,
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

#include "mars/sensors/gps/gps_utils.h"

mars::GPSInit::GPSInit(const double& init_duration) : init_duration_(init_duration)
{
  if (b_verbose_)
  {
    std::cout << "[GPSInit]: set init duration to " << init_duration << " s." << std::endl;
  }
}

void mars::GPSInit::Reset()
{
  b_is_initialized_ = false;
}

void mars::GPSInit::set_done()
{
  b_is_initialized_ = true;
}

mars::GpsCoordinates mars::GPSInit::get_gps_mean(const std::shared_ptr<mars::SensorAbsClass>& sensor_handle,
                                                 const mars::Buffer& buffer, const GpsCoordinates& cur_meas,
                                                 const mars::Time& cur_time)
{
  if (b_verbose_)
  {
    std::cout << "[GPSInit]: trying to initialize GPS" << std::endl;
  }

  // if the init duration is smaller than 0.0 then only use 'current' measurement
  if (init_duration_ < 0.0)
  {
    std::cout << "Warning: [GPSInit] Init duration was negative but corrected to zero" << std::endl;
    b_is_initialized_ = true;
    return cur_meas;
  }

  // get ordered values from buffer
  std::vector<const mars::BufferEntryType*> gps_entries;
  buffer.get_sensor_handle_measurements(sensor_handle, &gps_entries);

  // check if enough measurements where recorded
  if (gps_entries.empty() || (cur_time - gps_entries.at(0)->timestamp_).get_seconds() < init_duration_)
  {
    if (b_verbose_)
    {
      std::cout << "[GPSInit]: could not init sensor (empty? " << gps_entries.empty() << ")" << std::endl;
    }
    return cur_meas;
  }

  // calcualte average pressure/height
  GpsCoordinates avg_ref = cur_meas;
  uint cnt_meas = 1;
  for (std::vector<const mars::BufferEntryType*>::reverse_iterator it = gps_entries.rbegin(); it != gps_entries.rend();
       ++it)
  {
    if ((cur_time - (*it)->timestamp_).get_seconds() <= init_duration_)
    {
      const GpsCoordinates meas = (*static_cast<GpsMeasurementType*>((*it)->data_.sensor_.get())).coordinates_;

      avg_ref += meas;
      cnt_meas++;
    }
    else
    {
      // in this case all other entries will not fullfill requirement of buffer so break early
      break;
    }
  }

  // compute average
  avg_ref /= cnt_meas;
  b_is_initialized_ = true;

  if (b_verbose_)
  {
    std::cout << "[GPSInit]: finished initialization" << std::endl;
  }

  return avg_ref;
}

bool mars::GPSInit::IsDone() const
{
  return b_is_initialized_;
}
