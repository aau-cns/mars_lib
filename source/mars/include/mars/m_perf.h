// Copyright (C) 2022 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <christian.brommer@ieee.org>

#ifndef M_PERF_H
#define M_PERF_H

#include <chrono>
#include <iostream>
#include <map>
#include <memory>
#include <vector>

namespace mars
{
///
/// \brief The MPerfType class Class of performance entry types
///
/// Represents one profiling entity and handles time tracking and statistical operations
///
class MPerfType
{
public:
  ///
  /// \brief AddStart Adds start time to time buffer of the current instance
  /// \return
  ///
  bool AddStart();

  ///
  /// \brief AddStop Adds stop time to time buffer of the current instance
  /// \return
  ///
  bool AddStop();

  ///
  /// \brief get_start_times Returns a full vector of all start times
  /// \return
  ///
  std::vector<double> get_start_times();
  ///
  /// \brief get_stop_times Returns a full vector of all stop times
  /// \return
  ///
  std::vector<double> get_stop_times();

  ///
  /// \brief get_mean Returns the mean of the duration times of the current instance
  /// \return
  ///
  double get_mean();

  ///
  /// \brief get_std Returns the std of the mean for the duration times of the current instance
  /// \return
  ///
  double get_std();

  ///
  /// \brief get_max Returns the max of the duration times of the current instance
  /// \return
  ///
  double get_max();

  ///
  /// \brief get_min Returns the min of the duration times of the current instance
  /// \return
  ///
  double get_min();

  ///
  /// \brief get_diff_vec Returns a vector with durations for all entries
  /// \return
  ///
  std::vector<double> get_diff_vec();

  ///
  /// \brief get_size Gets the size of all start stop time combinations
  /// \return
  ///
  int get_size();

private:
  ///
  /// \brief time_type Type which the timer isntance returns
  ///
  using time_type = std::chrono::high_resolution_clock::time_point;

  ///
  /// \brief name_ Name of the current tracking instance
  ///
  std::string name_;

  ///
  /// \brief start_ Vector of tracked starting times
  ///
  std::vector<time_type> start_;

  ///
  /// \brief stop_ Vector of tracked stopping times
  ///
  std::vector<time_type> stop_;

  ///
  /// \brief is_running_ Indicator if the current instance is already tracking a duration
  ///
  bool is_running_{ false };

  ///
  /// \brief get_time Get current time
  /// \return
  ///
  time_type get_time();
};

///
/// \brief The MPerf class Class which hosts individual time tracking instances
///
/// This class also handles the print and export of time tracking stats
///
class MPerf
{
public:
  MPerf(const std::string& name);
  ~MPerf();

  ///
  /// \brief StartEntity Starts/Adds an entity to be tracked
  /// \param entity_name Name of the entity
  /// \return
  ///
  bool StartEntity(const std::string& entity_name);

  ///
  /// \brief StopEntity Stops an entity that was tracked
  /// \param entity_name Name of the entity
  /// \return
  ///
  bool StopEntity(const std::string& entity_name);

  ///
  /// \brief PrintStats Prints the stats of all entities
  /// \return
  ///
  std::string PrintStats();

  ///
  /// \brief get_stats_as_csv Provide stats of all entities in form of a csv string
  /// \param entity_name
  /// \return
  ///
  std::string get_stats_as_csv(const std::string& entity_name);

  ///
  /// \brief get_entity_names Print the names of all registered entities
  /// \return
  ///
  std::string get_entity_names();

private:
  ///
  /// \brief m_perf_map Type of the map which hosts time tracking elements
  ///
  using m_perf_map = std::map<std::string, std::shared_ptr<MPerfType>>;

  ///
  /// \brief data_ Map of all time tracked elements associated by map key
  ///
  m_perf_map data_;

  ///
  /// \brief name_ Name of the performance tracking instance
  ///
  std::string name_;

  ///
  /// \brief name_set_ Indicator if a name for the tracker was set
  ///
  bool name_set_{ false };

  ///
  /// \brief verbose_ Increased console output
  ///
  bool verbose_{ false };
};
}
#endif  // M_PERF_H
