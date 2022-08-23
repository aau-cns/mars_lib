// Copyright (C) 2022 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <christian.brommer@ieee.org>

#include <mars/m_perf.h>
#include <math.h>
#include <algorithm>
#include <iomanip>
#include <set>

namespace mars
{
MPerf::MPerf(const std::string& name) : name_(name)
{
  name_set_ = true;
}

MPerf::~MPerf()
{
  std::cout << PrintStats() << std::endl;
}

bool MPerf::StartEntity(const std::string& entity_name)
{
  // If element does not exist
  if (data_.find(entity_name) == data_.end())
  {
    // Create element
    data_.insert({ entity_name, std::make_shared<MPerfType>(MPerfType()) });
  }

  // Start Element
  if (!data_[entity_name].get()->AddStart() && verbose_)
  {
    std::cout << "[Warn]: Asked to start timer which was already running - Overwriting existing entry" << std::endl;
  }
  return true;
}

bool MPerf::StopEntity(const std::string& entity_name)
{
  // If element does not exist
  if (data_.find(entity_name) == data_.end())
  {
    if (verbose_)
    {
      std::cout << "[Warn]: Asked to stop profile entity which didn't exist - Ignoring" << std::endl;
    }
    return false;
  }

  // If element exists
  if (!data_[entity_name].get()->AddStop() && verbose_)
  {
    std::cout << "[Warn]: Asked to stop timer which was not running - Ignoring" << std::endl;
  }
  return true;
}

std::string MPerf::PrintStats()
{
  if (name_set_)
  {
    std::cout << "Profiler name: " << name_ << std::endl;
  }

  std::string stat_header("Entity\t\tt_mean[us]\tt_min[us]\tt_max[us]\tt_std[us]\tnum_calls");

  std::cout << stat_header << std::endl;

  m_perf_map::iterator map_it = data_.begin();
  while (map_it != data_.end())
  {
    std::cout << std::setprecision(4);
    std::cout << map_it->first << "\t\t";
    std::cout << map_it->second.get()->get_mean() << "\t";
    std::cout << map_it->second.get()->get_min() << "\t";
    std::cout << map_it->second.get()->get_max() << "\t";
    std::cout << map_it->second.get()->get_std() << "\t";
    std::cout << map_it->second.get()->get_size() << std::endl;
    map_it++;
  }
  return {};
}

std::string MPerf::get_entity_names()
{
  std::set<std::string> entity_names;  // is always sorted
  m_perf_map::iterator map_it = data_.begin();
  while (map_it != data_.end())
  {
    entity_names.insert(map_it->first);
    map_it++;
  }

  // Append elements from sorted list
  std::string entity_list;
  for (const auto& name : entity_names)
  {
    entity_list.append(std::string(name + "\n"));
  }

  return entity_list;
}

bool MPerfType::AddStart()
{
  if (!is_running_)
  {
    start_.push_back(get_time());
    is_running_ = true;
    return true;
  }

  // Start element already existed, overwriting with new start time
  start_.back() = get_time();
  return false;
}

bool MPerfType::AddStop()
{
  if (is_running_)
  {
    stop_.push_back(get_time());
    is_running_ = false;
    return true;
  }

  // Asked to stop profiler, but was not running. Ignoring request.
  return false;
}

double MPerfType::get_mean()
{
  std::vector<double> v_diff = get_diff_vec();
  double accumulated_duration = 0;
  for (int k = 0; k < v_diff.size(); k++)
  {
    accumulated_duration += v_diff.at(k);
  }

  return (accumulated_duration / static_cast<double>(v_diff.size()));
}

double MPerfType::get_max()
{
  std::vector<double> v_diff = get_diff_vec();
  return *max_element(v_diff.begin(), v_diff.end());
}

double MPerfType::get_min()
{
  std::vector<double> v_diff = get_diff_vec();
  return *min_element(v_diff.begin(), v_diff.end());
}

double MPerfType::get_std()
{
  const std::vector<double> v_diff = get_diff_vec();
  const double mean = get_mean();

  double std_accum = 0;
  for (int k = 0; k < v_diff.size(); k++)
  {
    std_accum += pow(v_diff.at(k) - mean, 2);
  }
  const double diff_std = sqrt(std_accum / v_diff.size());

  return diff_std;
}

std::vector<double> MPerfType::get_diff_vec()
{
  std::vector<double> v_diff;

  // Using size of the stop vector in case the start vector has an active, unfinished, time instance
  for (int k = 0; k < stop_.size(); k++)
  {
    const double diff_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(stop_.at(k) - start_.at(k)).count();
    v_diff.push_back(diff_ns / 1000.0);
  }
  return v_diff;
}

int MPerfType::get_size()
{
  return static_cast<int>(stop_.size());
}

MPerfType::time_type MPerfType::get_time()
{
  return std::chrono::high_resolution_clock::now();
}
}
