// Copyright (C) 2021 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <christian.brommer@ieee.org>

#include <gmock/gmock.h>
#include <mars/m_perf.h>

class mars_m_perf_test : public testing::Test
{
public:
};

TEST_F(mars_m_perf_test, CTOR)
{
  mars::MPerf m_perf("Test Case");

  m_perf.StartEntity("cb");
  m_perf.StartEntity("ab");
  m_perf.StartEntity("bb");
  m_perf.StopEntity("cb");
  m_perf.StopEntity("ab");
  m_perf.StopEntity("bb");

  m_perf.StartEntity("cb");
  m_perf.StartEntity("ab");
  m_perf.StartEntity("bb");
  m_perf.StopEntity("cb");
  m_perf.StopEntity("ab");
  m_perf.StopEntity("bb");

  m_perf.StartEntity("cb");
  m_perf.StartEntity("ab");
  m_perf.StartEntity("bb");
  m_perf.StopEntity("cb");
  m_perf.StopEntity("ab");
  m_perf.StopEntity("bb");

  m_perf.StartEntity("cb");
  m_perf.StartEntity("ab");
  m_perf.StartEntity("bb");
  m_perf.StopEntity("cb");
  m_perf.StopEntity("ab");
  m_perf.StopEntity("bb");

  m_perf.PrintStats();

  std::cout << "List of Entries \n" << m_perf.get_entity_names() << std::endl;
}
