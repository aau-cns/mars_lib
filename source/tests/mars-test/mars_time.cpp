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
#include <mars/time.h>

class mars_time_test : public testing::Test
{
public:
};

TEST_F(mars_time_test, CTOR)
{
  mars::Time empty_ctor;
  EXPECT_EQ(empty_ctor.get_seconds(), 0.0);

  mars::Time positive_ctor(10);
  EXPECT_EQ(positive_ctor.get_seconds(), 10.0);

  mars::Time negative_ctor(-10);
  EXPECT_EQ(negative_ctor.get_seconds(), -10.0);
}

TEST_F(mars_time_test, OPERATOR)
{
  mars::Time timestamp(15);

  // Logic
  EXPECT_FALSE(timestamp < 15);
  EXPECT_FALSE(timestamp > 15);
  EXPECT_FALSE(timestamp > 16);
  EXPECT_FALSE(timestamp < 14);

  EXPECT_TRUE(timestamp <= 15);
  EXPECT_TRUE(timestamp >= 15);
  EXPECT_TRUE(timestamp == 15);
  EXPECT_TRUE(timestamp < 16);
  EXPECT_TRUE(timestamp > 14);

  // Math
  const double a = 2.1;
  const double b = 3.2;

  mars::Time t1(a);
  mars::Time t2(b);
  EXPECT_EQ(t1 + t2, mars::Time(a + b));
  EXPECT_EQ(t1 - t2, mars::Time(a - b));
  EXPECT_EQ(t2 - t1, mars::Time(b - a));
}

TEST_F(mars_time_test, ABS)
{
  mars::Time positive_ctor(1.3);
  EXPECT_EQ(positive_ctor.abs(), 1.3);

  mars::Time negative_ctor(-1.3);
  EXPECT_EQ(negative_ctor.abs(), 1.3);
}
