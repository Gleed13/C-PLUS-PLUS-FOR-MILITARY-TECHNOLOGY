#include "ballistics.hpp"

#include <gtest/gtest.h>

TEST(Ballistics, ComputesKnownDropPoint)
{
  const InputData input{
    .xd = 100.0,
    .yd = 100.0,
    .zd = 100.0,
    .xt = 200.0,
    .yt = 200.0,
    .attackSpeed = 10.0,
    .accelerationPath = 10.0,
    .ammoName = "VOG-17",
  };

  OutputData output;
  bool is_failed = ComputeDropSolution(&input, &output);

  EXPECT_FALSE(is_failed);
  EXPECT_NEAR(output.fireX, 173.759, 0.01);
  EXPECT_NEAR(output.fireY, 173.759, 0.01);
}

TEST(Ballistics, HandlesUnknownAmmoType)
{
  const InputData input{
    .xd = 100.0,
    .yd = 100.0,
    .zd = 100.0,
    .xt = 200.0,
    .yt = 200.0,
    .attackSpeed = 10.0,
    .accelerationPath = 10.0,
    .ammoName = "UnknownAmmo",
  };

  OutputData output;
  bool is_failed = ComputeDropSolution(&input, &output);

  EXPECT_TRUE(is_failed);
}

TEST(Ballistics, HandlesZeroHorizontalDistance)
{
  const InputData input{
    .xd = 100.0,
    .yd = 100.0,
    .zd = 100.0,
    .xt = 100.0,
    .yt = 100.0,
    .attackSpeed = 10.0,
    .accelerationPath = 10.0,
    .ammoName = "VOG-17",
  };

  OutputData output;
  bool is_failed = ComputeDropSolution(&input, &output);

  EXPECT_TRUE(is_failed);
}