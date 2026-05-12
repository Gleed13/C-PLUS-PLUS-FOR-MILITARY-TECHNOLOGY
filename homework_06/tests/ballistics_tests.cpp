#include "ballistics.hpp"

#include <gtest/gtest.h>

TEST(Ballistics, ComputesKnownDropPoint)
{
  const InputData kInput{
    .xd_ = 100.0,
    .yd_ = 100.0,
    .zd_ = 100.0,
    .xt_ = 200.0,
    .yt_ = 200.0,
    .attack_speed_ = 10.0,
    .acceleration_path_ = 10.0,
    .ammo_name_ = "VOG-17",
  };

  OutputData output{};
  int result = compute_drop_solution(&kInput, &output);

  EXPECT_EQ(result, 0);
  EXPECT_NEAR(output.fire_x_, 173.759, 0.01);
  EXPECT_NEAR(output.fire_y_, 173.759, 0.01);
}

TEST(Ballistics, HandlesUnknownAmmoType)
{
  const InputData kInput{
    .xd_ = 100.0,
    .yd_ = 100.0,
    .zd_ = 100.0,
    .xt_ = 200.0,
    .yt_ = 200.0,
    .attack_speed_ = 10.0,
    .acceleration_path_ = 10.0,
    .ammo_name_ = "UnknownAmmo",
  };

  OutputData output{};
  int result = compute_drop_solution(&kInput, &output);

  EXPECT_NE(result, 0);
}

TEST(Ballistics, HandlesZeroHorizontalDistance)
{
  const InputData kInput{
    .xd_ = 100.0,
    .yd_ = 100.0,
    .zd_ = 100.0,
    .xt_ = 100.0,
    .yt_ = 100.0,
    .attack_speed_ = 10.0,
    .acceleration_path_ = 10.0,
    .ammo_name_ = "VOG-17",
  };

  OutputData output{};
  int result = compute_drop_solution(&kInput, &output);

  EXPECT_NE(result, 0);
}