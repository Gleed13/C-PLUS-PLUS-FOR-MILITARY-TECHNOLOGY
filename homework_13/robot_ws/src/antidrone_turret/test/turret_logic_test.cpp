#include <gtest/gtest.h>

#include "antidrone_turret/actuator_model.hpp"
#include "antidrone_turret/target_decision_logic.hpp"
#include "antidrone_turret/turret_status.hpp"

namespace {

using antidrone_turret::ActuatorState;
using antidrone_turret::DecisionConfig;
using antidrone_turret::GimbalDirection;
using antidrone_turret::ServoDirection;
using antidrone_turret::TargetInput;
using antidrone_turret::TargetState;
using antidrone_turret::TriggerState;
using antidrone_turret::TurretAction;

// Ціль з дефолтними параметрами: видима, надійна, у робочій дистанції.
TargetInput make_valid_target()
{
  auto target = TargetInput{};
  target.visible = true;
  target.x = 320.0F;
  target.y = 240.0F;
  target.distance_m = 25.0F;
  target.confidence = 0.90F;
  return target;
}

// --- оцінка цілі ---

TEST(EvaluateTargetTest, InvisibleTargetIsNone)
{
  auto target = make_valid_target();
  target.visible = false;

  EXPECT_EQ(antidrone_turret::evaluate_target(target, DecisionConfig{}), TargetState::kNone);
}

TEST(EvaluateTargetTest, ConfidenceBelowThresholdIsLowConfidence)
{
  auto target = make_valid_target();
  target.confidence = 0.79F;

  EXPECT_EQ(
    antidrone_turret::evaluate_target(target, DecisionConfig{}),
    TargetState::kLowConfidence);
}

TEST(EvaluateTargetTest, ConfidenceAtThresholdIsLocked)
{
  auto target = make_valid_target();
  target.confidence = 0.80F;

  EXPECT_EQ(antidrone_turret::evaluate_target(target, DecisionConfig{}), TargetState::kLocked);
}

TEST(EvaluateTargetTest, LowConfidenceTargetGivesIdleAndSkip)
{
  auto target = make_valid_target();
  target.confidence = 0.50F;

  const auto status = antidrone_turret::decide(target, DecisionConfig{}, ActuatorState::kReady);

  EXPECT_EQ(status.target_state, TargetState::kLowConfidence);
  EXPECT_EQ(status.action, TurretAction::kIdle);
  EXPECT_EQ(status.trigger_state, TriggerState::kSkip);
}

// --- команда yaw-серво ---

TEST(ServoCommandTest, TargetRightOfCenterTurnsRight)
{
  const auto command = antidrone_turret::make_servo_command(420.0F);

  EXPECT_EQ(command.direction, ServoDirection::kRight);
  EXPECT_FLOAT_EQ(command.target_x, 420.0F);
  EXPECT_FLOAT_EQ(command.error_x, 100.0F);
  EXPECT_GT(command.error_x, 0.0F);
}

TEST(ServoCommandTest, TargetLeftOfCenterTurnsLeft)
{
  const auto command = antidrone_turret::make_servo_command(220.0F);

  EXPECT_EQ(command.direction, ServoDirection::kLeft);
  EXPECT_FLOAT_EQ(command.error_x, -100.0F);
}

TEST(ServoCommandTest, TargetOnCenterHoldsCenter)
{
  const auto command = antidrone_turret::make_servo_command(320.0F);

  EXPECT_EQ(command.direction, ServoDirection::kCenter);
  EXPECT_FLOAT_EQ(command.error_x, 0.0F);
}

// --- команда гімбала ---

TEST(GimbalCommandTest, TargetAboveCenterMovesUp)
{
  const auto command = antidrone_turret::make_gimbal_command(180.0F);

  EXPECT_EQ(command.direction, GimbalDirection::kUp);
  EXPECT_FLOAT_EQ(command.target_y, 180.0F);
  EXPECT_FLOAT_EQ(command.error_y, 60.0F);
  EXPECT_GT(command.error_y, 0.0F);
}

TEST(GimbalCommandTest, TargetBelowCenterMovesDown)
{
  const auto command = antidrone_turret::make_gimbal_command(300.0F);

  EXPECT_EQ(command.direction, GimbalDirection::kDown);
  EXPECT_FLOAT_EQ(command.error_y, -60.0F);
}

TEST(GimbalCommandTest, TargetOnCenterHoldsCenter)
{
  const auto command = antidrone_turret::make_gimbal_command(240.0F);

  EXPECT_EQ(command.direction, GimbalDirection::kCenter);
  EXPECT_FLOAT_EQ(command.error_y, 0.0F);
}

// --- рішення щодо пострілу ---

TEST(TriggerDecisionTest, CloseTargetWithReadyActuatorIsRequested)
{
  EXPECT_EQ(
    antidrone_turret::decide_trigger(
      TargetState::kLocked, 25.0F, DecisionConfig{}, ActuatorState::kReady),
    TriggerState::kRequested);
}

TEST(TriggerDecisionTest, CloseTargetWhileReloadingIsReloading)
{
  EXPECT_EQ(
    antidrone_turret::decide_trigger(
      TargetState::kLocked, 7.0F, DecisionConfig{}, ActuatorState::kReloading),
    TriggerState::kReloading);
}

TEST(TriggerDecisionTest, TargetBeyondMaxDistanceIsSkipped)
{
  EXPECT_EQ(
    antidrone_turret::decide_trigger(
      TargetState::kLocked, 55.0F, DecisionConfig{}, ActuatorState::kReady),
    TriggerState::kSkip);
}

TEST(TriggerDecisionTest, TargetAtMaxDistanceIsStillRequested)
{
  EXPECT_EQ(
    antidrone_turret::decide_trigger(
      TargetState::kLocked, 30.0F, DecisionConfig{}, ActuatorState::kReady),
    TriggerState::kRequested);
}

TEST(TriggerDecisionTest, UnlockedTargetIsNeverTriggered)
{
  EXPECT_EQ(
    antidrone_turret::decide_trigger(
      TargetState::kLowConfidence, 5.0F, DecisionConfig{}, ActuatorState::kReady),
    TriggerState::kSkip);
  EXPECT_EQ(
    antidrone_turret::decide_trigger(
      TargetState::kNone, 5.0F, DecisionConfig{}, ActuatorState::kReady),
    TriggerState::kSkip);
}

// --- складання TurretStatus ---

TEST(TurretStatusTest, InvisibleTargetGivesNoneIdleSkip)
{
  auto target = make_valid_target();
  target.visible = false;

  const auto status = antidrone_turret::decide(target, DecisionConfig{}, ActuatorState::kReady);

  EXPECT_EQ(status.target_state, TargetState::kNone);
  EXPECT_EQ(status.action, TurretAction::kIdle);
  EXPECT_EQ(status.trigger_state, TriggerState::kSkip);
}

// far_flyby_no_trigger.csv: ціль надійна, але задалеко.
TEST(TurretStatusTest, DistantValidTargetIsTrackedButNotTriggered)
{
  auto target = make_valid_target();
  target.distance_m = 55.0F;

  const auto status = antidrone_turret::decide(target, DecisionConfig{}, ActuatorState::kReady);

  EXPECT_EQ(status.target_state, TargetState::kLocked);
  EXPECT_EQ(status.action, TurretAction::kTrack);
  EXPECT_EQ(status.trigger_state, TriggerState::kSkip);
}

// approach_trigger.csv: настав момент для пострілу.
TEST(TurretStatusTest, CloseValidTargetWithReadyActuatorRequestsShot)
{
  const auto status =
    antidrone_turret::decide(make_valid_target(), DecisionConfig{}, ActuatorState::kReady);

  EXPECT_EQ(status.target_state, TargetState::kLocked);
  EXPECT_EQ(status.action, TurretAction::kTrack);
  EXPECT_EQ(status.trigger_state, TriggerState::kRequested);
}

// reload_pressure.csv: другий FPV доходить, поки актуатор ще перезаряджається.
TEST(TurretStatusTest, CloseValidTargetWhileReloadingKeepsTrackingWithoutShot)
{
  auto target = make_valid_target();
  target.distance_m = 7.0F;

  const auto status =
    antidrone_turret::decide(target, DecisionConfig{}, ActuatorState::kReloading);

  EXPECT_EQ(status.target_state, TargetState::kLocked);
  EXPECT_EQ(status.action, TurretAction::kTrack);
  EXPECT_EQ(status.trigger_state, TriggerState::kReloading);
}

TEST(TurretStatusTest, StatusCarriesTargetMeasurements)
{
  const auto status =
    antidrone_turret::decide(make_valid_target(), DecisionConfig{}, ActuatorState::kReady);

  EXPECT_FLOAT_EQ(status.confidence, 0.90F);
  EXPECT_FLOAT_EQ(status.distance_m, 25.0F);
}

TEST(TurretStatusTest, CustomConfigOverridesDefaults)
{
  auto config = DecisionConfig{};
  config.confidence_threshold = 0.95F;
  config.max_distance_m = 10.0F;

  const auto status =
    antidrone_turret::decide(make_valid_target(), config, ActuatorState::kReady);

  EXPECT_EQ(status.target_state, TargetState::kLowConfidence);
  EXPECT_EQ(status.action, TurretAction::kIdle);
  EXPECT_EQ(status.trigger_state, TriggerState::kSkip);
}

}  // namespace
