#pragma once

#include <cstdint>

#include "antidrone_turret/actuator_model.hpp"
#include "antidrone_turret/turret_status.hpp"

namespace antidrone_turret {

// Спрощена модель кадру з ДЗ: 640x480, центр (320, 240), x росте праворуч,
// y росте вниз.
inline constexpr float kFrameCenterX = 320.0F;
inline constexpr float kFrameCenterY = 240.0F;

// Значення збігаються з константами msg/ServoCommand.msg і msg/GimbalCommand.msg.
enum class ServoDirection : std::int8_t {
  kLeft = -1,
  kCenter = 0,
  kRight = 1,
};

enum class GimbalDirection : std::int8_t {
  kDown = -1,
  kCenter = 0,
  kUp = 1,
};

[[nodiscard]] inline const char* to_string(const ServoDirection direction)
{
  switch (direction) {
    case ServoDirection::kLeft:
      return "LEFT";
    case ServoDirection::kCenter:
      return "CENTER";
    case ServoDirection::kRight:
      return "RIGHT";
  }

  return "UNKNOWN";
}

[[nodiscard]] inline const char* to_string(const GimbalDirection direction)
{
  switch (direction) {
    case GimbalDirection::kDown:
      return "DOWN";
    case GimbalDirection::kCenter:
      return "CENTER";
    case GimbalDirection::kUp:
      return "UP";
  }

  return "UNKNOWN";
}

struct TargetInput {
  bool visible{false};
  float x{0.0F};
  float y{0.0F};
  float distance_m{0.0F};
  float confidence{0.0F};
};

struct DecisionConfig {
  float confidence_threshold{0.8F};
  float max_distance_m{30.0F};
};

struct ServoCommand {
  ServoDirection direction{ServoDirection::kCenter};
  float target_x{0.0F};
  float error_x{0.0F};
};

struct GimbalCommand {
  GimbalDirection direction{GimbalDirection::kCenter};
  float target_y{0.0F};
  float error_y{0.0F};
};

// Оцінка цілі: visible і confidence_threshold -> стан цілі.
[[nodiscard]] inline TargetState evaluate_target(
  const TargetInput& target,
  const DecisionConfig& config)
{
  if (!target.visible) {
    return TargetState::kNone;
  }

  if (target.confidence < config.confidence_threshold) {
    return TargetState::kLowConfidence;
  }

  return TargetState::kLocked;
}

// Наведення публікується тільки для надійно захопленої цілі.
[[nodiscard]] inline TurretAction decide_action(const TargetState target_state)
{
  return target_state == TargetState::kLocked ? TurretAction::kTrack
                                              : TurretAction::kIdle;
}

// Команда yaw-серво: Target.x -> напрямок, target_x, error_x.
[[nodiscard]] inline ServoCommand make_servo_command(const float target_x)
{
  const auto error_x = target_x - kFrameCenterX;

  auto direction = ServoDirection::kCenter;
  if (error_x > 0.0F) {
    direction = ServoDirection::kRight;
  } else if (error_x < 0.0F) {
    direction = ServoDirection::kLeft;
  }

  return ServoCommand{direction, target_x, error_x};
}

// Команда гімбала: Target.y -> напрямок, target_y, error_y.
// error_y рахується як 240 - y, бо y росте вниз: ціль вище центру дає UP.
[[nodiscard]] inline GimbalCommand make_gimbal_command(const float target_y)
{
  const auto error_y = kFrameCenterY - target_y;

  auto direction = GimbalDirection::kCenter;
  if (error_y > 0.0F) {
    direction = GimbalDirection::kUp;
  } else if (error_y < 0.0F) {
    direction = GimbalDirection::kDown;
  }

  return GimbalCommand{direction, target_y, error_y};
}

// Рішення щодо пострілу: дистанція, max_distance_m і останній стан актуатора.
// Постріл можливий тільки для захопленої цілі у робочій дистанції, коли
// актуатор READY. Близька ціль під час RELOADING позначається окремо, щоб
// пропущений епізод було видно у /turret/status.
[[nodiscard]] inline TriggerState decide_trigger(
  const TargetState target_state,
  const float distance_m,
  const DecisionConfig& config,
  const ActuatorState actuator_state)
{
  if (target_state != TargetState::kLocked) {
    return TriggerState::kSkip;
  }

  if (distance_m > config.max_distance_m) {
    return TriggerState::kSkip;
  }

  return actuator_state == ActuatorState::kReady ? TriggerState::kRequested
                                                 : TriggerState::kReloading;
}

// Складання TurretStatus для перевірки через /turret/status.
// actuator_state - останній стан з /actuator/status. Поки жодного статусу не
// надійшло, нода передає kReloading, щоб не запитати постріл наосліп.
[[nodiscard]] inline TurretStatus decide(
  const TargetInput& target,
  const DecisionConfig& config,
  const ActuatorState actuator_state)
{
  const auto target_state = evaluate_target(target, config);

  return TurretStatus{
    target_state,
    decide_action(target_state),
    decide_trigger(target_state, target.distance_m, config, actuator_state),
    target.confidence,
    target.distance_m,
  };
}

}  // namespace antidrone_turret
