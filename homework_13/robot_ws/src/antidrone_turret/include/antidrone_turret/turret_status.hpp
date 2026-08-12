#pragma once

#include <cstdint>

namespace antidrone_turret {

// Значення збігаються з константами msg/TurretStatus.msg, тому нода може
// віддати їх у повідомлення через static_cast без таблиці відповідності.
enum class TargetState : std::uint8_t {
  kNone = 0,
  kLowConfidence = 1,
  kLocked = 2,
};

enum class TurretAction : std::uint8_t {
  kIdle = 0,
  kTrack = 1,
};

enum class TriggerState : std::uint8_t {
  kSkip = 0,
  kRequested = 1,
  kReloading = 2,
};

struct TurretStatus {
  TargetState target_state{TargetState::kNone};
  TurretAction action{TurretAction::kIdle};
  TriggerState trigger_state{TriggerState::kSkip};
  float confidence{0.0F};
  float distance_m{0.0F};
};

[[nodiscard]] inline const char* to_string(const TargetState state)
{
  switch (state) {
    case TargetState::kNone:
      return "TARGET_NONE";
    case TargetState::kLowConfidence:
      return "TARGET_LOW_CONFIDENCE";
    case TargetState::kLocked:
      return "TARGET_LOCKED";
  }

  return "UNKNOWN";
}

[[nodiscard]] inline const char* to_string(const TurretAction action)
{
  switch (action) {
    case TurretAction::kIdle:
      return "ACTION_IDLE";
    case TurretAction::kTrack:
      return "ACTION_TRACK";
  }

  return "UNKNOWN";
}

[[nodiscard]] inline const char* to_string(const TriggerState state)
{
  switch (state) {
    case TriggerState::kSkip:
      return "TRIGGER_SKIP";
    case TriggerState::kRequested:
      return "TRIGGER_REQUESTED";
    case TriggerState::kReloading:
      return "TRIGGER_RELOADING";
  }

  return "UNKNOWN";
}

}  // namespace antidrone_turret
