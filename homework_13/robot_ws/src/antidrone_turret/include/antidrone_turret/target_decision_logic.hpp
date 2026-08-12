#pragma once

#include <cstdint>

namespace target_decision_logic {

struct TargetInput {
  bool visible{false};
  float confidence{0.0F};
  float distance_m{0.0F};
};

struct DecisionConfig {
  float confidence_threshold{0.8F};
  float max_distance_m{30.0F};
};

enum class Decision {
  kIdle,
  kTrack,
  kRequestShot,
};

enum class ActuatorState {
  kReady,
  kReloading,
};

struct TurretDecision {
  uint8_t target_state, action, trigger_state;
  float confidence, distance_m;
};

inline Decision decide(const TargetInput& target, const DecisionConfig& config)
{
  if (!target.visible) {
    return Decision::kIdle;
  }

  if (target.confidence < config.confidence_threshold) {
    return Decision::kIdle;
  }

  if (target.distance_m <= config.max_distance_m) {
    return Decision::kRequestShot;
  }

  return Decision::kTrack;
}

inline const char* decision_name(const Decision decision)
{
  switch (decision) {
    case Decision::kIdle:
      return "IDLE";
    case Decision::kTrack:
      return "TRACK";
    case Decision::kRequestShot:
      return "REQUEST_SHOT";
  }

  return "UNKNOWN";
}

}  // namespace target_decision_logic