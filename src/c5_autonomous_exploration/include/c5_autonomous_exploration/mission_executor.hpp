#pragma once

#include <cstdint>

namespace c5_autonomous_exploration {

enum class MissionState : std::uint8_t {
  kTakeoff = 0,
  kInitialize = 1,
  kExplore = 2,
  kAvoid = 3,
  kRelocalize = 4,
  kLinkContinue = 5,
  kReturn = 6,
  kLand = 7,
  kLocked = 8
};

struct MissionInput {
  bool airborne;
  bool link_available;
  bool localization_healthy;
  double battery_fraction;
  int planner_failures;
  bool map_invalid;
  bool task_interrupted;
  double obstacle_risk;
};

struct MissionConfig {
  double return_battery_fraction;
  double land_battery_fraction;
  double link_continue_seconds;
  int relocalize_failures;
  int lock_failures;
  double avoid_risk;
  MissionConfig();
};

class MissionExecutor {
 public:
  explicit MissionExecutor(const MissionConfig& config = MissionConfig());
  void reset(double stamp);
  MissionState update(const MissionInput& input, double stamp);
  MissionState state() const;

 private:
  MissionConfig config_;
  MissionState state_;
  double link_lost_stamp_;
};

}
