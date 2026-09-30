#include "c5_autonomous_exploration/mission_executor.hpp"

namespace c5_autonomous_exploration {

MissionConfig::MissionConfig()
    : return_battery_fraction(0.25), land_battery_fraction(0.12), link_continue_seconds(30.0),
      relocalize_failures(2), lock_failures(5), avoid_risk(0.65) {}

MissionExecutor::MissionExecutor(const MissionConfig& config)
    : config_(config), state_(MissionState::kTakeoff), link_lost_stamp_(-1.0) {}

void MissionExecutor::reset(double) {
  state_ = MissionState::kTakeoff;
  link_lost_stamp_ = -1.0;
}

MissionState MissionExecutor::update(const MissionInput& input, double stamp) {
  if (state_ == MissionState::kLocked) return state_;
  if (input.battery_fraction <= config_.land_battery_fraction) {
    state_ = MissionState::kLand;
    return state_;
  }
  if (input.task_interrupted || input.map_invalid || input.planner_failures >= config_.lock_failures) {
    state_ = input.planner_failures >= config_.lock_failures ? MissionState::kLocked : MissionState::kReturn;
    return state_;
  }
  if (input.battery_fraction <= config_.return_battery_fraction) {
    state_ = MissionState::kReturn;
    return state_;
  }
  if (!input.link_available) {
    if (link_lost_stamp_ < 0.0) link_lost_stamp_ = stamp;
    state_ = stamp - link_lost_stamp_ >= config_.link_continue_seconds ? MissionState::kReturn : MissionState::kLinkContinue;
    return state_;
  }
  link_lost_stamp_ = -1.0;
  if (!input.localization_healthy || input.planner_failures >= config_.relocalize_failures) {
    state_ = MissionState::kRelocalize;
    return state_;
  }
  if (!input.airborne) {
    state_ = MissionState::kTakeoff;
    return state_;
  }
  if (input.obstacle_risk >= config_.avoid_risk) {
    state_ = MissionState::kAvoid;
    return state_;
  }
  state_ = MissionState::kExplore;
  return state_;
}

MissionState MissionExecutor::state() const { return state_; }

}
