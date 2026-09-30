#pragma once

#include "c5_autonomous_exploration/autonomous_exploration.hpp"

#include <vector>

namespace c5_autonomous_exploration {

struct MpcConfig {
  int horizon_steps;
  int optimization_steps;
  double timestep;
  double maximum_acceleration;
  double maximum_velocity;
  double obstacle_clearance;
  double position_weight;
  double velocity_weight;
  double effort_weight;
  double obstacle_weight;
  MpcConfig();
};

struct MpcState {
  Vec3 position;
  Vec3 velocity;
};

struct MpcCommand {
  Vec3 acceleration;
  Vec3 predicted_position;
  double cost;
};

class ModelPredictiveController {
 public:
  explicit ModelPredictiveController(const MpcConfig& config = MpcConfig());
  MpcCommand solve(const MpcState& state, const Vec3& target,
                   const std::vector<Vec3>& obstacles) const;

 private:
  MpcConfig config_;

  static double norm(const Vec3& value);
  static Vec3 scaleToNorm(const Vec3& value, double limit);
};

}
