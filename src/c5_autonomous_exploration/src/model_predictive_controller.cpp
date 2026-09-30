#include "c5_autonomous_exploration/model_predictive_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace c5_autonomous_exploration {

MpcConfig::MpcConfig()
    : horizon_steps(12), optimization_steps(8), timestep(0.12), maximum_acceleration(3.0),
      maximum_velocity(2.0), obstacle_clearance(0.8), position_weight(4.0), velocity_weight(0.8),
      effort_weight(0.15), obstacle_weight(2.5) {}

ModelPredictiveController::ModelPredictiveController(const MpcConfig& config) : config_(config) {}

MpcCommand ModelPredictiveController::solve(const MpcState& state, const Vec3& target,
                                            const std::vector<Vec3>& obstacles) const {
  Vec3 acceleration;
  const double horizon = config_.horizon_steps * config_.timestep;
  Vec3 predicted(state.position.x + state.velocity.x * horizon,
                 state.position.y + state.velocity.y * horizon,
                 state.position.z + state.velocity.z * horizon);
  const Vec3 error(target.x - predicted.x, target.y - predicted.y, target.z - predicted.z);
  const double gain = config_.position_weight /
                      std::max(1e-3, config_.effort_weight + config_.position_weight * horizon * horizon);
  acceleration = Vec3(error.x * gain - state.velocity.x * config_.velocity_weight,
                      error.y * gain - state.velocity.y * config_.velocity_weight,
                      error.z * gain - state.velocity.z * config_.velocity_weight);
  for (int iteration = 0; iteration < config_.optimization_steps; ++iteration) {
    Vec3 repulsion;
    const Vec3 current_prediction(state.position.x + state.velocity.x * horizon +
                                      0.5 * acceleration.x * horizon * horizon,
                                  state.position.y + state.velocity.y * horizon +
                                      0.5 * acceleration.y * horizon * horizon,
                                  state.position.z + state.velocity.z * horizon +
                                      0.5 * acceleration.z * horizon * horizon);
    for (const Vec3& obstacle : obstacles) {
      const Vec3 delta(current_prediction.x - obstacle.x, current_prediction.y - obstacle.y,
                       current_prediction.z - obstacle.z);
      const double separation = std::max(0.05, norm(delta));
      if (separation < config_.obstacle_clearance * 2.0) {
        const double magnitude = config_.obstacle_weight *
                                 (1.0 / separation - 1.0 / (config_.obstacle_clearance * 2.0)) /
                                 (separation * separation);
        repulsion.x += delta.x / separation * magnitude;
        repulsion.y += delta.y / separation * magnitude;
        repulsion.z += delta.z / separation * magnitude;
      }
    }
    acceleration.x += 0.08 * repulsion.x;
    acceleration.y += 0.08 * repulsion.y;
    acceleration.z += 0.08 * repulsion.z;
    acceleration = scaleToNorm(acceleration, config_.maximum_acceleration);
  }
  const Vec3 terminal_velocity(state.velocity.x + acceleration.x * horizon,
                               state.velocity.y + acceleration.y * horizon,
                               state.velocity.z + acceleration.z * horizon);
  if (norm(terminal_velocity) > config_.maximum_velocity) {
    const Vec3 limited_velocity = scaleToNorm(terminal_velocity, config_.maximum_velocity);
    acceleration = Vec3((limited_velocity.x - state.velocity.x) / horizon,
                        (limited_velocity.y - state.velocity.y) / horizon,
                        (limited_velocity.z - state.velocity.z) / horizon);
  }
  predicted = Vec3(state.position.x + state.velocity.x * horizon + 0.5 * acceleration.x * horizon * horizon,
                   state.position.y + state.velocity.y * horizon + 0.5 * acceleration.y * horizon * horizon,
                   state.position.z + state.velocity.z * horizon + 0.5 * acceleration.z * horizon * horizon);
  const Vec3 terminal_error(target.x - predicted.x, target.y - predicted.y, target.z - predicted.z);
  MpcCommand command;
  command.acceleration = acceleration;
  command.predicted_position = predicted;
  command.cost = config_.position_weight * norm(terminal_error) * norm(terminal_error) +
                 config_.effort_weight * norm(acceleration) * norm(acceleration);
  return command;
}

double ModelPredictiveController::norm(const Vec3& value) {
  return std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z);
}

Vec3 ModelPredictiveController::scaleToNorm(const Vec3& value, double limit) {
  const double current_norm = norm(value);
  if (current_norm <= limit || current_norm < std::numeric_limits<double>::epsilon()) {
    return value;
  }
  const double scale = limit / current_norm;
  return Vec3(value.x * scale, value.y * scale, value.z * scale);
}

}
