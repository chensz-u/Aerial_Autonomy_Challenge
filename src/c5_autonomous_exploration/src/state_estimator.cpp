#include "c5_autonomous_exploration/state_estimator.hpp"

#include <algorithm>
#include <cmath>

namespace c5_autonomous_exploration {

EstimatorConfig::EstimatorConfig()
    : process_noise(0.02), minimum_covariance(0.005), maximum_covariance(4.0),
      lio_innovation_limit(4.0), vio_innovation_limit(6.0) {}

MultiModalStateEstimator::MultiModalStateEstimator(const EstimatorConfig& config)
    : config_(config), initialized_(false) {
  reset(Vec3(), 0.0);
}

void MultiModalStateEstimator::reset(const Vec3& position, double stamp) {
  estimate_.position = position;
  estimate_.velocity = Vec3();
  estimate_.acceleration = Vec3();
  estimate_.stamp = stamp;
  estimate_.position_covariance = config_.minimum_covariance;
  estimate_.lio_confidence = 0.0;
  estimate_.vio_confidence = 0.0;
  estimate_.fused_confidence = 0.0;
  initialized_ = true;
}

void MultiModalStateEstimator::propagate(const Vec3& acceleration, const Vec3&, double stamp) {
  if (!initialized_) {
    reset(Vec3(), stamp);
    return;
  }
  const double dt = std::max(0.0, stamp - estimate_.stamp);
  estimate_.position.x += estimate_.velocity.x * dt + 0.5 * acceleration.x * dt * dt;
  estimate_.position.y += estimate_.velocity.y * dt + 0.5 * acceleration.y * dt * dt;
  estimate_.position.z += estimate_.velocity.z * dt + 0.5 * acceleration.z * dt * dt;
  estimate_.velocity.x += acceleration.x * dt;
  estimate_.velocity.y += acceleration.y * dt;
  estimate_.velocity.z += acceleration.z * dt;
  estimate_.acceleration = acceleration;
  estimate_.stamp = stamp;
  estimate_.position_covariance = clamp(estimate_.position_covariance + config_.process_noise * (1.0 + dt),
                                        config_.minimum_covariance, config_.maximum_covariance);
  estimate_.fused_confidence = clamp(1.0 / (1.0 + estimate_.position_covariance), 0.0, 1.0);
}

bool MultiModalStateEstimator::correctLio(const Vec3& position, double covariance) {
  const Vec3 innovation(position.x - estimate_.position.x, position.y - estimate_.position.y,
                        position.z - estimate_.position.z);
  if (norm(innovation) > config_.lio_innovation_limit || covariance <= 0.0) {
    estimate_.lio_confidence *= 0.5;
    return false;
  }
  const double gain = estimate_.position_covariance /
                      (estimate_.position_covariance + std::max(covariance, config_.minimum_covariance));
  estimate_.position.x += gain * innovation.x;
  estimate_.position.y += gain * innovation.y;
  estimate_.position.z += gain * innovation.z;
  estimate_.position_covariance = clamp((1.0 - gain) * estimate_.position_covariance,
                                        config_.minimum_covariance, config_.maximum_covariance);
  estimate_.lio_confidence = clamp(1.0 / (1.0 + covariance), 0.0, 1.0);
  estimate_.fused_confidence = clamp(0.65 * estimate_.lio_confidence +
                                     0.35 * estimate_.vio_confidence, 0.0, 1.0);
  return true;
}

bool MultiModalStateEstimator::correctVio(const Vec3& position, double scale, double covariance) {
  if (scale <= 0.0 || covariance <= 0.0) {
    estimate_.vio_confidence *= 0.5;
    return false;
  }
  const Vec3 metric(position.x * scale, position.y * scale, position.z * scale);
  const Vec3 innovation(metric.x - estimate_.position.x, metric.y - estimate_.position.y,
                        metric.z - estimate_.position.z);
  if (norm(innovation) > config_.vio_innovation_limit) {
    estimate_.vio_confidence *= 0.5;
    return false;
  }
  const double gain = estimate_.position_covariance /
                      (estimate_.position_covariance + std::max(covariance, config_.minimum_covariance));
  estimate_.position.x += gain * innovation.x;
  estimate_.position.y += gain * innovation.y;
  estimate_.position.z += gain * innovation.z;
  estimate_.position_covariance = clamp((1.0 - gain) * estimate_.position_covariance,
                                        config_.minimum_covariance, config_.maximum_covariance);
  estimate_.vio_confidence = clamp(1.0 / (1.0 + covariance), 0.0, 1.0);
  estimate_.fused_confidence = clamp(0.65 * estimate_.lio_confidence +
                                     0.35 * estimate_.vio_confidence, 0.0, 1.0);
  return true;
}

double MultiModalStateEstimator::alignScale(const std::vector<Vec3>& visual_positions,
                                            const std::vector<Vec3>& metric_positions) const {
  if (visual_positions.size() != metric_positions.size() || visual_positions.size() < 2) {
    return 1.0;
  }
  double numerator = 0.0;
  double denominator = 0.0;
  for (std::size_t index = 1; index < visual_positions.size(); ++index) {
    const Vec3 visual(visual_positions[index].x - visual_positions[index - 1].x,
                      visual_positions[index].y - visual_positions[index - 1].y,
                      visual_positions[index].z - visual_positions[index - 1].z);
    const Vec3 metric(metric_positions[index].x - metric_positions[index - 1].x,
                      metric_positions[index].y - metric_positions[index - 1].y,
                      metric_positions[index].z - metric_positions[index - 1].z);
    numerator += visual.x * metric.x + visual.y * metric.y + visual.z * metric.z;
    denominator += visual.x * visual.x + visual.y * visual.y + visual.z * visual.z;
  }
  return denominator > 1e-9 ? std::max(0.001, numerator / denominator) : 1.0;
}

StateEstimate MultiModalStateEstimator::estimate() const { return estimate_; }

double MultiModalStateEstimator::clamp(double value, double minimum, double maximum) {
  return std::max(minimum, std::min(maximum, value));
}

double MultiModalStateEstimator::norm(const Vec3& value) {
  return std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z);
}

}
