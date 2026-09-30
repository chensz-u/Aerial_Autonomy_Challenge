#pragma once

#include "c5_autonomous_exploration/autonomous_exploration.hpp"

#include <vector>

namespace c5_autonomous_exploration {

struct StateEstimate {
  Vec3 position;
  Vec3 velocity;
  Vec3 acceleration;
  double stamp;
  double position_covariance;
  double lio_confidence;
  double vio_confidence;
  double fused_confidence;
};

struct EstimatorConfig {
  double process_noise;
  double minimum_covariance;
  double maximum_covariance;
  double lio_innovation_limit;
  double vio_innovation_limit;
  EstimatorConfig();
};

class MultiModalStateEstimator {
 public:
  explicit MultiModalStateEstimator(const EstimatorConfig& config = EstimatorConfig());
  void reset(const Vec3& position, double stamp);
  void propagate(const Vec3& acceleration, const Vec3& angular_velocity, double stamp);
  bool correctLio(const Vec3& position, double covariance);
  bool correctVio(const Vec3& position, double scale, double covariance);
  double alignScale(const std::vector<Vec3>& visual_positions,
                    const std::vector<Vec3>& metric_positions) const;
  StateEstimate estimate() const;

 private:
  EstimatorConfig config_;
  StateEstimate estimate_;
  bool initialized_;

  static double clamp(double value, double minimum, double maximum);
  static double norm(const Vec3& value);
};

}
