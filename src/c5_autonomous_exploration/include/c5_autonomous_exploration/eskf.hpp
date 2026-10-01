#pragma once

#include "c5_autonomous_exploration/autonomous_exploration.hpp"

#include <array>

namespace c5_autonomous_exploration {

struct Quaternion {
  double w;
  double x;
  double y;
  double z;
  Quaternion();
  Quaternion(double w_value, double x_value, double y_value, double z_value);
};

struct ImuSample {
  Vec3 acceleration;
  Vec3 angular_velocity;
  double stamp;
};

struct NavigationState {
  Vec3 position;
  Vec3 velocity;
  Quaternion orientation;
  Vec3 gyro_bias;
  Vec3 acceleration_bias;
  Vec3 gravity;
  std::array<double, 324> covariance;
  double stamp;
};

struct EskfConfig {
  double gyro_noise;
  double acceleration_noise;
  double gyro_bias_noise;
  double acceleration_bias_noise;
  double gravity_noise;
  double nis_limit;
  double covariance_floor;
  EskfConfig();
};

class ErrorStateKalmanFilter {
 public:
  explicit ErrorStateKalmanFilter(const EskfConfig& config = EskfConfig());
  void reset(const NavigationState& state, double stamp);
  bool propagate(const ImuSample& sample);
  bool correctPosition(const Vec3& position, double variance);
  bool correctVelocity(const Vec3& velocity, double variance);
  NavigationState state() const;
  double lastNis() const;
  double covarianceTrace() const;

 private:
  EskfConfig config_;
  NavigationState state_;
  double last_nis_;
  int consecutive_position_rejections_;
  Vec3 last_rejected_position_;
  int consecutive_velocity_rejections_;
  Vec3 last_rejected_velocity_;
  bool initialized_;

  void correctVector(const Vec3& residual, int covariance_offset, double variance);
  void applyError(const std::array<double, 18>& error);
  static Quaternion multiply(const Quaternion& first, const Quaternion& second);
  static Quaternion normalized(const Quaternion& value);
  static Quaternion deltaQuaternion(const Vec3& rotation);
  static Vec3 rotate(const Quaternion& orientation, const Vec3& vector);
  static Vec3 cross(const Vec3& first, const Vec3& second);
  static double clamp(double value, double minimum, double maximum);
};

}
