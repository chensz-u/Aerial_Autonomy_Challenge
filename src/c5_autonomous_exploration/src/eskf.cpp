#include "c5_autonomous_exploration/eskf.hpp"

#include <algorithm>
#include <cmath>

namespace c5_autonomous_exploration {

namespace {

double& covariance(std::array<double, 324>& matrix, int row, int column) {
  return matrix[static_cast<std::size_t>(row * 18 + column)];
}

double covarianceValue(const std::array<double, 324>& matrix, int row, int column) {
  return matrix[static_cast<std::size_t>(row * 18 + column)];
}

bool inverse3(const double input[3][3], double output[3][3]) {
  const double determinant = input[0][0] * (input[1][1] * input[2][2] - input[1][2] * input[2][1]) -
                             input[0][1] * (input[1][0] * input[2][2] - input[1][2] * input[2][0]) +
                             input[0][2] * (input[1][0] * input[2][1] - input[1][1] * input[2][0]);
  if (std::abs(determinant) < 1e-12) return false;
  const double reciprocal = 1.0 / determinant;
  output[0][0] = reciprocal * (input[1][1] * input[2][2] - input[1][2] * input[2][1]);
  output[0][1] = reciprocal * (input[0][2] * input[2][1] - input[0][1] * input[2][2]);
  output[0][2] = reciprocal * (input[0][1] * input[1][2] - input[0][2] * input[1][1]);
  output[1][0] = reciprocal * (input[1][2] * input[2][0] - input[1][0] * input[2][2]);
  output[1][1] = reciprocal * (input[0][0] * input[2][2] - input[0][2] * input[2][0]);
  output[1][2] = reciprocal * (input[0][2] * input[1][0] - input[0][0] * input[1][2]);
  output[2][0] = reciprocal * (input[1][0] * input[2][1] - input[1][1] * input[2][0]);
  output[2][1] = reciprocal * (input[0][1] * input[2][0] - input[0][0] * input[2][1]);
  output[2][2] = reciprocal * (input[0][0] * input[1][1] - input[0][1] * input[1][0]);
  return true;
}

}

Quaternion::Quaternion() : w(1.0), x(0.0), y(0.0), z(0.0) {}
Quaternion::Quaternion(double w_value, double x_value, double y_value, double z_value)
    : w(w_value), x(x_value), y(y_value), z(z_value) {}

EskfConfig::EskfConfig()
    : gyro_noise(0.004), acceleration_noise(0.08), gyro_bias_noise(0.0002),
      acceleration_bias_noise(0.002), gravity_noise(0.0001), nis_limit(11.345), covariance_floor(1e-8) {}

ErrorStateKalmanFilter::ErrorStateKalmanFilter(const EskfConfig& config)
    : config_(config), last_nis_(0.0), consecutive_position_rejections_(0),
      last_rejected_position_(0.0, 0.0, 0.0), consecutive_velocity_rejections_(0),
      last_rejected_velocity_(0.0, 0.0, 0.0), initialized_(false) {
  NavigationState initial;
  initial.gravity = Vec3(0.0, 0.0, -9.81);
  reset(initial, 0.0);
}

void ErrorStateKalmanFilter::reset(const NavigationState& state, double stamp) {
  state_ = state;
  state_.orientation = normalized(state.orientation);
  state_.stamp = stamp;
  state_.covariance.fill(0.0);
  for (int index = 0; index < 18; ++index) covariance(state_.covariance, index, index) = 0.05;
  last_nis_ = 0.0;
  consecutive_position_rejections_ = 0;
  consecutive_velocity_rejections_ = 0;
  initialized_ = true;
}

bool ErrorStateKalmanFilter::propagate(const ImuSample& sample) {
  if (!initialized_ || sample.stamp <= state_.stamp) return false;
  const double dt = sample.stamp - state_.stamp;
  const Vec3 omega(sample.angular_velocity.x - state_.gyro_bias.x,
                   sample.angular_velocity.y - state_.gyro_bias.y,
                   sample.angular_velocity.z - state_.gyro_bias.z);
  const Vec3 body_acceleration(sample.acceleration.x - state_.acceleration_bias.x,
                               sample.acceleration.y - state_.acceleration_bias.y,
                               sample.acceleration.z - state_.acceleration_bias.z);
  const Vec3 world_acceleration_before = rotate(state_.orientation, body_acceleration);
  const Vec3 acceleration_before(world_acceleration_before.x + state_.gravity.x,
                                 world_acceleration_before.y + state_.gravity.y,
                                 world_acceleration_before.z + state_.gravity.z);
  state_.position.x += state_.velocity.x * dt + 0.5 * acceleration_before.x * dt * dt;
  state_.position.y += state_.velocity.y * dt + 0.5 * acceleration_before.y * dt * dt;
  state_.position.z += state_.velocity.z * dt + 0.5 * acceleration_before.z * dt * dt;
  state_.velocity.x += acceleration_before.x * dt;
  state_.velocity.y += acceleration_before.y * dt;
  state_.velocity.z += acceleration_before.z * dt;
  state_.orientation = normalized(multiply(state_.orientation, deltaQuaternion(Vec3(omega.x * dt, omega.y * dt, omega.z * dt))));
  std::array<double, 324> transition;
  transition.fill(0.0);
  for (int index = 0; index < 18; ++index) covariance(transition, index, index) = 1.0;
  for (int axis = 0; axis < 3; ++axis) {
    covariance(transition, axis, axis + 3) = dt;
    covariance(transition, axis + 3, axis + 12) = -dt;
    covariance(transition, axis + 3, axis + 15) = dt;
    covariance(transition, axis + 6, axis + 9) = -dt;
  }
  std::array<double, 324> propagated;
  propagated.fill(0.0);
  for (int row = 0; row < 18; ++row) {
    for (int column = 0; column < 18; ++column) {
      double value = 0.0;
      for (int left = 0; left < 18; ++left) {
        for (int right = 0; right < 18; ++right) {
          value += covarianceValue(transition, row, left) * covarianceValue(state_.covariance, left, right) *
                   covarianceValue(transition, column, right);
        }
      }
      covariance(propagated, row, column) = value;
    }
  }
  for (int axis = 0; axis < 3; ++axis) {
    covariance(propagated, axis + 3, axis + 3) += config_.acceleration_noise * dt;
    covariance(propagated, axis + 6, axis + 6) += config_.gyro_noise * dt;
    covariance(propagated, axis + 9, axis + 9) += config_.gyro_bias_noise * dt;
    covariance(propagated, axis + 12, axis + 12) += config_.acceleration_bias_noise * dt;
    covariance(propagated, axis + 15, axis + 15) += config_.gravity_noise * dt;
  }
  state_.covariance = propagated;
  state_.stamp = sample.stamp;
  return true;
}

bool ErrorStateKalmanFilter::correctPosition(const Vec3& position, double variance) {
  if (variance <= 0.0) return false;
  correctVector(Vec3(position.x - state_.position.x, position.y - state_.position.y,
                     position.z - state_.position.z), 0, variance);
  if (last_nis_ <= config_.nis_limit) {
    consecutive_position_rejections_ = 0;
    return true;
  }

  const double dx = position.x - last_rejected_position_.x;
  const double dy = position.y - last_rejected_position_.y;
  const double dz = position.z - last_rejected_position_.z;
  if (consecutive_position_rejections_ > 0 && dx * dx + dy * dy + dz * dz <= 0.25 * 0.25) {
    ++consecutive_position_rejections_;
  } else {
    consecutive_position_rejections_ = 1;
  }
  last_rejected_position_ = position;
  if (consecutive_position_rejections_ < 3) return false;

  NavigationState recovered = state_;
  recovered.position = position;
  reset(recovered, state_.stamp);
  return true;
}

bool ErrorStateKalmanFilter::correctVelocity(const Vec3& velocity, double variance) {
  if (variance <= 0.0) return false;
  correctVector(Vec3(velocity.x - state_.velocity.x, velocity.y - state_.velocity.y,
                     velocity.z - state_.velocity.z), 3, variance);
  if (last_nis_ <= config_.nis_limit) {
    consecutive_velocity_rejections_ = 0;
    return true;
  }

  const double dx = velocity.x - last_rejected_velocity_.x;
  const double dy = velocity.y - last_rejected_velocity_.y;
  const double dz = velocity.z - last_rejected_velocity_.z;
  if (consecutive_velocity_rejections_ > 0 && dx * dx + dy * dy + dz * dz <= 0.25 * 0.25) {
    ++consecutive_velocity_rejections_;
  } else {
    consecutive_velocity_rejections_ = 1;
  }
  last_rejected_velocity_ = velocity;
  if (consecutive_velocity_rejections_ < 3) return false;

  NavigationState recovered = state_;
  recovered.velocity = velocity;
  reset(recovered, state_.stamp);
  return true;
}

void ErrorStateKalmanFilter::correctVector(const Vec3& residual, int covariance_offset, double variance) {
  double innovation[3][3];
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      innovation[row][column] = covarianceValue(state_.covariance, covariance_offset + row, covariance_offset + column);
      if (row == column) innovation[row][column] += variance;
    }
  }
  double inverse[3][3];
  if (!inverse3(innovation, inverse)) {
    last_nis_ = config_.nis_limit + 1.0;
    return;
  }
  const double values[3] = {residual.x, residual.y, residual.z};
  last_nis_ = 0.0;
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) last_nis_ += values[row] * inverse[row][column] * values[column];
  }
  if (last_nis_ > config_.nis_limit) return;
  std::array<double, 54> gain;
  gain.fill(0.0);
  for (int row = 0; row < 18; ++row) {
    for (int column = 0; column < 3; ++column) {
      for (int intermediate = 0; intermediate < 3; ++intermediate) {
        gain[static_cast<std::size_t>(row * 3 + column)] +=
            covarianceValue(state_.covariance, row, covariance_offset + intermediate) * inverse[intermediate][column];
      }
    }
  }
  std::array<double, 18> correction;
  correction.fill(0.0);
  for (int row = 0; row < 18; ++row) {
    for (int column = 0; column < 3; ++column) correction[static_cast<std::size_t>(row)] += gain[static_cast<std::size_t>(row * 3 + column)] * values[column];
  }
  applyError(correction);
  std::array<double, 324> updated = state_.covariance;
  for (int row = 0; row < 18; ++row) {
    for (int column = 0; column < 18; ++column) {
      double delta = 0.0;
      for (int intermediate = 0; intermediate < 3; ++intermediate) {
        delta += gain[static_cast<std::size_t>(row * 3 + intermediate)] *
                 covarianceValue(state_.covariance, covariance_offset + intermediate, column);
      }
      covariance(updated, row, column) = covarianceValue(state_.covariance, row, column) - delta;
    }
  }
  for (int index = 0; index < 18; ++index) {
    covariance(updated, index, index) = std::max(config_.covariance_floor, covarianceValue(updated, index, index));
  }
  state_.covariance = updated;
}

void ErrorStateKalmanFilter::applyError(const std::array<double, 18>& error) {
  state_.position.x += error[0]; state_.position.y += error[1]; state_.position.z += error[2];
  state_.velocity.x += error[3]; state_.velocity.y += error[4]; state_.velocity.z += error[5];
  state_.orientation = normalized(multiply(state_.orientation, deltaQuaternion(Vec3(error[6], error[7], error[8]))));
  state_.gyro_bias.x += error[9]; state_.gyro_bias.y += error[10]; state_.gyro_bias.z += error[11];
  state_.acceleration_bias.x += error[12]; state_.acceleration_bias.y += error[13]; state_.acceleration_bias.z += error[14];
  state_.gravity.x += error[15]; state_.gravity.y += error[16]; state_.gravity.z += error[17];
}

NavigationState ErrorStateKalmanFilter::state() const { return state_; }
double ErrorStateKalmanFilter::lastNis() const { return last_nis_; }
double ErrorStateKalmanFilter::covarianceTrace() const { double trace = 0.0; for (int i = 0; i < 18; ++i) trace += covarianceValue(state_.covariance, i, i); return trace; }
Quaternion ErrorStateKalmanFilter::multiply(const Quaternion& a, const Quaternion& b) { return Quaternion(a.w*b.w-a.x*b.x-a.y*b.y-a.z*b.z, a.w*b.x+a.x*b.w+a.y*b.z-a.z*b.y, a.w*b.y-a.x*b.z+a.y*b.w+a.z*b.x, a.w*b.z+a.x*b.y-a.y*b.x+a.z*b.w); }
Quaternion ErrorStateKalmanFilter::normalized(const Quaternion& q) { const double n = std::sqrt(q.w*q.w+q.x*q.x+q.y*q.y+q.z*q.z); return n > 1e-12 ? Quaternion(q.w/n,q.x/n,q.y/n,q.z/n) : Quaternion(); }
Quaternion ErrorStateKalmanFilter::deltaQuaternion(const Vec3& r) { const double angle=std::sqrt(r.x*r.x+r.y*r.y+r.z*r.z); if (angle<1e-12) return Quaternion(); const double s=std::sin(angle*0.5)/angle; return Quaternion(std::cos(angle*0.5),r.x*s,r.y*s,r.z*s); }
Vec3 ErrorStateKalmanFilter::rotate(const Quaternion& q, const Vec3& v) { const Quaternion p(0.0,v.x,v.y,v.z); const Quaternion c(q.w,-q.x,-q.y,-q.z); const Quaternion result=multiply(multiply(q,p),c); return Vec3(result.x,result.y,result.z); }
Vec3 ErrorStateKalmanFilter::cross(const Vec3& a, const Vec3& b) { return Vec3(a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x); }
double ErrorStateKalmanFilter::clamp(double value, double minimum, double maximum) { return std::max(minimum,std::min(maximum,value)); }

}
