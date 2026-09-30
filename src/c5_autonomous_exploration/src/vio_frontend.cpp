#include "c5_autonomous_exploration/vio_frontend.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace c5_autonomous_exploration {

VioConfig::VioConfig()
    : keyframe_parallax(0.04), minimum_features(20), maximum_reprojection_rms(0.03), track_timeout(0.5) {}

VioFrontend::VioFrontend(const VioConfig& config) : config_(config) {
  health_.initialized = false;
  health_.failed = true;
  health_.keyframes = 0;
  health_.active_tracks = 0;
  health_.scale = 1.0;
  health_.reprojection_rms = 1.0;
  health_.confidence = 0.0;
}

bool VioFrontend::ingest(const FeatureFrame& frame) {
  for (const FeatureObservation& feature : frame.features) {
    if (feature.quality <= 0.0) continue;
    Track track;
    track.u = feature.u;
    track.v = feature.v;
    track.stamp = frame.stamp;
    track.observations = 1;
    const std::map<int, Track>::iterator existing = tracks_.find(feature.id);
    if (existing != tracks_.end()) track.observations = existing->second.observations + 1;
    tracks_[feature.id] = track;
  }
  for (std::map<int, Track>::iterator it = tracks_.begin(); it != tracks_.end();) {
    if (frame.stamp - it->second.stamp > config_.track_timeout) it = tracks_.erase(it);
    else ++it;
  }
  const bool create_keyframe = keyframes_.empty() || parallaxFromLastKeyframe() >= config_.keyframe_parallax;
  if (create_keyframe && !tracks_.empty()) {
    Keyframe keyframe;
    keyframe.stamp = frame.stamp;
    keyframe.features = tracks_;
    keyframes_.push_back(keyframe);
  }
  health_.keyframes = static_cast<int>(keyframes_.size());
  health_.active_tracks = static_cast<int>(tracks_.size());
  health_.failed = health_.active_tracks < config_.minimum_features ||
                   health_.reprojection_rms > config_.maximum_reprojection_rms;
  health_.confidence = health_.failed ? 0.0 :
      std::min(1.0, static_cast<double>(health_.active_tracks) / (2.0 * config_.minimum_features));
  return !health_.failed;
}

void VioFrontend::integrateImu(const ImuSample& sample) {
  imu_samples_.push_back(sample);
  while (imu_samples_.size() > 400) imu_samples_.erase(imu_samples_.begin());
}

bool VioFrontend::initializeScale(const std::vector<Vec3>& visual_positions,
                                  const std::vector<Vec3>& metric_positions) {
  if (visual_positions.size() != metric_positions.size() || visual_positions.size() < 2) return false;
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
  if (denominator < 1e-9) return false;
  health_.scale = numerator / denominator;
  health_.initialized = health_.scale > 1e-4 && std::isfinite(health_.scale);
  health_.failed = !health_.initialized;
  return health_.initialized;
}

double VioFrontend::evaluateReprojection(const std::vector<Vec3>& observed,
                                         const std::vector<Vec3>& projected) {
  if (observed.empty() || observed.size() != projected.size()) {
    health_.reprojection_rms = std::numeric_limits<double>::infinity();
    health_.failed = true;
    return health_.reprojection_rms;
  }
  double sum = 0.0;
  for (std::size_t index = 0; index < observed.size(); ++index) {
    const double dx = observed[index].x - projected[index].x;
    const double dy = observed[index].y - projected[index].y;
    sum += dx * dx + dy * dy;
  }
  health_.reprojection_rms = std::sqrt(sum / static_cast<double>(observed.size()));
  health_.failed = health_.reprojection_rms > config_.maximum_reprojection_rms;
  return health_.reprojection_rms;
}

VioHealth VioFrontend::health() const { return health_; }

double VioFrontend::parallaxFromLastKeyframe() const {
  if (keyframes_.empty() || tracks_.empty()) return 0.0;
  const Keyframe& keyframe = keyframes_.back();
  double sum = 0.0;
  int count = 0;
  for (std::map<int, Track>::const_iterator it = tracks_.begin(); it != tracks_.end(); ++it) {
    const std::map<int, Track>::const_iterator previous = keyframe.features.find(it->first);
    if (previous == keyframe.features.end()) continue;
    const double du = it->second.u - previous->second.u;
    const double dv = it->second.v - previous->second.v;
    sum += std::sqrt(du * du + dv * dv);
    ++count;
  }
  return count == 0 ? 0.0 : sum / static_cast<double>(count);
}

}
