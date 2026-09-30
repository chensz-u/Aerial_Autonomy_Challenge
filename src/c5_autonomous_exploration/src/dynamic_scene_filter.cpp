#include "c5_autonomous_exploration/dynamic_scene_filter.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace c5_autonomous_exploration {

DynamicFilterConfig::DynamicFilterConfig()
    : association_radius(1.5), dynamic_speed_threshold(0.5), track_timeout(1.5) {}

DynamicSceneFilter::DynamicSceneFilter(const DynamicFilterConfig& config) : config_(config) {}

void DynamicSceneFilter::update(const std::vector<Vec3>& points, double stamp) {
  tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(), [this, stamp](const SceneTrack& track) {
    return stamp - track.stamp > config_.track_timeout;
  }), tracks_.end());
  static_points_.clear();
  dynamic_points_.clear();
  std::vector<bool> consumed(tracks_.size(), false);
  for (const Vec3& point : points) {
    std::size_t nearest = tracks_.size();
    double best_distance = config_.association_radius;
    for (std::size_t index = 0; index < tracks_.size(); ++index) {
      if (!consumed[index]) {
        const double candidate_distance = distance(point, tracks_[index].position);
        if (candidate_distance < best_distance) {
          nearest = index;
          best_distance = candidate_distance;
        }
      }
    }
    if (nearest == tracks_.size() && !tracks_.empty()) {
      double closest_distance = std::numeric_limits<double>::max();
      for (std::size_t index = 0; index < tracks_.size(); ++index) {
        const double candidate_distance = distance(point, tracks_[index].position);
        if (candidate_distance < closest_distance) {
          nearest = index;
          closest_distance = candidate_distance;
        }
      }
    }
    if (nearest == tracks_.size()) {
      SceneTrack track;
      track.position = point;
      track.velocity = Vec3();
      track.stamp = stamp;
      track.observations = 1;
      track.dynamic = false;
      tracks_.push_back(track);
      static_points_.push_back(point);
      continue;
    }
    SceneTrack& track = tracks_[nearest];
    const double dt = std::max(1e-3, stamp - track.stamp);
    track.velocity = Vec3((point.x - track.position.x) / dt, (point.y - track.position.y) / dt,
                          (point.z - track.position.z) / dt);
    track.position = point;
    track.stamp = stamp;
    ++track.observations;
    track.dynamic = std::sqrt(track.velocity.x * track.velocity.x + track.velocity.y * track.velocity.y +
                              track.velocity.z * track.velocity.z) > config_.dynamic_speed_threshold;
    consumed[nearest] = true;
    if (track.dynamic) {
      dynamic_points_.push_back(point);
    } else {
      static_points_.push_back(point);
    }
  }
}

std::vector<Vec3> DynamicSceneFilter::staticPoints() const { return static_points_; }

std::vector<Vec3> DynamicSceneFilter::dynamicPoints() const { return dynamic_points_; }

std::vector<SceneTrack> DynamicSceneFilter::tracks() const { return tracks_; }

double DynamicSceneFilter::distance(const Vec3& first, const Vec3& second) {
  const double dx = first.x - second.x;
  const double dy = first.y - second.y;
  const double dz = first.z - second.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

}
