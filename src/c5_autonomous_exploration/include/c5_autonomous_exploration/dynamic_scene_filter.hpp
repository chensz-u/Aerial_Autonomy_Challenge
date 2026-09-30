#pragma once

#include "c5_autonomous_exploration/autonomous_exploration.hpp"

#include <vector>

namespace c5_autonomous_exploration {

struct DynamicFilterConfig {
  double association_radius;
  double dynamic_speed_threshold;
  double track_timeout;
  DynamicFilterConfig();
};

struct SceneTrack {
  Vec3 position;
  Vec3 velocity;
  double stamp;
  int observations;
  bool dynamic;
};

class DynamicSceneFilter {
 public:
  explicit DynamicSceneFilter(const DynamicFilterConfig& config = DynamicFilterConfig());
  void update(const std::vector<Vec3>& points, double stamp);
  std::vector<Vec3> staticPoints() const;
  std::vector<Vec3> dynamicPoints() const;
  std::vector<SceneTrack> tracks() const;

 private:
  DynamicFilterConfig config_;
  std::vector<SceneTrack> tracks_;
  std::vector<Vec3> static_points_;
  std::vector<Vec3> dynamic_points_;

  static double distance(const Vec3& first, const Vec3& second);
};

}
