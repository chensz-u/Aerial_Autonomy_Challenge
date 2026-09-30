#pragma once

#include "c5_autonomous_exploration/autonomous_exploration.hpp"

#include <map>
#include <vector>

namespace c5_autonomous_exploration {

struct LocalMapConfig {
  double resolution;
  double window_radius;
  double hit_log_odds;
  double miss_log_odds;
  double occupied_threshold;
  double evidence_limit;
  double decay_seconds;
  double inflation_radius;
  LocalMapConfig();
};

struct MapDelta {
  std::vector<Vec3> occupied_added;
  std::vector<Vec3> occupied_removed;
};

class RollingOccupancyMap {
 public:
  explicit RollingOccupancyMap(const LocalMapConfig& config = LocalMapConfig());
  void setCenter(const Vec3& center);
  void integrateRay(const Vec3& origin, const Vec3& endpoint, double stamp);
  void decay(double stamp);
  bool occupied(const Vec3& point) const;
  bool traversable(const Vec3& point) const;
  bool segmentClear(const Vec3& start, const Vec3& end, double clearance) const;
  double distanceToObstacle(const Vec3& point) const;
  std::vector<Vec3> occupiedPoints() const;
  MapDelta takeDelta();

 private:
  struct Index {
    int x;
    int y;
    int z;
    bool operator<(const Index& other) const;
  };
  struct Cell {
    double log_odds;
    double stamp;
  };

  LocalMapConfig config_;
  Vec3 center_;
  std::map<Index, Cell> cells_;
  MapDelta delta_;

  Index indexOf(const Vec3& point) const;
  Vec3 centerOf(const Index& index) const;
  void update(const Index& index, double increment, double stamp);
  void prune();
  static double distance(const Vec3& first, const Vec3& second);
};

}
