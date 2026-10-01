#include "c5_autonomous_exploration/local_map.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace c5_autonomous_exploration {

bool pointHasClearance(const std::vector<Vec3>& occupied_points, const Vec3& candidate, double clearance) {
  if (!std::isfinite(clearance) || clearance < 0.0) return false;
  const double clearance_squared = clearance * clearance;
  for (const Vec3& point : occupied_points) {
    const double dx = candidate.x - point.x;
    const double dy = candidate.y - point.y;
    const double dz = candidate.z - point.z;
    if (dx * dx + dy * dy + dz * dz < clearance_squared) return false;
  }
  return true;
}

LocalMapConfig::LocalMapConfig()
    : resolution(0.25), window_radius(18.0), hit_log_odds(0.9), miss_log_odds(-0.6),
      occupied_threshold(0.0), evidence_limit(4.0), decay_seconds(12.0), inflation_radius(0.6) {}

RollingOccupancyMap::RollingOccupancyMap(const LocalMapConfig& config) : config_(config), center_() {}

void RollingOccupancyMap::setCenter(const Vec3& center) { center_ = center; prune(); }

void RollingOccupancyMap::integrateRay(const Vec3& origin, const Vec3& endpoint, double stamp) {
  const double length = distance(origin, endpoint);
  const int steps = std::max(1, static_cast<int>(std::ceil(length / (config_.resolution * 0.5))));
  for (int index = 0; index < steps; ++index) {
    const double fraction = static_cast<double>(index) / static_cast<double>(steps);
    update(indexOf(Vec3(origin.x + fraction * (endpoint.x - origin.x), origin.y + fraction * (endpoint.y - origin.y),
                        origin.z + fraction * (endpoint.z - origin.z))), config_.miss_log_odds, stamp);
  }
  update(indexOf(endpoint), config_.hit_log_odds, stamp);
  prune();
}

void RollingOccupancyMap::decay(double stamp) {
  for (std::map<Index, Cell>::iterator it = cells_.begin(); it != cells_.end();) {
    if (stamp - it->second.stamp > config_.decay_seconds) {
      if (it->second.log_odds > config_.occupied_threshold) delta_.occupied_removed.push_back(centerOf(it->first));
      it = cells_.erase(it);
    } else ++it;
  }
}

bool RollingOccupancyMap::occupied(const Vec3& point) const {
  const std::map<Index, Cell>::const_iterator found = cells_.find(indexOf(point));
  return found != cells_.end() && found->second.log_odds > config_.occupied_threshold;
}

bool RollingOccupancyMap::traversable(const Vec3& point) const {
  return distanceToObstacle(point) >= config_.inflation_radius;
}

bool RollingOccupancyMap::segmentClear(const Vec3& start, const Vec3& end, double clearance) const {
  const double length = distance(start, end);
  const int steps = std::max(1, static_cast<int>(std::ceil(length / (config_.resolution * 0.5))));
  for (int index = 0; index <= steps; ++index) {
    const double fraction = static_cast<double>(index) / static_cast<double>(steps);
    const Vec3 point(start.x + fraction * (end.x - start.x), start.y + fraction * (end.y - start.y),
                     start.z + fraction * (end.z - start.z));
    if (distanceToObstacle(point) < clearance) return false;
  }
  return true;
}

double RollingOccupancyMap::distanceToObstacle(const Vec3& point) const {
  double nearest = std::numeric_limits<double>::infinity();
  for (std::map<Index, Cell>::const_iterator it = cells_.begin(); it != cells_.end(); ++it) {
    if (it->second.log_odds > config_.occupied_threshold) nearest = std::min(nearest, distance(point, centerOf(it->first)));
  }
  return nearest;
}

bool RollingOccupancyMap::nearestClearPoint(const std::vector<Vec3>& candidates, const Vec3& target,
                                            double clearance, Vec3* selected) const {
  if (selected == NULL || clearance < 0.0) return false;
  bool found = false;
  double best_distance = std::numeric_limits<double>::infinity();
  for (std::vector<Vec3>::const_iterator it = candidates.begin(); it != candidates.end(); ++it) {
    if (distanceToObstacle(*it) < clearance) continue;
    const double candidate_distance = distance(*it, target);
    if (!found || candidate_distance < best_distance) {
      *selected = *it;
      best_distance = candidate_distance;
      found = true;
    }
  }
  return found;
}

std::vector<Vec3> RollingOccupancyMap::occupiedPoints() const {
  std::vector<Vec3> result;
  for (std::map<Index, Cell>::const_iterator it = cells_.begin(); it != cells_.end(); ++it) {
    if (it->second.log_odds > config_.occupied_threshold) result.push_back(centerOf(it->first));
  }
  return result;
}

MapDelta RollingOccupancyMap::takeDelta() { MapDelta value = delta_; delta_ = MapDelta(); return value; }
bool RollingOccupancyMap::Index::operator<(const Index& other) const { if (x != other.x) return x < other.x; if (y != other.y) return y < other.y; return z < other.z; }
RollingOccupancyMap::Index RollingOccupancyMap::indexOf(const Vec3& point) const { return Index{static_cast<int>(std::floor(point.x / config_.resolution)), static_cast<int>(std::floor(point.y / config_.resolution)), static_cast<int>(std::floor(point.z / config_.resolution))}; }
Vec3 RollingOccupancyMap::centerOf(const Index& index) const { return Vec3((index.x + 0.5) * config_.resolution, (index.y + 0.5) * config_.resolution, (index.z + 0.5) * config_.resolution); }
void RollingOccupancyMap::update(const Index& index, double increment, double stamp) { Cell& cell = cells_[index]; const bool occupied_before = cell.log_odds > config_.occupied_threshold; cell.log_odds = std::max(-config_.evidence_limit, std::min(config_.evidence_limit, cell.log_odds + increment)); cell.stamp = stamp; const bool occupied_after = cell.log_odds > config_.occupied_threshold; if (!occupied_before && occupied_after) delta_.occupied_added.push_back(centerOf(index)); if (occupied_before && !occupied_after) delta_.occupied_removed.push_back(centerOf(index)); }
void RollingOccupancyMap::prune() { for (std::map<Index, Cell>::iterator it = cells_.begin(); it != cells_.end();) { if (distance(center_, centerOf(it->first)) > config_.window_radius) { if (it->second.log_odds > config_.occupied_threshold) delta_.occupied_removed.push_back(centerOf(it->first)); it = cells_.erase(it); } else ++it; } }
double RollingOccupancyMap::distance(const Vec3& a, const Vec3& b) { const double x=a.x-b.x,y=a.y-b.y,z=a.z-b.z; return std::sqrt(x*x+y*y+z*z); }

}
