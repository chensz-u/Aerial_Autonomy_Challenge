#include "c5_autonomous_exploration/autonomous_exploration.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace c5_autonomous_exploration {

namespace {

const int kNeighborCount = 6;
const int kNeighborOffsets[kNeighborCount][3] = {
    {1, 0, 0}, {-1, 0, 0}, {0, 1, 0},
    {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};

}

Vec3::Vec3() : x(0.0), y(0.0), z(0.0) {}

Vec3::Vec3(double x_value, double y_value, double z_value)
    : x(x_value), y(y_value), z(z_value) {}

ExplorerConfig::ExplorerConfig()
    : map_resolution(0.25),
      occupied_log_odds(0.9),
      free_log_odds(-0.6),
      map_log_odds_limit(4.0),
      min_clearance(0.6),
      frontier_gain_weight(1.0),
      distance_weight(0.25),
      home_distance_weight(0.1),
      goal_min_distance(1.0),
      nbs_beam_width(8),
      nbs_search_depth(3),
      rrag_connection_distance(5.0),
      fls_search_radius(4.0),
      return_battery_fraction(0.25),
      land_battery_fraction(0.12),
      link_loss_continue_seconds(30.0),
      estimator_covariance_limit(0.5),
      minimum_confidence(0.45),
      target_association_radius(1.0),
      target_confirmation_confidence(0.65),
      max_planner_failures(2) {}

ExplorationDecision::ExplorationDecision()
    : has_goal(false), goal(), score(0.0), mode(SafetyState::kHold), event(SafetyEvent::kNone) {}

bool AutonomousExplorer::VoxelIndex::operator==(const VoxelIndex& other) const {
  return x == other.x && y == other.y && z == other.z;
}

bool AutonomousExplorer::VoxelIndex::operator<(const VoxelIndex& other) const {
  if (x != other.x) return x < other.x;
  if (y != other.y) return y < other.y;
  return z < other.z;
}

AutonomousExplorer::AutonomousExplorer(const ExplorerConfig& config)
    : config_(config),
      home_(),
      pose_(),
      home_valid_(false),
      pose_valid_(false),
      pose_stamp_(0.0),
      battery_fraction_(1.0),
      link_available_(true),
      link_lost_since_(-1.0),
      lio_confidence_(0.0),
      vio_confidence_(0.0),
      fused_confidence_(0.0),
      planner_failures_(0),
      task_interrupted_(false),
      safety_state_(SafetyState::kHold),
      active_event_(SafetyEvent::kEstimatorDegraded) {}

void AutonomousExplorer::setHome(const Vec3& home) {
  home_ = home;
  home_valid_ = true;
}

void AutonomousExplorer::updatePose(const Vec3& pose, double stamp) {
  pose_ = pose;
  pose_stamp_ = stamp;
  pose_valid_ = true;
}

void AutonomousExplorer::updateBattery(double fraction) {
  battery_fraction_ = clamp(fraction, 0.0, 1.0);
}

void AutonomousExplorer::updateLink(bool available, double stamp) {
  if (!available && link_available_) link_lost_since_ = stamp;
  if (available) link_lost_since_ = -1.0;
  link_available_ = available;
}

void AutonomousExplorer::updateEstimator(double lio_covariance, double vio_covariance,
                                         double vio_initialization, double lio_quality) {
  const double lio_covariance_confidence = 1.0 / (1.0 + std::max(0.0, lio_covariance));
  const double vio_covariance_confidence = 1.0 / (1.0 + std::max(0.0, vio_covariance));
  lio_confidence_ = clamp(lio_covariance_confidence * clamp(lio_quality, 0.0, 1.0), 0.0, 1.0);
  vio_confidence_ = clamp(vio_covariance_confidence * clamp(vio_initialization, 0.0, 1.0), 0.0, 1.0);
  const double lio_weight = lio_confidence_ * lio_confidence_;
  const double vio_weight = vio_confidence_ * vio_confidence_;
  fused_confidence_ = clamp((lio_weight + vio_weight) / 2.0, 0.0, 1.0);
}

void AutonomousExplorer::setVoxel(const Vec3& position, CellState state) {
  const VoxelIndex index = indexOf(position);
  if (state == CellState::kOccupied) {
    voxels_[index] = config_.map_log_odds_limit;
  } else if (state == CellState::kFree) {
    voxels_[index] = -config_.map_log_odds_limit;
  } else {
    voxels_.erase(index);
  }
}

void AutonomousExplorer::integrateRay(const Vec3& origin, const Vec3& endpoint) {
  const double length = distance(origin, endpoint);
  if (length <= 1e-9) {
    setVoxel(endpoint, CellState::kOccupied);
    return;
  }
  const int steps = std::max(1, static_cast<int>(std::ceil(length / (config_.map_resolution * 0.5))));
  for (int i = 0; i < steps; ++i) {
    const double ratio = static_cast<double>(i) / static_cast<double>(steps);
    const Vec3 point(origin.x + (endpoint.x - origin.x) * ratio,
                     origin.y + (endpoint.y - origin.y) * ratio,
                     origin.z + (endpoint.z - origin.z) * ratio);
    updateEvidence(indexOf(point), config_.free_log_odds);
  }
  updateEvidence(indexOf(endpoint), config_.occupied_log_odds);
}

std::vector<Vec3> AutonomousExplorer::frontiers() const {
  std::vector<Vec3> result;
  for (std::map<VoxelIndex, double>::const_iterator it = voxels_.begin(); it != voxels_.end(); ++it) {
    if (isFrontier(it->first)) result.push_back(centerOf(it->first));
  }
  return result;
}

std::vector<Vec3> AutonomousExplorer::occupiedVoxels() const {
  std::vector<Vec3> result;
  for (std::map<VoxelIndex, double>::const_iterator it = voxels_.begin(); it != voxels_.end(); ++it) {
    if (cellAt(it->first) == CellState::kOccupied) result.push_back(centerOf(it->first));
  }
  return result;
}

ExplorationDecision AutonomousExplorer::planNextGoal() {
  ExplorationDecision decision;
  const SafetyState current_state = evaluateSafety(pose_stamp_);
  decision.mode = current_state;
  decision.event = active_event_;
  if (current_state == SafetyState::kReturn && home_valid_) {
    decision.has_goal = true;
    decision.goal = home_;
    decision.score = 0.0;
    return decision;
  }
  if (current_state != SafetyState::kExplore || !pose_valid_) return decision;

  std::vector<VoxelIndex> candidates;
  const std::vector<std::vector<VoxelIndex>> clusters = frontierClusters();
  for (std::vector<std::vector<VoxelIndex>>::const_iterator cluster = clusters.begin(); cluster != clusters.end(); ++cluster) {
    VoxelIndex selected = cluster->front();
    double selected_score = -std::numeric_limits<double>::infinity();
    for (std::vector<VoxelIndex>::const_iterator voxel = cluster->begin(); voxel != cluster->end(); ++voxel) {
      if (distance(pose_, centerOf(*voxel)) < config_.goal_min_distance || !visibleFromPose(*voxel)) continue;
      const double score = frontierScore(*voxel);
      if (score > selected_score) { selected = *voxel; selected_score = score; }
    }
    if (selected_score > -std::numeric_limits<double>::infinity()) candidates.push_back(selected);
  }

  struct Beam {
    std::vector<VoxelIndex> path;
    double score;
  };

  std::vector<Beam> beams;
  for (std::vector<VoxelIndex>::const_iterator it = candidates.begin(); it != candidates.end(); ++it) {
    Beam beam;
    beam.path.push_back(*it);
    beam.score = frontierScore(*it) + ringConnectivity(*it, candidates);
    beams.push_back(beam);
  }
  std::sort(beams.begin(), beams.end(), [](const Beam& left, const Beam& right) {
    return left.score > right.score;
  });
  if (beams.size() > static_cast<std::size_t>(config_.nbs_beam_width)) {
    beams.resize(static_cast<std::size_t>(config_.nbs_beam_width));
  }
  for (int depth = 1; depth < config_.nbs_search_depth && !beams.empty(); ++depth) {
    std::vector<Beam> expanded;
    for (std::vector<Beam>::const_iterator beam = beams.begin(); beam != beams.end(); ++beam) {
      for (std::vector<VoxelIndex>::const_iterator candidate = candidates.begin(); candidate != candidates.end(); ++candidate) {
        if (std::find(beam->path.begin(), beam->path.end(), *candidate) != beam->path.end()) continue;
        Beam next = *beam;
        const Vec3 prior = centerOf(next.path.back());
        const Vec3 current = centerOf(*candidate);
        next.path.push_back(*candidate);
        next.score += frontierScore(*candidate) - config_.distance_weight * distance(prior, current) +
                      ringConnectivity(*candidate, candidates);
        expanded.push_back(next);
      }
    }
    std::sort(expanded.begin(), expanded.end(), [](const Beam& left, const Beam& right) {
      return left.score > right.score;
    });
    if (expanded.size() > static_cast<std::size_t>(config_.nbs_beam_width)) {
      expanded.resize(static_cast<std::size_t>(config_.nbs_beam_width));
    }
    beams.swap(expanded);
  }

  if (!beams.empty()) {
    decision.has_goal = true;
    decision.goal = centerOf(beams.front().path.front());
    decision.score = beams.front().score;
  } else {
    VoxelIndex fallback = {0, 0, 0};
    if (fallbackGoal(&fallback)) {
      decision.has_goal = true;
      decision.goal = centerOf(fallback);
      decision.score = frontierScore(fallback);
    }
  }
  return decision;
}

SafetyState AutonomousExplorer::evaluateSafety(double stamp) {
  active_event_ = SafetyEvent::kNone;
  if (battery_fraction_ <= config_.land_battery_fraction) {
    safety_state_ = SafetyState::kLand;
    active_event_ = SafetyEvent::kBatteryLow;
    return safety_state_;
  }
  if (task_interrupted_) {
    safety_state_ = SafetyState::kReturn;
    active_event_ = SafetyEvent::kTaskInterrupt;
    return safety_state_;
  }
  if (planner_failures_ >= config_.max_planner_failures) {
    safety_state_ = SafetyState::kReturn;
    active_event_ = SafetyEvent::kPlannerFailure;
    return safety_state_;
  }
  if (battery_fraction_ <= config_.return_battery_fraction) {
    safety_state_ = SafetyState::kReturn;
    active_event_ = SafetyEvent::kBatteryLow;
    return safety_state_;
  }
  if (!link_available_ && link_lost_since_ >= 0.0 &&
      stamp - link_lost_since_ >= config_.link_loss_continue_seconds) {
    safety_state_ = SafetyState::kReturn;
    active_event_ = SafetyEvent::kLinkLost;
    return safety_state_;
  }
  if (!pose_valid_ || fused_confidence_ < config_.minimum_confidence) {
    safety_state_ = SafetyState::kHold;
    active_event_ = SafetyEvent::kEstimatorDegraded;
    return safety_state_;
  }
  safety_state_ = SafetyState::kExplore;
  return safety_state_;
}

void AutonomousExplorer::reportTaskInterrupt() { task_interrupted_ = true; }

void AutonomousExplorer::reportPlannerFailure() { ++planner_failures_; }

void AutonomousExplorer::clearPlannerFailures() { planner_failures_ = 0; }

void AutonomousExplorer::addDetection(const Detection& detection) {
  if (detection.confidence < config_.minimum_confidence) return;
  for (std::vector<TargetEstimate>::iterator it = tracks_.begin(); it != tracks_.end(); ++it) {
    if (it->label != detection.label || distance(it->position, detection.position) > config_.target_association_radius) continue;
    const double prior_weight = std::max(1, it->observations);
    it->position.x = (it->position.x * prior_weight + detection.position.x) / (prior_weight + 1.0);
    it->position.y = (it->position.y * prior_weight + detection.position.y) / (prior_weight + 1.0);
    it->position.z = (it->position.z * prior_weight + detection.position.z) / (prior_weight + 1.0);
    it->confidence = 1.0 - (1.0 - it->confidence) * (1.0 - detection.confidence);
    ++it->observations;
    it->last_stamp = detection.stamp;
    return;
  }
  TargetEstimate track;
  track.position = detection.position;
  track.confidence = detection.confidence;
  track.label = detection.label;
  track.observations = 1;
  track.last_stamp = detection.stamp;
  tracks_.push_back(track);
}

std::vector<TargetEstimate> AutonomousExplorer::confirmedTargets() const {
  std::vector<TargetEstimate> result;
  for (std::vector<TargetEstimate>::const_iterator it = tracks_.begin(); it != tracks_.end(); ++it) {
    if (it->confidence >= config_.target_confirmation_confidence) result.push_back(*it);
  }
  return result;
}

ExplorerStatus AutonomousExplorer::status() const {
  ExplorerStatus value;
  value.pose = pose_;
  value.pose_valid = pose_valid_;
  value.battery_fraction = battery_fraction_;
  value.link_available = link_available_;
  value.lio_confidence = lio_confidence_;
  value.vio_confidence = vio_confidence_;
  value.fused_confidence = fused_confidence_;
  value.safety_state = safety_state_;
  value.active_event = active_event_;
  return value;
}

AutonomousExplorer::VoxelIndex AutonomousExplorer::indexOf(const Vec3& point) const {
  VoxelIndex index;
  index.x = static_cast<int>(std::floor(point.x / config_.map_resolution));
  index.y = static_cast<int>(std::floor(point.y / config_.map_resolution));
  index.z = static_cast<int>(std::floor(point.z / config_.map_resolution));
  return index;
}

Vec3 AutonomousExplorer::centerOf(const VoxelIndex& index) const {
  return Vec3((static_cast<double>(index.x) + 0.5) * config_.map_resolution,
              (static_cast<double>(index.y) + 0.5) * config_.map_resolution,
              (static_cast<double>(index.z) + 0.5) * config_.map_resolution);
}

CellState AutonomousExplorer::cellAt(const VoxelIndex& index) const {
  const std::map<VoxelIndex, double>::const_iterator it = voxels_.find(index);
  if (it == voxels_.end()) return CellState::kUnknown;
  if (it->second > 0.0) return CellState::kOccupied;
  if (it->second < 0.0) return CellState::kFree;
  return CellState::kUnknown;
}

void AutonomousExplorer::updateEvidence(const VoxelIndex& index, double increment) {
  const std::map<VoxelIndex, double>::iterator found = voxels_.find(index);
  const double previous = found == voxels_.end() ? 0.0 : found->second;
  voxels_[index] = clamp(previous + increment, -config_.map_log_odds_limit, config_.map_log_odds_limit);
}

bool AutonomousExplorer::isFrontier(const VoxelIndex& index) const {
  return cellAt(index) == CellState::kFree && unknownNeighbors(index) > 0 && isSafe(index);
}

bool AutonomousExplorer::isSafe(const VoxelIndex& index) const {
  const Vec3 center = centerOf(index);
  for (std::map<VoxelIndex, double>::const_iterator it = voxels_.begin(); it != voxels_.end(); ++it) {
    if (cellAt(it->first) == CellState::kOccupied && distance(center, centerOf(it->first)) < config_.min_clearance) return false;
  }
  return true;
}

bool AutonomousExplorer::visibleFromPose(const VoxelIndex& index) const {
  if (!pose_valid_) return false;
  const Vec3 target = centerOf(index);
  const double length = distance(pose_, target);
  const int steps = std::max(1, static_cast<int>(std::ceil(length / (config_.map_resolution * 0.5))));
  for (int step = 1; step < steps; ++step) {
    const double ratio = static_cast<double>(step) / static_cast<double>(steps);
    const Vec3 point(pose_.x + ratio * (target.x - pose_.x), pose_.y + ratio * (target.y - pose_.y),
                     pose_.z + ratio * (target.z - pose_.z));
    if (cellAt(indexOf(point)) == CellState::kOccupied) return false;
  }
  return true;
}

std::vector<std::vector<AutonomousExplorer::VoxelIndex>> AutonomousExplorer::frontierClusters() const {
  std::vector<VoxelIndex> frontier_indices;
  for (std::map<VoxelIndex, double>::const_iterator it = voxels_.begin(); it != voxels_.end(); ++it) {
    if (isFrontier(it->first)) frontier_indices.push_back(it->first);
  }
  std::vector<std::vector<VoxelIndex>> clusters;
  std::vector<bool> assigned(frontier_indices.size(), false);
  for (std::size_t root = 0; root < frontier_indices.size(); ++root) {
    if (assigned[root]) continue;
    std::vector<VoxelIndex> cluster;
    std::vector<std::size_t> queue(1, root);
    assigned[root] = true;
    for (std::size_t cursor = 0; cursor < queue.size(); ++cursor) {
      const std::size_t current = queue[cursor];
      cluster.push_back(frontier_indices[current]);
      for (std::size_t candidate = 0; candidate < frontier_indices.size(); ++candidate) {
        if (assigned[candidate]) continue;
        const int dx = std::abs(frontier_indices[current].x - frontier_indices[candidate].x);
        const int dy = std::abs(frontier_indices[current].y - frontier_indices[candidate].y);
        const int dz = std::abs(frontier_indices[current].z - frontier_indices[candidate].z);
        if (dx + dy + dz == 1) { assigned[candidate] = true; queue.push_back(candidate); }
      }
    }
    clusters.push_back(cluster);
  }
  return clusters;
}

int AutonomousExplorer::unknownNeighbors(const VoxelIndex& index) const {
  int count = 0;
  for (int i = 0; i < kNeighborCount; ++i) {
    VoxelIndex neighbor = {index.x + kNeighborOffsets[i][0], index.y + kNeighborOffsets[i][1], index.z + kNeighborOffsets[i][2]};
    if (cellAt(neighbor) == CellState::kUnknown) ++count;
  }
  return count;
}

double AutonomousExplorer::frontierScore(const VoxelIndex& index) const {
  const Vec3 candidate = centerOf(index);
  const double gain = static_cast<double>(unknownNeighbors(index));
  const double travel = pose_valid_ ? distance(pose_, candidate) : 0.0;
  const double return_cost = home_valid_ ? distance(candidate, home_) : 0.0;
  return config_.frontier_gain_weight * gain - config_.distance_weight * travel -
         config_.home_distance_weight * (1.0 - battery_fraction_) * return_cost;
}

double AutonomousExplorer::ringConnectivity(const VoxelIndex& index,
                                             const std::vector<VoxelIndex>& candidates) const {
  const Vec3 center = centerOf(index);
  int links = 0;
  for (std::vector<VoxelIndex>::const_iterator it = candidates.begin(); it != candidates.end(); ++it) {
    if (it->x == index.x && it->y == index.y && it->z == index.z) continue;
    if (distance(center, centerOf(*it)) <= config_.rrag_connection_distance) ++links;
  }
  return 0.25 * static_cast<double>(std::min(2, links));
}

bool AutonomousExplorer::fallbackGoal(VoxelIndex* index) const {
  if (!pose_valid_ || index == 0) return false;
  double best_score = -std::numeric_limits<double>::infinity();
  bool found = false;
  for (std::map<VoxelIndex, double>::const_iterator it = voxels_.begin(); it != voxels_.end(); ++it) {
    if (cellAt(it->first) != CellState::kFree || !isSafe(it->first)) continue;
    const double travel = distance(pose_, centerOf(it->first));
    if (travel < config_.goal_min_distance || travel > config_.fls_search_radius) continue;
    const double score = static_cast<double>(unknownNeighbors(it->first)) - config_.distance_weight * travel;
    if (!found || score > best_score) {
      *index = it->first;
      best_score = score;
      found = true;
    }
  }
  return found;
}

double AutonomousExplorer::clamp(double value, double minimum, double maximum) {
  return std::max(minimum, std::min(maximum, value));
}

double AutonomousExplorer::distance(const Vec3& a, const Vec3& b) {
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  const double dz = a.z - b.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

}
