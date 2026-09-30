#include "c5_autonomous_exploration/loop_closure.hpp"

#include <algorithm>
#include <cmath>
#include <set>

namespace c5_autonomous_exploration {

LoopConfig::LoopConfig() : minimum_similarity(0.82), minimum_label_similarity(0.2), minimum_time_separation(2.0), optimization_steps(6) {}
LoopClosureDatabase::LoopClosureDatabase(const LoopConfig& config) : config_(config) {}

void LoopClosureDatabase::add(const PlaceKeyframe& keyframe) { keyframes_.push_back(keyframe); }

LoopClosure LoopClosureDatabase::query(const PlaceKeyframe& keyframe) const {
  LoopClosure result;
  result.accepted = false;
  result.query_id = keyframe.id;
  result.match_id = -1;
  result.similarity = 0.0;
  result.correction = Vec3();
  for (const PlaceKeyframe& candidate : keyframes_) {
    if (candidate.id == keyframe.id) continue;
    if (std::abs(candidate.stamp - keyframe.stamp) < config_.minimum_time_separation) continue;
    const double visual = cosine(candidate.descriptor, keyframe.descriptor);
    const double semantic = labelSimilarity(candidate.labels, keyframe.labels);
    const double combined = 0.85 * visual + 0.15 * semantic;
    if (visual >= config_.minimum_similarity && semantic >= config_.minimum_label_similarity && combined > result.similarity) {
      result.accepted = true;
      result.match_id = candidate.id;
      result.similarity = combined;
      result.correction = Vec3(candidate.position.x - keyframe.position.x,
                               candidate.position.y - keyframe.position.y,
                               candidate.position.z - keyframe.position.z);
    }
  }
  return result;
}

bool LoopClosureDatabase::addAndOptimize(const PlaceKeyframe& keyframe) {
  const LoopClosure closure = query(keyframe);
  if (closure.accepted) {
    constraints_.push_back(PoseConstraint{keyframe.id, closure.match_id, closure.correction,
                                           std::max(0.01, closure.similarity)});
  }
  keyframes_.push_back(keyframe);
  optimize();
  return closure.accepted;
}

std::vector<PlaceKeyframe> LoopClosureDatabase::keyframes() const { return keyframes_; }
std::vector<PoseConstraint> LoopClosureDatabase::constraints() const { return constraints_; }

void LoopClosureDatabase::optimize() {
  if (keyframes_.size() < 2 || constraints_.empty()) return;
  for (int iteration = 0; iteration < config_.optimization_steps; ++iteration) {
    for (const PoseConstraint& constraint : constraints_) {
      std::vector<PlaceKeyframe>::iterator source = std::find_if(keyframes_.begin(), keyframes_.end(),
          [&constraint](const PlaceKeyframe& keyframe) { return keyframe.id == constraint.source_id; });
      std::vector<PlaceKeyframe>::iterator target = std::find_if(keyframes_.begin(), keyframes_.end(),
          [&constraint](const PlaceKeyframe& keyframe) { return keyframe.id == constraint.target_id; });
      if (source == keyframes_.end() || target == keyframes_.end()) continue;
      const Vec3 error((target->position.x - source->position.x) - constraint.translation.x,
                       (target->position.y - source->position.y) - constraint.translation.y,
                       (target->position.z - source->position.z) - constraint.translation.z);
      const double step = 0.5 * constraint.weight / static_cast<double>(iteration + 1);
      source->position.x += step * error.x;
      source->position.y += step * error.y;
      source->position.z += step * error.z;
      target->position.x -= step * error.x;
      target->position.y -= step * error.y;
      target->position.z -= step * error.z;
    }
  }
}

double LoopClosureDatabase::cosine(const std::vector<double>& first, const std::vector<double>& second) {
  if (first.empty() || first.size() != second.size()) return 0.0;
  double dot = 0.0;
  double left_norm = 0.0;
  double right_norm = 0.0;
  for (std::size_t index = 0; index < first.size(); ++index) {
    dot += first[index] * second[index];
    left_norm += first[index] * first[index];
    right_norm += second[index] * second[index];
  }
  if (left_norm < 1e-12 || right_norm < 1e-12) return 0.0;
  return dot / std::sqrt(left_norm * right_norm);
}

double LoopClosureDatabase::labelSimilarity(const std::vector<std::string>& first,
                                            const std::vector<std::string>& second) {
  if (first.empty() || second.empty()) return 0.0;
  const std::set<std::string> left(first.begin(), first.end());
  const std::set<std::string> right(second.begin(), second.end());
  int intersection = 0;
  for (std::set<std::string>::const_iterator it = left.begin(); it != left.end(); ++it) {
    if (right.count(*it) != 0) ++intersection;
  }
  const int union_size = static_cast<int>(left.size() + right.size()) - intersection;
  return union_size == 0 ? 0.0 : static_cast<double>(intersection) / static_cast<double>(union_size);
}

}
