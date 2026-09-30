#include "c5_autonomous_exploration/target_evidence.hpp"

#include <algorithm>
#include <cmath>

namespace c5_autonomous_exploration {

EvidenceConfig::EvidenceConfig()
    : association_radius(1.5), minimum_observation_reliability(0.25), confirmation_belief(0.75) {}

TargetEvidenceFusion::TargetEvidenceFusion(const EvidenceConfig& config) : config_(config) {}

void TargetEvidenceFusion::add(const std::string& label, double detector_confidence,
                               double tracking_confidence, const Vec3& position, double stamp) {
  const double reliability = clamp(detector_confidence, 0.0, 1.0) *
                             clamp(tracking_confidence, 0.0, 1.0);
  if (reliability < config_.minimum_observation_reliability) {
    return;
  }
  for (TargetEvidence& target : evidence_) {
    if (target.label == label && distance(target.position, position) <= config_.association_radius) {
      const double total_weight = static_cast<double>(target.observations) + reliability;
      target.position.x = (target.position.x * target.observations + position.x * reliability) / total_weight;
      target.position.y = (target.position.y * target.observations + position.y * reliability) / total_weight;
      target.position.z = (target.position.z * target.observations + position.z * reliability) / total_weight;
      target.belief = 1.0 - (1.0 - target.belief) * (1.0 - reliability);
      target.disbelief = (1.0 - target.belief) * (1.0 - reliability);
      target.uncertainty = std::max(0.0, 1.0 - target.belief - target.disbelief);
      target.stamp = stamp;
      ++target.observations;
      return;
    }
  }
  TargetEvidence target;
  target.label = label;
  target.position = position;
  target.belief = reliability;
  target.disbelief = 1.0 - reliability;
  target.uncertainty = 0.0;
  target.stamp = stamp;
  target.observations = 1;
  evidence_.push_back(target);
}

TargetEvidence TargetEvidenceFusion::best() const {
  if (evidence_.empty()) {
    return TargetEvidence{"", Vec3(), 0.0, 0.0, 1.0, 0.0, 0};
  }
  return *std::max_element(evidence_.begin(), evidence_.end(), [](const TargetEvidence& first,
                                                                    const TargetEvidence& second) {
    return first.belief < second.belief;
  });
}

std::vector<TargetEvidence> TargetEvidenceFusion::confirmed() const {
  std::vector<TargetEvidence> confirmed_targets;
  for (const TargetEvidence& target : evidence_) {
    if (target.belief >= config_.confirmation_belief) {
      confirmed_targets.push_back(target);
    }
  }
  return confirmed_targets;
}

double TargetEvidenceFusion::distance(const Vec3& first, const Vec3& second) {
  const double dx = first.x - second.x;
  const double dy = first.y - second.y;
  const double dz = first.z - second.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double TargetEvidenceFusion::clamp(double value, double minimum, double maximum) {
  return std::max(minimum, std::min(maximum, value));
}

}
