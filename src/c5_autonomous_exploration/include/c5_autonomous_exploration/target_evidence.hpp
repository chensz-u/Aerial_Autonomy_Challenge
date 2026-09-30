#pragma once

#include "c5_autonomous_exploration/autonomous_exploration.hpp"

#include <string>
#include <vector>

namespace c5_autonomous_exploration {

struct EvidenceConfig {
  double association_radius;
  double minimum_observation_reliability;
  double confirmation_belief;
  EvidenceConfig();
};

struct TargetEvidence {
  std::string label;
  Vec3 position;
  double belief;
  double disbelief;
  double uncertainty;
  double stamp;
  int observations;
};

class TargetEvidenceFusion {
 public:
  explicit TargetEvidenceFusion(const EvidenceConfig& config = EvidenceConfig());
  void add(const std::string& label, double detector_confidence, double tracking_confidence,
           const Vec3& position, double stamp);
  TargetEvidence best() const;
  std::vector<TargetEvidence> confirmed() const;

 private:
  EvidenceConfig config_;
  std::vector<TargetEvidence> evidence_;

  static double distance(const Vec3& first, const Vec3& second);
  static double clamp(double value, double minimum, double maximum);
};

}
