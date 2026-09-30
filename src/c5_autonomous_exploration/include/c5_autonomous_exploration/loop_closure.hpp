#pragma once

#include "c5_autonomous_exploration/autonomous_exploration.hpp"

#include <string>
#include <vector>

namespace c5_autonomous_exploration {

struct PlaceKeyframe {
  int id;
  double stamp;
  Vec3 position;
  std::vector<double> descriptor;
  std::vector<std::string> labels;
};

struct LoopClosure {
  bool accepted;
  int query_id;
  int match_id;
  double similarity;
  Vec3 correction;
};

struct PoseConstraint {
  int source_id;
  int target_id;
  Vec3 translation;
  double weight;
};

struct LoopConfig {
  double minimum_similarity;
  double minimum_label_similarity;
  double minimum_time_separation;
  int optimization_steps;
  LoopConfig();
};

class LoopClosureDatabase {
 public:
  explicit LoopClosureDatabase(const LoopConfig& config = LoopConfig());
  void add(const PlaceKeyframe& keyframe);
  LoopClosure query(const PlaceKeyframe& keyframe) const;
  bool addAndOptimize(const PlaceKeyframe& keyframe);
  std::vector<PlaceKeyframe> keyframes() const;
  std::vector<PoseConstraint> constraints() const;

 private:
  LoopConfig config_;
  std::vector<PlaceKeyframe> keyframes_;
  std::vector<PoseConstraint> constraints_;
  void optimize();
  static double cosine(const std::vector<double>& first, const std::vector<double>& second);
  static double labelSimilarity(const std::vector<std::string>& first,
                                const std::vector<std::string>& second);
};

}
