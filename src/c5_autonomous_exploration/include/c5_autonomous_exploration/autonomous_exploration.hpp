#pragma once

#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace c5_autonomous_exploration {

struct Vec3 {
  double x;
  double y;
  double z;
  Vec3();
  Vec3(double x_value, double y_value, double z_value);
};

enum class CellState : std::uint8_t {
  kUnknown = 0,
  kFree = 1,
  kOccupied = 2
};

enum class SafetyState : std::uint8_t {
  kExplore = 0,
  kHold = 1,
  kReturn = 2,
  kLand = 3
};

enum class SafetyEvent : std::uint8_t {
  kNone = 0,
  kTaskInterrupt = 1,
  kLinkLost = 2,
  kBatteryLow = 3,
  kEstimatorDegraded = 4,
  kPlannerFailure = 5
};

struct ExplorerConfig {
  double map_resolution;
  double occupied_log_odds;
  double free_log_odds;
  double map_log_odds_limit;
  double min_clearance;
  double frontier_gain_weight;
  double distance_weight;
  double home_distance_weight;
  double goal_min_distance;
  double min_goal_altitude;
  double max_goal_altitude;
  int nbs_beam_width;
  int nbs_search_depth;
  double rrag_connection_distance;
  double fls_search_radius;
  double return_battery_fraction;
  double land_battery_fraction;
  double link_loss_continue_seconds;
  double estimator_covariance_limit;
  double minimum_confidence;
  double target_association_radius;
  double target_confirmation_confidence;
  int max_planner_failures;
  ExplorerConfig();
};

struct Detection {
  Vec3 position;
  double confidence;
  std::string label;
  double stamp;
};

struct TargetEstimate {
  Vec3 position;
  double confidence;
  std::string label;
  int observations;
  double last_stamp;
};

struct ExplorationDecision {
  bool has_goal;
  Vec3 goal;
  double score;
  SafetyState mode;
  SafetyEvent event;
  ExplorationDecision();
};

struct ExplorerStatus {
  Vec3 pose;
  bool pose_valid;
  double battery_fraction;
  bool link_available;
  double lio_confidence;
  double vio_confidence;
  double fused_confidence;
  SafetyState safety_state;
  SafetyEvent active_event;
};

class AutonomousExplorer {
 public:
  explicit AutonomousExplorer(const ExplorerConfig& config);
  void setHome(const Vec3& home);
  void updatePose(const Vec3& pose, double stamp);
  void updateBattery(double fraction);
  void updateLink(bool available, double stamp);
  void updateEstimator(double lio_covariance, double vio_covariance,
                       double vio_initialization, double lio_quality);
  void setVoxel(const Vec3& position, CellState state);
  void integrateRay(const Vec3& origin, const Vec3& endpoint);
  std::vector<Vec3> frontiers() const;
  std::vector<Vec3> occupiedVoxels() const;
  ExplorationDecision planNextGoal();
  SafetyState evaluateSafety(double stamp);
  void reportTaskInterrupt();
  void reportPlannerFailure();
  void clearPlannerFailures();
  void addDetection(const Detection& detection);
  std::vector<TargetEstimate> confirmedTargets() const;
  ExplorerStatus status() const;

 private:
  struct VoxelIndex {
    int x;
    int y;
    int z;
    bool operator==(const VoxelIndex& other) const;
    bool operator<(const VoxelIndex& other) const;
  };

  ExplorerConfig config_;
  std::map<VoxelIndex, double> voxels_;
  Vec3 home_;
  Vec3 pose_;
  bool home_valid_;
  bool pose_valid_;
  double pose_stamp_;
  double battery_fraction_;
  bool link_available_;
  double link_lost_since_;
  double lio_confidence_;
  double vio_confidence_;
  double fused_confidence_;
  int planner_failures_;
  bool task_interrupted_;
  SafetyState safety_state_;
  SafetyEvent active_event_;
  std::vector<TargetEstimate> tracks_;

  VoxelIndex indexOf(const Vec3& point) const;
  Vec3 centerOf(const VoxelIndex& index) const;
  CellState cellAt(const VoxelIndex& index) const;
  void updateEvidence(const VoxelIndex& index, double increment);
  bool isFrontier(const VoxelIndex& index) const;
  bool isSafe(const VoxelIndex& index) const;
  bool visibleFromPose(const VoxelIndex& index) const;
  std::vector<std::vector<VoxelIndex>> frontierClusters() const;
  int unknownNeighbors(const VoxelIndex& index) const;
  double frontierScore(const VoxelIndex& index) const;
  double ringConnectivity(const VoxelIndex& index, const std::vector<VoxelIndex>& candidates) const;
  bool fallbackGoal(VoxelIndex* index) const;
  static double clamp(double value, double minimum, double maximum);
  static double distance(const Vec3& a, const Vec3& b);
};

}
