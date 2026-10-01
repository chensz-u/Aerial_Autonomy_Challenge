#pragma once

#include <cstdint>
#include <vector>

namespace diff_planner {

struct BridgePoint {
  double x;
  double y;
  double z;
  BridgePoint();
  BridgePoint(double x_value, double y_value, double z_value);
};

enum class PlannerRequestMode : std::uint8_t {
  kExplore = 0,
  kReturnHome = 1,
  kHold = 2
};

enum class BridgeFeedbackState : std::uint8_t {
  kAccepted = 0,
  kRejectedNoMap = 1,
  kRejectedMap = 2,
  kRejectedStale = 3,
  kActive = 4,
  kArrived = 5,
  kCommandTimeout = 6,
  kGoalTimeout = 7,
  kHeartbeatTimeout = 8,
  kMapStale = 9,
  kPlannerRejected = 10,
  kStopped = 11
};

struct PlannerBridgeConfig {
  double minimum_clearance;
  double command_timeout;
  double goal_timeout;
  double heartbeat_timeout;
  double map_timeout;
  double arrival_tolerance;
  bool require_map;
  bool require_clear_path;
  PlannerBridgeConfig();
};

struct PlannerBridgeRequest {
  std::uint32_t sequence;
  PlannerRequestMode mode;
  BridgePoint goal;
  double clearance;
  double stamp;
};

struct PlannerBridgeFeedback {
  std::uint32_t sequence;
  BridgeFeedbackState state;
  bool accepted;
  bool active;
  bool success;
  BridgePoint goal;
  double distance_to_goal;
  double stamp;
  PlannerBridgeFeedback();
};

class C5PlannerBridgeCore {
 public:
  explicit C5PlannerBridgeCore(const PlannerBridgeConfig& config = PlannerBridgeConfig());
  void updateMap(const std::vector<BridgePoint>& occupied, double stamp);
  PlannerBridgeFeedback updatePosition(const BridgePoint& position, double stamp);
  void observePlannerHeartbeat(double stamp);
  PlannerBridgeFeedback submit(const PlannerBridgeRequest& request, const BridgePoint& origin);
  PlannerBridgeFeedback markPlannerCommand(bool accepted, double stamp);
  PlannerBridgeFeedback tick(double stamp);
  PlannerBridgeFeedback stop(double stamp);
  const PlannerBridgeFeedback& feedback() const;

 private:
  PlannerBridgeConfig config_;
  std::vector<BridgePoint> occupied_;
  PlannerBridgeRequest request_;
  PlannerBridgeFeedback feedback_;
  BridgePoint position_;
  bool map_ready_;
  bool position_ready_;
  bool active_;
  bool command_seen_;
  std::uint32_t last_sequence_;
  double last_request_stamp_;
  double map_stamp_;
  double heartbeat_stamp_;

  bool pathClear(const BridgePoint& origin, const BridgePoint& target, double clearance) const;
  bool goalClear(const BridgePoint& target, double clearance) const;
  void setFeedback(BridgeFeedbackState state, bool accepted, bool active, bool success, double stamp);
  static double distance(const BridgePoint& left, const BridgePoint& right);
  static double distanceToSegment(const BridgePoint& point, const BridgePoint& start, const BridgePoint& end);
};

}
