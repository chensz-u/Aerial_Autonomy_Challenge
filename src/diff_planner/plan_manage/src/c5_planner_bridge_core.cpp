#include <plan_manage/c5_planner_bridge_core.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace diff_planner {

BridgePoint::BridgePoint() : x(0.0), y(0.0), z(0.0) {}

BridgePoint::BridgePoint(double x_value, double y_value, double z_value)
    : x(x_value), y(y_value), z(z_value) {}

PlannerBridgeConfig::PlannerBridgeConfig()
    : minimum_clearance(0.6), command_timeout(1.5), goal_timeout(12.0), heartbeat_timeout(1.0),
      map_timeout(2.0), arrival_tolerance(0.35), require_map(true) {}

PlannerBridgeFeedback::PlannerBridgeFeedback()
    : sequence(0), state(BridgeFeedbackState::kRejectedNoMap), accepted(false), active(false), success(false),
      goal(), distance_to_goal(std::numeric_limits<double>::infinity()), stamp(0.0) {}

C5PlannerBridgeCore::C5PlannerBridgeCore(const PlannerBridgeConfig& config)
    : config_(config), occupied_(), request_{0, PlannerRequestMode::kExplore, BridgePoint(), 0.0, 0.0}, feedback_(),
      position_(), map_ready_(false), position_ready_(false), active_(false), command_seen_(false), last_sequence_(0),
      map_stamp_(-std::numeric_limits<double>::infinity()), heartbeat_stamp_(-std::numeric_limits<double>::infinity()) {}

void C5PlannerBridgeCore::updateMap(const std::vector<BridgePoint>& occupied, double stamp) {
  occupied_ = occupied;
  map_ready_ = true;
  map_stamp_ = stamp;
  if (active_ && position_ready_ && !pathClear(position_, request_.goal, std::max(config_.minimum_clearance, request_.clearance))) {
    active_ = false;
    setFeedback(BridgeFeedbackState::kRejectedMap, false, false, false, stamp);
  }
}

PlannerBridgeFeedback C5PlannerBridgeCore::updatePosition(const BridgePoint& position, double stamp) {
  position_ = position;
  position_ready_ = true;
  if (!active_ || !command_seen_) return feedback_;
  feedback_.distance_to_goal = distance(position_, request_.goal);
  if (feedback_.distance_to_goal <= config_.arrival_tolerance) {
    active_ = false;
    setFeedback(BridgeFeedbackState::kArrived, true, false, true, stamp);
  }
  return feedback_;
}

void C5PlannerBridgeCore::observePlannerHeartbeat(double stamp) {
  heartbeat_stamp_ = stamp;
}

PlannerBridgeFeedback C5PlannerBridgeCore::submit(const PlannerBridgeRequest& request, const BridgePoint& origin) {
  if (request.sequence <= last_sequence_) {
    setFeedback(BridgeFeedbackState::kRejectedStale, false, false, false, request.stamp);
    feedback_.sequence = request.sequence;
    feedback_.goal = request.goal;
    return feedback_;
  }
  last_sequence_ = request.sequence;
  request_ = request;
  feedback_.sequence = request.sequence;
  feedback_.goal = request.goal;
  feedback_.distance_to_goal = distance(origin, request.goal);
  if (config_.require_map && !map_ready_) {
    active_ = false;
    command_seen_ = false;
    setFeedback(BridgeFeedbackState::kRejectedNoMap, false, false, false, request.stamp);
    return feedback_;
  }
  const double clearance = std::max(config_.minimum_clearance, request.clearance);
  if (!pathClear(origin, request.goal, clearance)) {
    active_ = false;
    command_seen_ = false;
    setFeedback(BridgeFeedbackState::kRejectedMap, false, false, false, request.stamp);
    return feedback_;
  }
  active_ = true;
  command_seen_ = false;
  setFeedback(BridgeFeedbackState::kAccepted, true, true, false, request.stamp);
  return feedback_;
}

PlannerBridgeFeedback C5PlannerBridgeCore::markPlannerCommand(bool accepted, double stamp) {
  if (!active_) return feedback_;
  if (!accepted) {
    active_ = false;
    command_seen_ = false;
    setFeedback(BridgeFeedbackState::kPlannerRejected, false, false, false, stamp);
    return feedback_;
  }
  command_seen_ = true;
  setFeedback(BridgeFeedbackState::kActive, true, true, false, stamp);
  return feedback_;
}

PlannerBridgeFeedback C5PlannerBridgeCore::tick(double stamp) {
  if (!active_) return feedback_;
  if (config_.require_map && stamp - map_stamp_ > config_.map_timeout) {
    active_ = false;
    setFeedback(BridgeFeedbackState::kMapStale, false, false, false, stamp);
    return feedback_;
  }
  if (!command_seen_ && stamp - request_.stamp > config_.command_timeout) {
    active_ = false;
    setFeedback(BridgeFeedbackState::kCommandTimeout, false, false, false, stamp);
    return feedback_;
  }
  if (command_seen_ && stamp - heartbeat_stamp_ > config_.heartbeat_timeout) {
    active_ = false;
    setFeedback(BridgeFeedbackState::kHeartbeatTimeout, false, false, false, stamp);
    return feedback_;
  }
  if (stamp - request_.stamp > config_.goal_timeout) {
    active_ = false;
    setFeedback(BridgeFeedbackState::kGoalTimeout, false, false, false, stamp);
    return feedback_;
  }
  return feedback_;
}

PlannerBridgeFeedback C5PlannerBridgeCore::stop(double stamp) {
  active_ = false;
  command_seen_ = false;
  setFeedback(BridgeFeedbackState::kStopped, false, false, false, stamp);
  return feedback_;
}

const PlannerBridgeFeedback& C5PlannerBridgeCore::feedback() const {
  return feedback_;
}

bool C5PlannerBridgeCore::pathClear(const BridgePoint& origin, const BridgePoint& target, double clearance) const {
  for (const BridgePoint& point : occupied_) {
    if (distanceToSegment(point, origin, target) < clearance) return false;
  }
  return true;
}

void C5PlannerBridgeCore::setFeedback(BridgeFeedbackState state, bool accepted, bool active, bool success, double stamp) {
  feedback_.state = state;
  feedback_.accepted = accepted;
  feedback_.active = active;
  feedback_.success = success;
  feedback_.stamp = stamp;
  if (position_ready_) feedback_.distance_to_goal = distance(position_, request_.goal);
}

double C5PlannerBridgeCore::distance(const BridgePoint& left, const BridgePoint& right) {
  const double dx = left.x - right.x;
  const double dy = left.y - right.y;
  const double dz = left.z - right.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double C5PlannerBridgeCore::distanceToSegment(const BridgePoint& point, const BridgePoint& start, const BridgePoint& end) {
  const double dx = end.x - start.x;
  const double dy = end.y - start.y;
  const double dz = end.z - start.z;
  const double denominator = dx * dx + dy * dy + dz * dz;
  if (denominator < std::numeric_limits<double>::epsilon()) return distance(point, start);
  const double projection = ((point.x - start.x) * dx + (point.y - start.y) * dy + (point.z - start.z) * dz) / denominator;
  const double ratio = std::max(0.0, std::min(1.0, projection));
  return distance(point, BridgePoint(start.x + ratio * dx, start.y + ratio * dy, start.z + ratio * dz));
}

}
