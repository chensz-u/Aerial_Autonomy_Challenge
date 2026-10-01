#include <plan_manage/c5_planner_bridge_core.hpp>

#include <cassert>
#include <vector>

using diff_planner::BridgeFeedbackState;
using diff_planner::BridgePoint;
using diff_planner::C5PlannerBridgeCore;
using diff_planner::PlannerBridgeConfig;
using diff_planner::PlannerBridgeRequest;
using diff_planner::PlannerRequestMode;

int main() {
  PlannerBridgeConfig config;
  config.minimum_clearance = 0.5;
  config.command_timeout = 1.0;
  config.goal_timeout = 8.0;
  config.arrival_tolerance = 0.25;
  C5PlannerBridgeCore bridge(config);

  bridge.updateMap(std::vector<BridgePoint>{BridgePoint(4.0, 2.0, 1.0)}, 0.0);
  const PlannerBridgeRequest clear_request{1, PlannerRequestMode::kExplore, BridgePoint(4.0, 0.0, 1.0), 0.5, 0.0};
  const auto accepted = bridge.submit(clear_request, BridgePoint(0.0, 0.0, 1.0));
  assert(accepted.state == BridgeFeedbackState::kAccepted);
  assert(accepted.accepted);

  const auto active = bridge.markPlannerCommand(true, 0.1);
  assert(active.state == BridgeFeedbackState::kActive);
  assert(active.active);
  const auto arrived = bridge.updatePosition(BridgePoint(3.9, 0.0, 1.0), 0.2);
  assert(arrived.state == BridgeFeedbackState::kArrived);
  assert(arrived.success);

  bridge.updateMap(std::vector<BridgePoint>{BridgePoint(2.0, 0.0, 1.0)}, 1.0);
  const PlannerBridgeRequest blocked_request{2, PlannerRequestMode::kExplore, BridgePoint(4.0, 0.0, 1.0), 0.5, 1.0};
  const auto blocked = bridge.submit(blocked_request, BridgePoint(0.0, 0.0, 1.0));
  assert(blocked.state == BridgeFeedbackState::kRejectedMap);
  assert(!blocked.accepted);

  bridge.updateMap(std::vector<BridgePoint>{BridgePoint(4.0, 2.0, 1.0)}, 2.0);
  const PlannerBridgeRequest timeout_request{3, PlannerRequestMode::kExplore, BridgePoint(4.0, 0.0, 1.0), 0.5, 2.0};
  assert(bridge.submit(timeout_request, BridgePoint(0.0, 0.0, 1.0)).state == BridgeFeedbackState::kAccepted);
  const auto command_timeout = bridge.tick(3.1);
  assert(command_timeout.state == BridgeFeedbackState::kCommandTimeout);
  assert(!command_timeout.success);

  const PlannerBridgeRequest return_request{4, PlannerRequestMode::kReturnHome, BridgePoint(0.0, 0.0, 1.0), 0.5, 4.0};
  assert(bridge.submit(return_request, BridgePoint(3.0, 0.0, 1.0)).state == BridgeFeedbackState::kAccepted);
  assert(bridge.markPlannerCommand(true, 4.1).state == BridgeFeedbackState::kActive);
  const auto stopped = bridge.stop(4.2);
  assert(stopped.state == BridgeFeedbackState::kStopped);
  assert(!stopped.active);

  const PlannerBridgeRequest restarted_request{1, PlannerRequestMode::kExplore, BridgePoint(4.0, 0.0, 1.0), 0.5, 5.0};
  assert(bridge.submit(restarted_request, BridgePoint(0.0, 0.0, 1.0)).state == BridgeFeedbackState::kAccepted);
  const PlannerBridgeRequest delayed_request{2, PlannerRequestMode::kExplore, BridgePoint(4.0, 0.0, 1.0), 0.5, 4.5};
  assert(bridge.submit(delayed_request, BridgePoint(0.0, 0.0, 1.0)).state == BridgeFeedbackState::kRejectedStale);

  PlannerBridgeConfig detour_config = config;
  detour_config.require_clear_path = false;
  C5PlannerBridgeCore detour_bridge(detour_config);
  detour_bridge.updateMap(std::vector<BridgePoint>{BridgePoint(2.0, 0.0, 1.0)}, 0.0);
  detour_bridge.updatePosition(BridgePoint(0.0, 0.0, 1.0), 0.0);
  const PlannerBridgeRequest detour_request{1, PlannerRequestMode::kExplore, BridgePoint(4.0, 0.0, 1.0), 0.5, 0.0};
  const auto detour_accepted = detour_bridge.submit(detour_request, BridgePoint(0.0, 0.0, 1.0));
  assert(detour_accepted.state == BridgeFeedbackState::kAccepted);
  assert(detour_bridge.markPlannerCommand(true, 0.1).active);
  detour_bridge.updateMap(std::vector<BridgePoint>{BridgePoint(2.0, 0.0, 1.0), BridgePoint(4.0, 0.0, 1.0)}, 0.2);
  assert(detour_bridge.feedback().state == BridgeFeedbackState::kRejectedMap);
  assert(!detour_bridge.feedback().active);
  return 0;
}
