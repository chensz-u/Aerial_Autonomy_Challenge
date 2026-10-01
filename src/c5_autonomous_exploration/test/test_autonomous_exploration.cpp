#include "c5_autonomous_exploration/autonomous_exploration.hpp"
#include "c5_autonomous_exploration/dynamic_scene_filter.hpp"
#include "c5_autonomous_exploration/eskf.hpp"
#include "c5_autonomous_exploration/local_map.hpp"
#include "c5_autonomous_exploration/loop_closure.hpp"
#include "c5_autonomous_exploration/mission_executor.hpp"
#include "c5_autonomous_exploration/model_predictive_controller.hpp"
#include "c5_autonomous_exploration/semantic_target_processor.hpp"
#include "c5_autonomous_exploration/state_estimator.hpp"
#include "c5_autonomous_exploration/target_evidence.hpp"
#include "c5_autonomous_exploration/vio_frontend.hpp"

#include <cassert>
#include <cmath>
#include <vector>

using c5_autonomous_exploration::AutonomousExplorer;
using c5_autonomous_exploration::CellState;
using c5_autonomous_exploration::Detection;
using c5_autonomous_exploration::ExplorerConfig;
using c5_autonomous_exploration::SafetyEvent;
using c5_autonomous_exploration::SafetyState;
using c5_autonomous_exploration::Vec3;

int main() {
  c5_autonomous_exploration::ErrorStateKalmanFilter recovering_eskf;
  c5_autonomous_exploration::NavigationState recovering_navigation;
  recovering_navigation.gravity = Vec3(0.0, 0.0, 0.0);
  recovering_eskf.reset(recovering_navigation, 0.0);
  assert(!recovering_eskf.correctPosition(Vec3(2.0, 0.0, 0.0), 0.01));
  assert(!recovering_eskf.correctPosition(Vec3(2.02, 0.0, 0.0), 0.01));
  assert(recovering_eskf.correctPosition(Vec3(2.01, 0.0, 0.0), 0.01));
  assert(std::abs(recovering_eskf.state().position.x - 2.01) < 1e-9);

  c5_autonomous_exploration::ErrorStateKalmanFilter velocity_recovery_eskf;
  c5_autonomous_exploration::NavigationState velocity_recovery_state;
  velocity_recovery_state.gravity = Vec3(0.0, 0.0, 0.0);
  velocity_recovery_eskf.reset(velocity_recovery_state, 0.0);
  c5_autonomous_exploration::ImuSample drift_sample;
  drift_sample.acceleration = Vec3(10.0, 0.0, 0.0);
  drift_sample.angular_velocity = Vec3(0.0, 0.0, 0.0);
  drift_sample.stamp = 1.0;
  velocity_recovery_eskf.propagate(drift_sample);
  assert(!velocity_recovery_eskf.correctVelocity(Vec3(0.0, 0.0, 0.0), 0.01));
  assert(!velocity_recovery_eskf.correctVelocity(Vec3(0.01, 0.0, 0.0), 0.01));
  assert(velocity_recovery_eskf.correctVelocity(Vec3(0.0, 0.0, 0.0), 0.01));
  assert(std::abs(velocity_recovery_eskf.state().velocity.x) < 1e-9);
  ExplorerConfig config;
  config.map_resolution = 1.0;
  config.min_clearance = 0.8;
  config.return_battery_fraction = 0.25;
  config.link_loss_continue_seconds = 30.0;
  config.minimum_confidence = 0.45;
  config.max_planner_failures = 1;

  ExplorerConfig altitude_config = config;
  altitude_config.min_goal_altitude = 1.0;
  altitude_config.max_goal_altitude = 1.75;
  AutonomousExplorer altitude_explorer(altitude_config);
  altitude_explorer.setHome(Vec3(0.0, 0.0, 1.0));
  altitude_explorer.updatePose(Vec3(0.0, 0.0, 1.0), 0.0);
  altitude_explorer.updateBattery(0.8);
  altitude_explorer.updateLink(true, 0.0);
  altitude_explorer.updateEstimator(0.0, 0.0, 1.0, 1.0);
  altitude_explorer.setVoxel(Vec3(0.0, 0.0, 1.0), CellState::kFree);
  altitude_explorer.setVoxel(Vec3(1.0, 0.0, 1.0), CellState::kFree);
  altitude_explorer.setVoxel(Vec3(2.0, 0.0, 1.0), CellState::kFree);
  altitude_explorer.setVoxel(Vec3(0.0, 0.0, 2.0), CellState::kFree);
  const auto altitude_decision = altitude_explorer.planNextGoal();
  assert(altitude_decision.has_goal);
  assert(altitude_decision.goal.z >= 1.0 && altitude_decision.goal.z <= 1.75);

  AutonomousExplorer explorer(config);
  explorer.setHome(Vec3(0.0, 0.0, 1.0));
  explorer.updatePose(Vec3(0.0, 0.0, 1.0), 0.0);
  explorer.updateBattery(0.8);
  explorer.updateLink(true, 0.0);
  explorer.updateEstimator(0.08, 0.30, 1.0, 1.0);

  explorer.integrateRay(Vec3(0.0, 0.0, 1.0), Vec3(3.0, 0.0, 1.0));
  assert(!explorer.occupiedVoxels().empty());

  explorer.setVoxel(Vec3(0.0, 0.0, 1.0), CellState::kFree);
  explorer.setVoxel(Vec3(1.0, 0.0, 1.0), CellState::kFree);
  explorer.setVoxel(Vec3(2.0, 0.0, 1.0), CellState::kFree);
  explorer.setVoxel(Vec3(1.0, 1.0, 1.0), CellState::kOccupied);

  const std::vector<Vec3> frontiers = explorer.frontiers();
  assert(!frontiers.empty());

  const auto decision = explorer.planNextGoal();
  assert(decision.has_goal);
  assert(decision.goal.x >= 1.0);
  assert(decision.mode == SafetyState::kExplore);

  explorer.updateLink(false, 10.0);
  assert(explorer.evaluateSafety(15.0) == SafetyState::kExplore);
  assert(explorer.evaluateSafety(41.0) == SafetyState::kReturn);

  explorer.updateLink(true, 42.0);
  explorer.updateBattery(0.20);
  assert(explorer.evaluateSafety(43.0) == SafetyState::kReturn);

  explorer.updateBattery(0.8);
  explorer.updateEstimator(1.1, 1.2, 0.9, 0.1);
  assert(explorer.evaluateSafety(44.0) == SafetyState::kHold);

  explorer.reportPlannerFailure();
  assert(explorer.evaluateSafety(45.0) == SafetyState::kReturn);

  Detection first;
  first.position = Vec3(2.0, 0.0, 1.0);
  first.confidence = 0.9;
  first.label = "person";
  first.stamp = 1.0;
  Detection second = first;
  second.position = Vec3(2.2, 0.0, 1.0);
  second.confidence = 0.8;
  second.stamp = 1.2;
  explorer.addDetection(first);
  explorer.addDetection(second);
  const auto tracks = explorer.confirmedTargets();
  assert(tracks.size() == 1);
  assert(tracks.front().confidence > 0.8);

  const auto status = explorer.status();
  assert(status.pose_valid);
  assert(std::abs(status.pose.x) < 1e-9);

  c5_autonomous_exploration::MultiModalStateEstimator estimator;
  estimator.reset(Vec3(0.0, 0.0, 0.0), 0.0);
  estimator.propagate(Vec3(1.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0), 1.0);
  estimator.correctLio(Vec3(0.5, 0.0, 0.0), 0.1);
  assert(estimator.estimate().position.x > 0.0);

  c5_autonomous_exploration::MultiModalStateEstimator recovering_estimator;
  recovering_estimator.reset(Vec3(0.0, 0.0, 0.0), 0.0);
  assert(!recovering_estimator.correctLio(Vec3(5.0, 0.0, 0.0), 0.05));
  assert(!recovering_estimator.correctLio(Vec3(5.02, 0.0, 0.0), 0.05));
  assert(recovering_estimator.correctLio(Vec3(5.01, 0.0, 0.0), 0.05));
  assert(std::abs(recovering_estimator.estimate().position.x - 5.01) < 1e-9);

  c5_autonomous_exploration::DynamicSceneFilter dynamic_filter;
  dynamic_filter.update(std::vector<Vec3>{Vec3(1.0, 0.0, 0.0)}, 0.0);
  dynamic_filter.update(std::vector<Vec3>{Vec3(1.3, 0.0, 0.0)}, 0.1);
  assert(!dynamic_filter.dynamicPoints().empty());

  c5_autonomous_exploration::ModelPredictiveController controller;
  c5_autonomous_exploration::MpcState mpc_state;
  mpc_state.position = Vec3(0.0, 0.0, 1.0);
  mpc_state.velocity = Vec3(0.0, 0.0, 0.0);
  const auto command = controller.solve(mpc_state, Vec3(3.0, 0.0, 1.0), std::vector<Vec3>());
  assert(command.acceleration.x > 0.0);

  c5_autonomous_exploration::TargetEvidenceFusion evidence;
  evidence.add("person", 0.8, 0.9, Vec3(1.0, 0.0, 1.0), 0.0);
  evidence.add("person", 0.7, 0.8, Vec3(1.1, 0.0, 1.0), 0.1);
  assert(evidence.best().label == "person");
  assert(evidence.best().belief > 0.8);

  c5_autonomous_exploration::ErrorStateKalmanFilter eskf;
  c5_autonomous_exploration::NavigationState navigation;
  navigation.position = Vec3(0.0, 0.0, 0.0);
  navigation.velocity = Vec3(0.0, 0.0, 0.0);
  navigation.gravity = Vec3(0.0, 0.0, 0.0);
  eskf.reset(navigation, 0.0);
  c5_autonomous_exploration::ImuSample imu;
  imu.acceleration = Vec3(1.0, 0.0, 0.0);
  imu.angular_velocity = Vec3(0.0, 0.0, 0.0);
  imu.stamp = 1.0;
  eskf.propagate(imu);
  assert(eskf.state().position.x > 0.4);
  assert(eskf.correctPosition(Vec3(0.6, 0.0, 0.0), 0.05));
  assert(!eskf.correctPosition(Vec3(100.0, 0.0, 0.0), 0.05));


  c5_autonomous_exploration::VioFrontend vio;
  c5_autonomous_exploration::FeatureFrame feature_frame;
  feature_frame.stamp = 0.0;
  feature_frame.features.push_back(c5_autonomous_exploration::FeatureObservation{1, 0.1, 0.1, 1.0});
  vio.ingest(feature_frame);
  feature_frame.stamp = 0.2;
  feature_frame.features[0].u = 0.3;
  vio.ingest(feature_frame);
  assert(vio.health().keyframes > 0);
  assert(vio.initializeScale(std::vector<Vec3>{Vec3(0.0, 0.0, 0.0), Vec3(1.0, 0.0, 0.0)},
                             std::vector<Vec3>{Vec3(0.0, 0.0, 0.0), Vec3(2.0, 0.0, 0.0)}));
  assert(vio.health().scale > 1.9);

  c5_autonomous_exploration::RollingOccupancyMap rolling_map;
  rolling_map.setCenter(Vec3(0.0, 0.0, 0.0));
  rolling_map.integrateRay(Vec3(0.0, 0.0, 0.0), Vec3(2.0, 0.0, 0.0), 0.0);
  assert(rolling_map.occupied(Vec3(2.0, 0.0, 0.0)));
  assert(rolling_map.distanceToObstacle(Vec3(1.0, 0.0, 0.0)) >= 0.0);
  assert(!rolling_map.segmentClear(Vec3(0.0, 0.0, 0.0), Vec3(2.0, 0.0, 0.0), 0.3));
  Vec3 clear_frontier;
  const std::vector<Vec3> frontier_candidates{Vec3(2.0, 0.0, 0.0), Vec3(1.75, 0.0, 0.0), Vec3(2.5, 0.0, 0.0)};
  assert(rolling_map.nearestClearPoint(frontier_candidates, Vec3(1.9, 0.0, 0.0), 0.3, &clear_frontier));
  assert(std::abs(clear_frontier.x - 1.75) < 1e-9);
  assert(!rolling_map.nearestClearPoint(std::vector<Vec3>{Vec3(2.0, 0.0, 0.0)},
                                       Vec3(2.0, 0.0, 0.0), 0.3, &clear_frontier));

  const std::vector<Vec3> planner_inflated_occupied{Vec3(3.65, -0.85, 1.15)};
  assert(!c5_autonomous_exploration::pointHasClearance(
      planner_inflated_occupied, Vec3(3.625, -0.875, 1.125), 0.10));
  assert(c5_autonomous_exploration::pointHasClearance(
      planner_inflated_occupied, Vec3(3.125, -1.375, 0.875), 0.10));

  rolling_map.decay(20.0);
  assert(!rolling_map.occupied(Vec3(2.0, 0.0, 0.0)));

  c5_autonomous_exploration::LoopClosureDatabase loop_database;
  loop_database.add(c5_autonomous_exploration::PlaceKeyframe{0, 0.0, Vec3(0.0, 0.0, 0.0), {1.0, 0.0, 0.0}, {"door"}});
  const auto loop = loop_database.query(c5_autonomous_exploration::PlaceKeyframe{1, 10.0, Vec3(3.0, 0.0, 0.0), {0.9, 0.1, 0.0}, {"door"}});
  assert(loop.accepted);
  assert(!loop_database.query(c5_autonomous_exploration::PlaceKeyframe{2, 0.1, Vec3(0.1, 0.0, 0.0), {0.9, 0.1, 0.0}, {"door"}}).accepted);

  c5_autonomous_exploration::MissionExecutor mission;
  mission.reset(0.0);
  mission.update(c5_autonomous_exploration::MissionInput{true, true, true, 0.8, 0, false, false, 0.0}, 0.0);
  assert(mission.state() == c5_autonomous_exploration::MissionState::kExplore);
  mission.update(c5_autonomous_exploration::MissionInput{true, true, true, 0.8, 3, false, false, 0.0}, 1.0);
  assert(mission.state() == c5_autonomous_exploration::MissionState::kRelocalize);
  mission.update(c5_autonomous_exploration::MissionInput{true, false, true, 0.8, 0, false, false, 0.0}, 1.0);
  assert(mission.state() == c5_autonomous_exploration::MissionState::kLinkContinue);
  mission.update(c5_autonomous_exploration::MissionInput{true, false, true, 0.8, 0, false, false, 0.0}, 32.0);
  assert(mission.state() == c5_autonomous_exploration::MissionState::kReturn);

  c5_autonomous_exploration::SemanticTargetProcessor processor;
  c5_autonomous_exploration::RawImage image;
  image.width = 2;
  image.height = 2;
  image.channels = 3;
  image.data = std::vector<unsigned char>(12, 255);
  const auto tensor = processor.preprocess(image);
  assert(tensor.width > 0);
  assert(tensor.values.size() == static_cast<std::size_t>(tensor.width * tensor.height * 3));
  assert(processor.project(c5_autonomous_exploration::Detection2d{"person", 0.9, 0.0, 0.0, 1.0, 1.0},
                           c5_autonomous_exploration::CameraModel{100.0, 100.0, 1.0, 1.0},
                           Vec3(0.0, 0.0, 0.0), std::vector<Vec3>{Vec3(0.0, 0.0, 3.0)}).valid);
  return 0;
}
