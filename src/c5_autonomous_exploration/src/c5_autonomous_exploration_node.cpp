#include "c5_autonomous_exploration/ExplorationStatus.h"
#include "c5_autonomous_exploration/PlannerFeedback.h"
#include "c5_autonomous_exploration/PlannerRequest.h"
#include "c5_autonomous_exploration/TargetTrack.h"
#include "c5_autonomous_exploration/autonomous_exploration.hpp"
#include "c5_autonomous_exploration/dynamic_scene_filter.hpp"
#include "c5_autonomous_exploration/eskf.hpp"
#include "c5_autonomous_exploration/local_map.hpp"
#include "c5_autonomous_exploration/mission_executor.hpp"
#include "c5_autonomous_exploration/model_predictive_controller.hpp"
#include "c5_autonomous_exploration/runtime_support.hpp"
#include "c5_autonomous_exploration/semantic_target_processor.hpp"
#include "c5_autonomous_exploration/state_estimator.hpp"
#include "c5_autonomous_exploration/target_evidence.hpp"
#include "c5_autonomous_exploration/vio_frontend.hpp"

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <quadrotor_msgs/TakeoffLand.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64MultiArray.h>

#include <algorithm>
#include <cstdint>
#include <cmath>
#include <string>
#include <vector>

namespace c5_autonomous_exploration {

class C5AutonomousExplorationNode {
 public:
  C5AutonomousExplorationNode()
      : private_node_("~"),
        explorer_(readConfig()),
        cloud_max_range_(40.0),
        cloud_leaf_size_(0.4),
        cloud_is_world_frame_(false),
        planner_map_ready_(false),
        planner_map_clearance_(0.15),
        require_clear_path_(true),
        planner_clearance_(0.50),
        planner_waypoint_distance_(0.60),
        goal_min_distance_(1.0),
        frame_id_("map"),
        have_home_(false),
        last_stop_state_(false),
        last_commanded_goal_(),
        home_position_(),
        exploration_destination_(),
        exploration_destination_valid_(false),
        task_interrupted_(false),
        planner_failures_(0),
        planner_feedback_seen_(false),
        planner_goal_active_(false),
        planner_request_sequence_(0),
        orientation_x_(0.0),
        orientation_y_(0.0),
        orientation_z_(0.0),
        orientation_w_(1.0) {
    private_node_.param("cloud_max_range", cloud_max_range_, cloud_max_range_);
    private_node_.param("cloud_leaf_size", cloud_leaf_size_, cloud_leaf_size_);
    private_node_.param("cloud_is_world_frame", cloud_is_world_frame_, cloud_is_world_frame_);
    private_node_.param("planner_map_clearance", planner_map_clearance_, planner_map_clearance_);
    private_node_.param("require_clear_path", require_clear_path_, require_clear_path_);
    private_node_.param("planner_clearance", planner_clearance_, planner_clearance_);
    private_node_.param("planner_waypoint_distance", planner_waypoint_distance_, planner_waypoint_distance_);
    private_node_.param("goal_min_distance", goal_min_distance_, goal_min_distance_);
    private_node_.param<std::string>("frame_id", frame_id_, frame_id_);
    double home_x = 0.0;
    double home_y = 0.0;
    double home_z = 1.0;
    private_node_.param("home_x", home_x, home_x);
    private_node_.param("home_y", home_y, home_y);
    private_node_.param("home_z", home_z, home_z);
    explorer_.setHome(Vec3(home_x, home_y, home_z));
    home_position_ = Vec3(home_x, home_y, home_z);
    have_home_ = true;

    std::string odom_topic = "/quad_0/lidar_slam/odom";
    std::string cloud_topic = "/drone_0_pcl_render_node/cloud";
    std::string planner_map_topic = "/drone_0_diff_planner_node/grid_map/occupancy_inflate_full";
    std::string imu_topic = "/mavros/imu/data";
    std::string image_topic = "/c5/camera/image_raw";
    std::string vio_feature_topic = "/c5/vio_features";
    std::string position_command_topic = "/position_cmd";
    std::string model_path;
    std::string battery_topic = "/mavros/battery";
    std::string link_topic = "/c5/link_available";
    std::string estimator_topic = "/c5/estimator_quality";
    std::string detection_topic = "/c5/detections";
    std::string planner_feedback_topic = "/c5/planner_feedback";
    std::string task_interrupt_topic = "/c5/task_interrupt";
    std::string planner_request_topic = "/c5/planner_request";
    std::string stop_topic = "/mandatory_stop_to_planner";
    private_node_.param<std::string>("odom_topic", odom_topic, odom_topic);
    private_node_.param<std::string>("cloud_topic", cloud_topic, cloud_topic);
    private_node_.param<std::string>("planner_map_topic", planner_map_topic, planner_map_topic);
    private_node_.param<std::string>("imu_topic", imu_topic, imu_topic);
    private_node_.param<std::string>("image_topic", image_topic, image_topic);
    private_node_.param<std::string>("vio_feature_topic", vio_feature_topic, vio_feature_topic);
    private_node_.param<std::string>("position_command_topic", position_command_topic, position_command_topic);
    private_node_.param<std::string>("model_path", model_path, model_path);
    private_node_.param<std::string>("battery_topic", battery_topic, battery_topic);
    private_node_.param<std::string>("link_topic", link_topic, link_topic);
    private_node_.param<std::string>("estimator_topic", estimator_topic, estimator_topic);
    private_node_.param<std::string>("detection_topic", detection_topic, detection_topic);
    private_node_.param<std::string>("planner_feedback_topic", planner_feedback_topic, planner_feedback_topic);
    private_node_.param<std::string>("task_interrupt_topic", task_interrupt_topic, task_interrupt_topic);
    private_node_.param<std::string>("planner_request_topic", planner_request_topic, planner_request_topic);
    private_node_.param<std::string>("stop_topic", stop_topic, stop_topic);

    odometry_sub_ = node_.subscribe(odom_topic, 20, &C5AutonomousExplorationNode::odometryCallback, this);
    cloud_sub_ = node_.subscribe(cloud_topic, 2, &C5AutonomousExplorationNode::cloudCallback, this);
    planner_map_sub_ = node_.subscribe(planner_map_topic, 1, &C5AutonomousExplorationNode::plannerMapCallback, this);
    imu_sub_ = node_.subscribe(imu_topic, 50, &C5AutonomousExplorationNode::imuCallback, this);
    image_sub_ = node_.subscribe(image_topic, 2, &C5AutonomousExplorationNode::imageCallback, this);
    vio_feature_sub_ = node_.subscribe(vio_feature_topic, 20, &C5AutonomousExplorationNode::vioFeatureCallback, this);
    position_command_sub_ = node_.subscribe(position_command_topic, 20, &C5AutonomousExplorationNode::positionCommandCallback, this);
    battery_sub_ = node_.subscribe(battery_topic, 10, &C5AutonomousExplorationNode::batteryCallback, this);
    link_sub_ = node_.subscribe(link_topic, 10, &C5AutonomousExplorationNode::linkCallback, this);
    estimator_sub_ = node_.subscribe(estimator_topic, 10, &C5AutonomousExplorationNode::estimatorCallback, this);
    detection_sub_ = node_.subscribe(detection_topic, 10, &C5AutonomousExplorationNode::detectionCallback, this);
    planner_feedback_sub_ = node_.subscribe(planner_feedback_topic, 10, &C5AutonomousExplorationNode::plannerFeedbackCallback, this);
    task_interrupt_sub_ = node_.subscribe(task_interrupt_topic, 1, &C5AutonomousExplorationNode::taskInterruptCallback, this);
    planner_request_pub_ = node_.advertise<PlannerRequest>(planner_request_topic, 10);
    stop_pub_ = node_.advertise<std_msgs::Empty>(stop_topic, 1);
    land_pub_ = node_.advertise<quadrotor_msgs::TakeoffLand>("/px4ctrl/takeoff_land", 1);
    status_pub_ = private_node_.advertise<ExplorationStatus>("status", 10);
    target_pub_ = private_node_.advertise<TargetTrack>("target_tracks", 20);
    map_pub_ = private_node_.advertise<sensor_msgs::PointCloud2>("occupied_map", 2);
    frontier_pub_ = private_node_.advertise<geometry_msgs::PoseArray>("frontiers", 2);
    if (!model_path.empty()) semantic_processor_.loadModel(model_path);

    double planning_rate = 2.0;
    private_node_.param("planning_rate", planning_rate, planning_rate);
    ParameterValidator validator;
    validator.requirePositive("cloud_max_range", cloud_max_range_);
    validator.requirePositive("cloud_leaf_size", cloud_leaf_size_);
    validator.requirePositive("planning_rate", planning_rate);
    validator.requirePositive("planner_clearance", planner_clearance_);
    validator.requirePositive("planner_map_clearance", planner_map_clearance_);
    validator.requirePositive("planner_waypoint_distance", planner_waypoint_distance_);
    validator.requirePositive("goal_min_distance", goal_min_distance_);
    if (!validator.valid()) ROS_FATAL("Invalid c5_autonomous_exploration parameters");
    timer_ = node_.createTimer(ros::Duration(1.0 / std::max(0.1, planning_rate)),
                               &C5AutonomousExplorationNode::timerCallback, this);
  }

 private:
  ExplorerConfig readConfig() {
    ExplorerConfig config;
    private_node_.param("map_resolution", config.map_resolution, config.map_resolution);
    private_node_.param("occupied_log_odds", config.occupied_log_odds, config.occupied_log_odds);
    private_node_.param("free_log_odds", config.free_log_odds, config.free_log_odds);
    private_node_.param("map_log_odds_limit", config.map_log_odds_limit, config.map_log_odds_limit);
    private_node_.param("min_clearance", config.min_clearance, config.min_clearance);
    private_node_.param("frontier_gain_weight", config.frontier_gain_weight, config.frontier_gain_weight);
    private_node_.param("distance_weight", config.distance_weight, config.distance_weight);
    private_node_.param("home_distance_weight", config.home_distance_weight, config.home_distance_weight);
    private_node_.param("goal_min_distance", config.goal_min_distance, config.goal_min_distance);
    private_node_.param("min_goal_altitude", config.min_goal_altitude, config.min_goal_altitude);
    private_node_.param("max_goal_altitude", config.max_goal_altitude, config.max_goal_altitude);
    private_node_.param("nbs_beam_width", config.nbs_beam_width, config.nbs_beam_width);
    private_node_.param("nbs_search_depth", config.nbs_search_depth, config.nbs_search_depth);
    private_node_.param("rrag_connection_distance", config.rrag_connection_distance, config.rrag_connection_distance);
    private_node_.param("fls_search_radius", config.fls_search_radius, config.fls_search_radius);
    private_node_.param("return_battery_fraction", config.return_battery_fraction, config.return_battery_fraction);
    private_node_.param("land_battery_fraction", config.land_battery_fraction, config.land_battery_fraction);
    private_node_.param("link_loss_continue_seconds", config.link_loss_continue_seconds, config.link_loss_continue_seconds);
    private_node_.param("estimator_covariance_limit", config.estimator_covariance_limit, config.estimator_covariance_limit);
    private_node_.param("minimum_confidence", config.minimum_confidence, config.minimum_confidence);
    private_node_.param("target_association_radius", config.target_association_radius, config.target_association_radius);
    private_node_.param("target_confirmation_confidence", config.target_confirmation_confidence, config.target_confirmation_confidence);
    private_node_.param("max_planner_failures", config.max_planner_failures, config.max_planner_failures);
    return config;
  }

  void odometryCallback(const nav_msgs::OdometryConstPtr& message) {
    const Vec3 pose(message->pose.pose.position.x,
                    message->pose.pose.position.y,
                    message->pose.pose.position.z);
    const double measurement_variance = std::max(0.01, message->pose.covariance[0]);
    if (eskf_.state().stamp <= 0.0 && message->header.stamp.toSec() > 0.0) {
      NavigationState initial;
      initial.position = pose;
      initial.velocity = Vec3(message->twist.twist.linear.x, message->twist.twist.linear.y,
                              message->twist.twist.linear.z);
      initial.gravity = Vec3(0.0, 0.0, -9.81);
      eskf_.reset(initial, message->header.stamp.toSec());
    }
    eskf_.correctPosition(pose, measurement_variance);
    eskf_.correctVelocity(Vec3(message->twist.twist.linear.x, message->twist.twist.linear.y,
                               message->twist.twist.linear.z), measurement_variance);
    const NavigationState fused = eskf_.state();
    explorer_.updatePose(fused.position, message->header.stamp.toSec());
    state_estimator_.correctLio(pose, measurement_variance);
    rolling_map_.setCenter(fused.position);
    orientation_x_ = message->pose.pose.orientation.x;
    orientation_y_ = message->pose.pose.orientation.y;
    orientation_z_ = message->pose.pose.orientation.z;
    orientation_w_ = message->pose.pose.orientation.w;
    if (!have_home_) {
      explorer_.setHome(pose);
      have_home_ = true;
    }
  }

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& message) {
    const ExplorerStatus status = explorer_.status();
    if (!status.pose_valid) return;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*message, cloud);
    pcl::VoxelGrid<pcl::PointXYZ> filter;
    filter.setInputCloud(cloud.makeShared());
    filter.setLeafSize(cloud_leaf_size_, cloud_leaf_size_, cloud_leaf_size_);
    pcl::PointCloud<pcl::PointXYZ> reduced;
    filter.filter(reduced);
    std::vector<Vec3> endpoints;
    for (pcl::PointCloud<pcl::PointXYZ>::const_iterator it = reduced.begin(); it != reduced.end(); ++it) {
      if (!std::isfinite(it->x) || !std::isfinite(it->y) || !std::isfinite(it->z)) continue;
      const Vec3 source(it->x, it->y, it->z);
      const Vec3 endpoint = cloud_is_world_frame_ ? source : localToWorld(source, status.pose);
      const double dx = endpoint.x - status.pose.x;
      const double dy = endpoint.y - status.pose.y;
      const double dz = endpoint.z - status.pose.z;
      if (std::sqrt(dx * dx + dy * dy + dz * dz) <= cloud_max_range_) endpoints.push_back(endpoint);
    }
    dynamic_filter_.update(endpoints, message->header.stamp.toSec());
    const std::vector<Vec3> static_points = dynamic_filter_.staticPoints();
    latest_static_points_ = static_points;
    for (std::vector<Vec3>::const_iterator it = static_points.begin(); it != static_points.end(); ++it) {
      explorer_.integrateRay(status.pose, *it);
      rolling_map_.integrateRay(status.pose, *it, message->header.stamp.toSec());
    }
  }

  void plannerMapCallback(const sensor_msgs::PointCloud2ConstPtr& message) {
    if (message->header.frame_id != "world") {
      ROS_WARN_THROTTLE(5.0, "C5 is waiting for the planner occupancy map in the world frame");
      planner_map_ready_ = false;
      return;
    }
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*message, cloud);
    std::vector<Vec3> occupied_points;
    occupied_points.reserve(cloud.size());
    for (const pcl::PointXYZ& point : cloud.points) {
      if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
        occupied_points.push_back(Vec3(point.x, point.y, point.z));
      }
    }
    planner_map_obstacles_.swap(occupied_points);
    planner_map_ready_ = true;
  }

  void imuCallback(const sensor_msgs::ImuConstPtr& message) {
    const Vec3 acceleration(message->linear_acceleration.x, message->linear_acceleration.y,
                            message->linear_acceleration.z);
    const Vec3 angular_velocity(message->angular_velocity.x, message->angular_velocity.y,
                                message->angular_velocity.z);
    state_estimator_.propagate(acceleration, angular_velocity, message->header.stamp.toSec());
    const ImuSample sample{acceleration, angular_velocity, message->header.stamp.toSec()};
    eskf_.propagate(sample);
    vio_frontend_.integrateImu(sample);
  }

  void imageCallback(const sensor_msgs::ImageConstPtr& message) {
    RawImage image;
    image.width = static_cast<int>(message->width);
    image.height = static_cast<int>(message->height);
    image.channels = static_cast<int>(message->step / std::max(1U, message->width));
    image.stamp = message->header.stamp.toSec();
    image.data.assign(message->data.begin(), message->data.end());
    latest_image_ = semantic_processor_.preprocess(image);
  }

  void vioFeatureCallback(const std_msgs::Float64MultiArrayConstPtr& message) {
    if (message->data.size() < 4 || message->data.size() % 4 != 0) return;
    FeatureFrame frame;
    frame.stamp = ros::Time::now().toSec();
    for (std::size_t index = 0; index < message->data.size(); index += 4) {
      frame.features.push_back(FeatureObservation{static_cast<int>(message->data[index]), message->data[index + 1],
                                                  message->data[index + 2], message->data[index + 3]});
    }
    vio_frontend_.ingest(frame);
    const VioHealth health = vio_frontend_.health();
    explorer_.updateEstimator(eskf_.covarianceTrace(), health.reprojection_rms,
                              health.initialized ? health.confidence : 0.0, 1.0);
  }

  void positionCommandCallback(const quadrotor_msgs::PositionCommandConstPtr& message) {
    if (message->trajectory_flag == quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY) {
      planner_feedback_seen_ = true;
      planner_failures_ = 0;
      explorer_.clearPlannerFailures();
    } else if (!planner_feedback_seen_) {
      return;
    } else if (message->trajectory_flag == quadrotor_msgs::PositionCommand::TRAJECTROY_STATUS_ABORT ||
               message->trajectory_flag == quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_ILLEGAL_START ||
               message->trajectory_flag == quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_ILLEGAL_FINAL ||
               message->trajectory_flag == quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_IMPOSSIBLE) {
      ++planner_failures_;
      explorer_.reportPlannerFailure();
    }
  }

  void batteryCallback(const sensor_msgs::BatteryStateConstPtr& message) {
    if (std::isfinite(message->percentage) && message->percentage >= 0.0) explorer_.updateBattery(message->percentage);
  }

  void linkCallback(const std_msgs::BoolConstPtr& message) {
    explorer_.updateLink(message->data, ros::Time::now().toSec());
  }

  void estimatorCallback(const std_msgs::Float64MultiArrayConstPtr& message) {
    if (message->data.size() < 4) return;
    explorer_.updateEstimator(message->data[0], message->data[1], message->data[2], message->data[3]);
  }

  void detectionCallback(const geometry_msgs::PoseArrayConstPtr& message) {
    for (std::vector<geometry_msgs::Pose>::const_iterator it = message->poses.begin(); it != message->poses.end(); ++it) {
      const Vec3 position(it->position.x, it->position.y, it->position.z);
      const double confidence = std::max(0.0, std::min(1.0, it->orientation.w));
      target_evidence_.add("target", confidence, confidence, position, message->header.stamp.toSec());
      const TargetEvidence fused = target_evidence_.best();
      Detection detection;
      detection.position = fused.position;
      detection.confidence = fused.belief;
      detection.label = fused.label;
      detection.stamp = fused.stamp;
      explorer_.addDetection(detection);
    }
  }

  void plannerFeedbackCallback(const PlannerFeedbackConstPtr& message) {
    if (message->state == PlannerFeedback::ACCEPTED || message->state == PlannerFeedback::ACTIVE) {
      planner_feedback_seen_ = true;
      planner_goal_active_ = true;
      planner_failures_ = 0;
      explorer_.clearPlannerFailures();
      return;
    }
    if (message->state == PlannerFeedback::ARRIVED) {
      planner_feedback_seen_ = true;
      planner_goal_active_ = false;
      planner_failures_ = 0;
      explorer_.clearPlannerFailures();
      return;
    }
    if (message->state == PlannerFeedback::REJECTED_NO_MAP || message->state == PlannerFeedback::REJECTED_MAP ||
        message->state == PlannerFeedback::COMMAND_TIMEOUT || message->state == PlannerFeedback::GOAL_TIMEOUT ||
        message->state == PlannerFeedback::HEARTBEAT_TIMEOUT || message->state == PlannerFeedback::MAP_STALE ||
        message->state == PlannerFeedback::PLANNER_REJECTED || message->state == PlannerFeedback::STOPPED) {
      planner_feedback_seen_ = true;
      planner_goal_active_ = false;
      ++planner_failures_;
      explorer_.reportPlannerFailure();
    }
  }

  void taskInterruptCallback(const std_msgs::EmptyConstPtr&) {
    explorer_.reportTaskInterrupt();
    task_interrupted_ = true;
  }

  Vec3 localToWorld(const Vec3& point, const Vec3& origin) const {
    const double qx = orientation_x_;
    const double qy = orientation_y_;
    const double qz = orientation_z_;
    const double qw = orientation_w_;
    const double tx = 2.0 * (qy * point.z - qz * point.y);
    const double ty = 2.0 * (qz * point.x - qx * point.z);
    const double tz = 2.0 * (qx * point.y - qy * point.x);
    const double rx = point.x + qw * tx + (qy * tz - qz * ty);
    const double ry = point.y + qw * ty + (qz * tx - qx * tz);
    const double rz = point.z + qw * tz + (qx * ty - qy * tx);
    return Vec3(origin.x + rx, origin.y + ry, origin.z + rz);
  }

  void timerCallback(const ros::TimerEvent&) {
    const double now = ros::Time::now().toSec();
    rolling_map_.decay(now);
    if (!explorer_.status().pose_valid) return;
    ExplorationDecision decision = explorer_.planNextGoal();
    const ExplorerStatus status = explorer_.status();
    const MissionState mission_state = mission_executor_.update(MissionInput{
        true, status.link_available, status.fused_confidence >= 0.45, status.battery_fraction,
        planner_failures_, false, task_interrupted_, dynamic_filter_.dynamicPoints().empty() ? 0.0 : 1.0}, now);
    if (mission_state == MissionState::kReturn) {
      decision.has_goal = true;
      decision.goal = home_position_;
      decision.mode = SafetyState::kReturn;
    }
    publishDecision(decision, mission_state, now);
    const NavigationState navigation = eskf_.state();
    recorder_.append(FlightRecord{now, navigation.position, navigation.velocity, status.battery_fraction,
                                  eskf_.covarianceTrace(), static_cast<int>(mission_state)});
    publishStatus(decision, now);
    publishTargets(now);
    publishMap(now);
  }

  void publishDecision(const ExplorationDecision& decision, MissionState mission_state, double stamp) {
    const bool stop_required = decision.mode == SafetyState::kHold || mission_state == MissionState::kRelocalize || mission_state == MissionState::kLocked;
    if (stop_required && !last_stop_state_) {
      ROS_WARN_STREAM("C5 requests mandatory stop: mode=" << static_cast<int>(decision.mode)
                      << " mission=" << static_cast<int>(mission_state)
                      << " planner_failures=" << planner_failures_);
      std_msgs::Empty message;
      stop_pub_.publish(message);
    }
    last_stop_state_ = stop_required;
    if (decision.mode == SafetyState::kLand || mission_state == MissionState::kLand) {
      quadrotor_msgs::TakeoffLand message;
      message.takeoff_land_cmd = quadrotor_msgs::TakeoffLand::LAND;
      land_pub_.publish(message);
      return;
    }
    if (mission_state == MissionState::kTakeoff) {
      quadrotor_msgs::TakeoffLand message;
      message.takeoff_land_cmd = quadrotor_msgs::TakeoffLand::TAKEOFF;
      land_pub_.publish(message);
      return;
    }
    if (!decision.has_goal) return;
    if ((decision.mode == SafetyState::kExplore || decision.mode == SafetyState::kReturn) &&
        planner_goal_active_) return;
    const NavigationState estimate = eskf_.state();
    const std::vector<Vec3> bridge_map_obstacles = explorer_.occupiedVoxels();
    const auto clearOfPublishedMaps = [&](const Vec3& point) {
      if (!planner_map_ready_ || !pointHasClearance(planner_map_obstacles_, point, planner_map_clearance_)) {
        return false;
      }
      for (std::vector<Vec3>::const_iterator it = bridge_map_obstacles.begin();
           it != bridge_map_obstacles.end(); ++it) {
        const double dx = point.x - it->x;
        const double dy = point.y - it->y;
        const double dz = point.z - it->z;
        if (std::sqrt(dx * dx + dy * dy + dz * dz) < planner_clearance_) return false;
      }
      return true;
    };
    const std::vector<Vec3> frontiers = explorer_.frontiers();
    Vec3 destination = decision.goal;
    if (decision.mode == SafetyState::kExplore) {
      if (exploration_destination_valid_) {
        const double dx = exploration_destination_.x - estimate.position.x;
        const double dy = exploration_destination_.y - estimate.position.y;
        const double dz = exploration_destination_.z - estimate.position.z;
        const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (distance <= 0.35 || !clearOfPublishedMaps(exploration_destination_) ||
            rolling_map_.distanceToObstacle(exploration_destination_) < planner_clearance_) {
          exploration_destination_valid_ = false;
        }
      }
      if (!exploration_destination_valid_) {
        std::vector<Vec3> global_candidates;
        for (std::vector<Vec3>::const_iterator it = frontiers.begin(); it != frontiers.end(); ++it) {
          const double dx = it->x - estimate.position.x;
          const double dy = it->y - estimate.position.y;
          const double dz = it->z - estimate.position.z;
          if (dx * dx + dy * dy + dz * dz >= goal_min_distance_ * goal_min_distance_ &&
              clearOfPublishedMaps(*it) && rolling_map_.distanceToObstacle(*it) >= planner_clearance_) {
            global_candidates.push_back(*it);
          }
        }
        if (!rolling_map_.nearestClearPoint(global_candidates, decision.goal, planner_clearance_,
                                            &exploration_destination_)) {
          ROS_WARN_THROTTLE(2.0, "C5 has no distant frontier with sufficient clearance in both planner maps");
          return;
        }
        exploration_destination_valid_ = true;
      }
      destination = exploration_destination_;
    } else {
      exploration_destination_valid_ = false;
    }

    const double destination_dx = destination.x - estimate.position.x;
    const double destination_dy = destination.y - estimate.position.y;
    const double destination_dz = destination.z - estimate.position.z;
    const double destination_distance = std::sqrt(destination_dx * destination_dx +
                                                   destination_dy * destination_dy +
                                                   destination_dz * destination_dz);
    Vec3 safe_goal = destination;
    if (destination_distance > planner_waypoint_distance_ ||
        std::abs(destination.z - estimate.position.z) > 0.35 ||
        !clearOfPublishedMaps(destination) ||
        rolling_map_.distanceToObstacle(destination) < planner_clearance_) {
      bool found_waypoint = false;
      double best_progress = -std::numeric_limits<double>::infinity();
      double best_remaining_distance = std::numeric_limits<double>::infinity();
      const double direction_x = destination_dx / destination_distance;
      const double direction_y = destination_dy / destination_distance;
      const double direction_z = destination_dz / destination_distance;
      for (std::vector<Vec3>::const_iterator it = frontiers.begin(); it != frontiers.end(); ++it) {
        const double dx = it->x - estimate.position.x;
        const double dy = it->y - estimate.position.y;
        const double dz = it->z - estimate.position.z;
        const double distance_squared = dx * dx + dy * dy + dz * dz;
        if (distance_squared < 0.45 * 0.45 ||
            distance_squared > planner_waypoint_distance_ * planner_waypoint_distance_ ||
            std::abs(dz) > 0.35 || !clearOfPublishedMaps(*it) ||
            rolling_map_.distanceToObstacle(*it) < planner_clearance_) continue;
        const double progress = dx * direction_x + dy * direction_y + dz * direction_z;
        const double remaining_x = destination.x - it->x;
        const double remaining_y = destination.y - it->y;
        const double remaining_z = destination.z - it->z;
        const double remaining_distance = std::sqrt(remaining_x * remaining_x +
                                                    remaining_y * remaining_y +
                                                    remaining_z * remaining_z);
        if (!found_waypoint || progress > best_progress + 1e-6 ||
            (std::abs(progress - best_progress) <= 1e-6 &&
             remaining_distance < best_remaining_distance)) {
          safe_goal = *it;
          best_progress = progress;
          best_remaining_distance = remaining_distance;
          found_waypoint = true;
        }
      }
      if (!found_waypoint) {
        const double horizontal_step = 0.50;
        const double vertical_step = 0.25;
        for (int ix = -1; ix <= 1; ++ix) {
          for (int iy = -1; iy <= 1; ++iy) {
            for (int iz = -1; iz <= 1; ++iz) {
              if (ix == 0 && iy == 0 && iz == 0) continue;
              const Vec3 candidate(estimate.position.x + ix * horizontal_step,
                                   estimate.position.y + iy * horizontal_step,
                                   estimate.position.z + iz * vertical_step);
              const double dx = candidate.x - estimate.position.x;
              const double dy = candidate.y - estimate.position.y;
              const double dz = candidate.z - estimate.position.z;
              const double distance_squared = dx * dx + dy * dy + dz * dz;
              if (distance_squared < 0.45 * 0.45 ||
                  distance_squared > planner_waypoint_distance_ * planner_waypoint_distance_ ||
                  !clearOfPublishedMaps(candidate) ||
                  rolling_map_.distanceToObstacle(candidate) < planner_clearance_) continue;
              const double progress = dx * direction_x + dy * direction_y + dz * direction_z;
              const double remaining_x = destination.x - candidate.x;
              const double remaining_y = destination.y - candidate.y;
              const double remaining_z = destination.z - candidate.z;
              const double remaining_distance = std::sqrt(remaining_x * remaining_x +
                                                          remaining_y * remaining_y +
                                                          remaining_z * remaining_z);
              if (!found_waypoint || progress > best_progress + 1e-6 ||
                  (std::abs(progress - best_progress) <= 1e-6 &&
                   remaining_distance < best_remaining_distance)) {
                safe_goal = candidate;
                best_progress = progress;
                best_remaining_distance = remaining_distance;
                found_waypoint = true;
              }
            }
          }
        }
      }
      if (!found_waypoint) {
        ROS_WARN_THROTTLE(2.0, "C5 has no short, clear waypoint in either planner map; waiting for map update");
        return;
      }
    }
    ROS_INFO_STREAM_THROTTLE(2.0, "C5 navigation destination=(" << destination.x << ", " << destination.y << ", "
                             << destination.z << ") waypoint=(" << safe_goal.x << ", " << safe_goal.y << ", "
                             << safe_goal.z << ") remaining=" << destination_distance);
    if (require_clear_path_ && !rolling_map_.segmentClear(estimate.position, safe_goal, planner_clearance_)) {
      ++planner_failures_;
      explorer_.reportPlannerFailure();
      return;
    }
    if (rolling_map_.distanceToObstacle(safe_goal) < planner_clearance_ || !clearOfPublishedMaps(safe_goal)) {
      ROS_WARN_THROTTLE(2.0, "C5 is withholding a planner request because its endpoint lacks clearance in a planner map");
      return;
    }
    last_commanded_goal_ = safe_goal;
    PlannerRequest request;
    request.header.stamp = ros::Time(stamp);
    request.header.frame_id = frame_id_;
    request.sequence = ++planner_request_sequence_;
    request.mode = decision.mode == SafetyState::kReturn ? PlannerRequest::RETURN_HOME : PlannerRequest::EXPLORE;
    request.goal.header = request.header;
    request.goal.pose.position.x = last_commanded_goal_.x;
    request.goal.pose.position.y = last_commanded_goal_.y;
    request.goal.pose.position.z = last_commanded_goal_.z;
    request.goal.pose.orientation.w = 1.0;
    request.clearance = planner_clearance_;
    request.timeout = 12.0;
    planner_request_pub_.publish(request);
  }

  void publishStatus(const ExplorationDecision& decision, double stamp) {
    const ExplorerStatus source = explorer_.status();
    ExplorationStatus message;
    message.header.stamp = ros::Time(stamp);
    message.header.frame_id = frame_id_;
    message.state = static_cast<std::uint8_t>(decision.mode);
    message.event = static_cast<std::uint8_t>(decision.event);
    message.pose_valid = source.pose_valid;
    message.link_available = source.link_available;
    message.battery_fraction = source.battery_fraction;
    message.lio_confidence = source.lio_confidence;
    message.vio_confidence = source.vio_confidence;
    message.fused_confidence = source.fused_confidence;
    message.current_position.x = source.pose.x;
    message.current_position.y = source.pose.y;
    message.current_position.z = source.pose.z;
    message.commanded_goal.x = last_commanded_goal_.x;
    message.commanded_goal.y = last_commanded_goal_.y;
    message.commanded_goal.z = last_commanded_goal_.z;
    message.has_commanded_goal = decision.has_goal;
    status_pub_.publish(message);
  }

  void publishTargets(double stamp) {
    const std::vector<TargetEstimate> targets = explorer_.confirmedTargets();
    for (std::vector<TargetEstimate>::const_iterator it = targets.begin(); it != targets.end(); ++it) {
      c5_autonomous_exploration::TargetTrack message;
      message.header.stamp = ros::Time(stamp);
      message.header.frame_id = frame_id_;
      message.position.x = it->position.x;
      message.position.y = it->position.y;
      message.position.z = it->position.z;
      message.label = it->label;
      message.confidence = it->confidence;
      message.observations = it->observations;
      target_pub_.publish(message);
    }
  }

  void publishMap(double stamp) {
    const std::vector<Vec3> occupied = explorer_.occupiedVoxels();
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.reserve(occupied.size());
    for (std::vector<Vec3>::const_iterator it = occupied.begin(); it != occupied.end(); ++it) {
      cloud.push_back(pcl::PointXYZ(it->x, it->y, it->z));
    }
    sensor_msgs::PointCloud2 map_message;
    pcl::toROSMsg(cloud, map_message);
    map_message.header.stamp = ros::Time(stamp);
    map_message.header.frame_id = frame_id_;
    map_pub_.publish(map_message);

    geometry_msgs::PoseArray frontier_message;
    frontier_message.header.stamp = ros::Time(stamp);
    frontier_message.header.frame_id = frame_id_;
    const std::vector<Vec3> frontiers = explorer_.frontiers();
    for (std::vector<Vec3>::const_iterator it = frontiers.begin(); it != frontiers.end(); ++it) {
      geometry_msgs::Pose pose;
      pose.position.x = it->x;
      pose.position.y = it->y;
      pose.position.z = it->z;
      pose.orientation.w = 1.0;
      frontier_message.poses.push_back(pose);
    }
    frontier_pub_.publish(frontier_message);
  }

  ros::NodeHandle node_;
  ros::NodeHandle private_node_;
  AutonomousExplorer explorer_;
  ros::Subscriber odometry_sub_;
  ros::Subscriber cloud_sub_;
  ros::Subscriber planner_map_sub_;
  ros::Subscriber imu_sub_;
  ros::Subscriber image_sub_;
  ros::Subscriber vio_feature_sub_;
  ros::Subscriber position_command_sub_;
  ros::Subscriber battery_sub_;
  ros::Subscriber link_sub_;
  ros::Subscriber estimator_sub_;
  ros::Subscriber detection_sub_;
  ros::Subscriber planner_feedback_sub_;
  ros::Subscriber task_interrupt_sub_;
  ros::Publisher planner_request_pub_;
  ros::Publisher stop_pub_;
  ros::Publisher land_pub_;
  ros::Publisher status_pub_;
  ros::Publisher target_pub_;
  ros::Publisher map_pub_;
  ros::Publisher frontier_pub_;
  ros::Timer timer_;
  MultiModalStateEstimator state_estimator_;
  ErrorStateKalmanFilter eskf_;
  RollingOccupancyMap rolling_map_;
  MissionExecutor mission_executor_;
  SemanticTargetProcessor semantic_processor_;
  VioFrontend vio_frontend_;
  FlightRecorder recorder_;
  DynamicSceneFilter dynamic_filter_;
  ModelPredictiveController mpc_controller_;
  TargetEvidenceFusion target_evidence_;
  double cloud_max_range_;
  double cloud_leaf_size_;
  bool cloud_is_world_frame_;
  bool planner_map_ready_;
  double planner_map_clearance_;
  std::vector<Vec3> planner_map_obstacles_;
  bool require_clear_path_;
  double planner_clearance_;
  double planner_waypoint_distance_;
  double goal_min_distance_;
  std::string frame_id_;
  bool have_home_;
  bool last_stop_state_;
  Vec3 last_commanded_goal_;
  Vec3 home_position_;
  Vec3 exploration_destination_;
  bool exploration_destination_valid_;
  bool task_interrupted_;
  int planner_failures_;
  bool planner_feedback_seen_;
  bool planner_goal_active_;
  std::uint32_t planner_request_sequence_;
  ImageTensor latest_image_;
  std::vector<Vec3> latest_static_points_;
  double orientation_x_;
  double orientation_y_;
  double orientation_z_;
  double orientation_w_;
};

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "c5_autonomous_exploration");
  c5_autonomous_exploration::C5AutonomousExplorationNode node;
  ros::spin();
  return 0;
}
