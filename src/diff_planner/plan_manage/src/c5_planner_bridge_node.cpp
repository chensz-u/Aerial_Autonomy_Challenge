#include <plan_manage/c5_planner_bridge_core.hpp>

#include <c5_autonomous_exploration/PlannerFeedback.h>
#include <c5_autonomous_exploration/PlannerRequest.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Empty.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace diff_planner {

class C5PlannerBridgeNode {
 public:
  C5PlannerBridgeNode() : node_(), private_node_("~"), bridge_(loadConfig()), have_position_(false), position_(), last_state_(BridgeFeedbackState::kRejectedNoMap) {
    std::string request_topic = "/c5/planner_request";
    std::string feedback_topic = "/c5/planner_feedback";
    std::string goal_topic = "/goal";
    std::string occupied_map_topic = "/c5_autonomous_exploration/occupied_map";
    std::string odom_topic = "/quad_0/lidar_slam/odom";
    std::string position_command_topic = "/position_cmd";
    std::string heartbeat_topic = "/drone_0_traj_server/heartbeat";
    std::string mandatory_stop_topic = "/mandatory_stop_to_planner";
    private_node_.param<std::string>("request_topic", request_topic, request_topic);
    private_node_.param<std::string>("feedback_topic", feedback_topic, feedback_topic);
    private_node_.param<std::string>("goal_topic", goal_topic, goal_topic);
    private_node_.param<std::string>("occupied_map_topic", occupied_map_topic, occupied_map_topic);
    private_node_.param<std::string>("odom_topic", odom_topic, odom_topic);
    private_node_.param<std::string>("position_command_topic", position_command_topic, position_command_topic);
    private_node_.param<std::string>("heartbeat_topic", heartbeat_topic, heartbeat_topic);
    private_node_.param<std::string>("mandatory_stop_topic", mandatory_stop_topic, mandatory_stop_topic);
    private_node_.param("map_leaf_size", map_leaf_size_, 0.2);
    request_sub_ = node_.subscribe(request_topic, 1, &C5PlannerBridgeNode::requestCallback, this);
    map_sub_ = node_.subscribe(occupied_map_topic, 1, &C5PlannerBridgeNode::mapCallback, this);
    odom_sub_ = node_.subscribe(odom_topic, 20, &C5PlannerBridgeNode::odomCallback, this);
    position_command_sub_ = node_.subscribe(position_command_topic, 20, &C5PlannerBridgeNode::positionCommandCallback, this);
    heartbeat_sub_ = node_.subscribe(heartbeat_topic, 10, &C5PlannerBridgeNode::heartbeatCallback, this);
    mandatory_stop_sub_ = node_.subscribe(mandatory_stop_topic, 2, &C5PlannerBridgeNode::mandatoryStopCallback, this);
    goal_pub_ = node_.advertise<geometry_msgs::PoseStamped>(goal_topic, 10);
    feedback_pub_ = node_.advertise<c5_autonomous_exploration::PlannerFeedback>(feedback_topic, 20);
    stop_pub_ = node_.advertise<std_msgs::Empty>(mandatory_stop_topic, 2);
    tick_timer_ = node_.createTimer(ros::Duration(0.05), &C5PlannerBridgeNode::tickCallback, this);
  }

 private:
  PlannerBridgeConfig loadConfig() {
    PlannerBridgeConfig config;
    private_node_.param("minimum_clearance", config.minimum_clearance, config.minimum_clearance);
    private_node_.param("command_timeout", config.command_timeout, config.command_timeout);
    private_node_.param("goal_timeout", config.goal_timeout, config.goal_timeout);
    private_node_.param("heartbeat_timeout", config.heartbeat_timeout, config.heartbeat_timeout);
    private_node_.param("map_timeout", config.map_timeout, config.map_timeout);
    private_node_.param("arrival_tolerance", config.arrival_tolerance, config.arrival_tolerance);
    private_node_.param("require_map", config.require_map, config.require_map);
    private_node_.param("require_clear_path", config.require_clear_path, config.require_clear_path);
    return config;
  }

  void requestCallback(const c5_autonomous_exploration::PlannerRequestConstPtr& message) {
    // Timeout starts when this bridge receives the request, not at its possibly queued source stamp.
    const double stamp = ros::Time::now().toSec();
    if (!have_position_) {
      publishDirect(message->sequence, BridgeFeedbackState::kRejectedStale,
                    BridgePoint(message->goal.pose.position.x, message->goal.pose.position.y, message->goal.pose.position.z), false, false, false,
                    std::numeric_limits<double>::infinity(), stamp);
      return;
    }
    PlannerRequestMode mode = PlannerRequestMode::kExplore;
    if (message->mode == c5_autonomous_exploration::PlannerRequest::RETURN_HOME) mode = PlannerRequestMode::kReturnHome;
    if (message->mode == c5_autonomous_exploration::PlannerRequest::HOLD) mode = PlannerRequestMode::kHold;
    const PlannerBridgeRequest request{message->sequence, mode,
                                       BridgePoint(message->goal.pose.position.x, message->goal.pose.position.y, message->goal.pose.position.z),
                                       message->clearance, stamp};
    const PlannerBridgeFeedback feedback = bridge_.submit(request, position_);
    publishFeedback(feedback);
    if (feedback.accepted) {
      geometry_msgs::PoseStamped goal = message->goal;
      goal.header.stamp = ros::Time(stamp);
      goal_pub_.publish(goal);
    }
  }

  void mapCallback(const sensor_msgs::PointCloud2ConstPtr& message) {
    const double received_at = ros::Time::now().toSec();
    pcl::PointCloud<pcl::PointXYZ>::Ptr raw(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*message, *raw);
    pcl::VoxelGrid<pcl::PointXYZ> filter;
    filter.setInputCloud(raw);
    filter.setLeafSize(map_leaf_size_, map_leaf_size_, map_leaf_size_);
    pcl::PointCloud<pcl::PointXYZ> reduced;
    filter.filter(reduced);
    std::vector<BridgePoint> occupied;
    occupied.reserve(reduced.points.size());
    for (const pcl::PointXYZ& point : reduced.points) {
      if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
        occupied.push_back(BridgePoint(point.x, point.y, point.z));
      }
    }
    const BridgeFeedbackState before = bridge_.feedback().state;
    // Freshness is based on local receipt time, not a possibly queued source timestamp.
    bridge_.updateMap(occupied, received_at);
    const PlannerBridgeFeedback& feedback = bridge_.feedback();
    if (feedback.state != before && !feedback.success) {
      publishFeedback(feedback);
      publishStop();
    }
  }

  void odomCallback(const nav_msgs::OdometryConstPtr& message) {
    const double stamp = message->header.stamp.isZero() ? ros::Time::now().toSec() : message->header.stamp.toSec();
    position_ = BridgePoint(message->pose.pose.position.x, message->pose.pose.position.y, message->pose.pose.position.z);
    have_position_ = true;
    const BridgeFeedbackState before = bridge_.feedback().state;
    bridge_.updatePosition(position_, stamp);
    const PlannerBridgeFeedback& feedback = bridge_.feedback();
    if (feedback.state != before) publishFeedback(feedback);
  }

  void positionCommandCallback(const quadrotor_msgs::PositionCommandConstPtr& message) {
    const double stamp = message->header.stamp.isZero() ? ros::Time::now().toSec() : message->header.stamp.toSec();
    const bool ready = message->trajectory_flag == quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
    const PlannerBridgeFeedback feedback = bridge_.markPlannerCommand(ready, stamp);
    if (ready || feedback.state == BridgeFeedbackState::kPlannerRejected) publishFeedback(feedback);
  }

  void heartbeatCallback(const std_msgs::EmptyConstPtr&) {
    bridge_.observePlannerHeartbeat(ros::Time::now().toSec());
  }

  void mandatoryStopCallback(const std_msgs::EmptyConstPtr&) {
    ROS_WARN("C5 planner bridge received mandatory-stop signal");
    const PlannerBridgeFeedback feedback = bridge_.stop(ros::Time::now().toSec());
    publishFeedback(feedback);
  }

  void tickCallback(const ros::TimerEvent& event) {
    const BridgeFeedbackState before = bridge_.feedback().state;
    const PlannerBridgeFeedback feedback = bridge_.tick(event.current_real.toSec());
    if (feedback.state != before) {
      publishFeedback(feedback);
      if (!feedback.success && feedback.state != BridgeFeedbackState::kArrived) publishStop();
    }
  }

  void publishStop() {
    ROS_WARN_STREAM("C5 planner bridge publishes mandatory stop: feedback_state="
                    << static_cast<int>(bridge_.feedback().state)
                    << " sequence=" << bridge_.feedback().sequence);
    std_msgs::Empty stop;
    stop_pub_.publish(stop);
  }

  void publishFeedback(const PlannerBridgeFeedback& feedback) {
    if (feedback.state == last_state_ && feedback.sequence == last_sequence_) return;
    publishDirect(feedback.sequence, feedback.state, feedback.goal, feedback.accepted, feedback.active, feedback.success,
                  feedback.distance_to_goal, feedback.stamp);
  }

  void publishDirect(std::uint32_t sequence, BridgeFeedbackState state, const BridgePoint& goal, bool accepted, bool active,
                     bool success, double distance_to_goal, double stamp) {
    c5_autonomous_exploration::PlannerFeedback message;
    message.header.stamp = ros::Time(stamp);
    message.sequence = sequence;
    message.state = static_cast<std::uint8_t>(state);
    message.accepted = accepted;
    message.active = active;
    message.success = success;
    message.active_goal.x = goal.x;
    message.active_goal.y = goal.y;
    message.active_goal.z = goal.z;
    message.distance_to_goal = distance_to_goal;
    message.detail = detailFor(state);
    feedback_pub_.publish(message);
    last_state_ = state;
    last_sequence_ = sequence;
  }

  static std::string detailFor(BridgeFeedbackState state) {
    switch (state) {
      case BridgeFeedbackState::kAccepted: return "accepted";
      case BridgeFeedbackState::kRejectedNoMap: return "map_unavailable";
      case BridgeFeedbackState::kRejectedMap: return "map_blocked";
      case BridgeFeedbackState::kRejectedStale: return "stale_request";
      case BridgeFeedbackState::kActive: return "trajectory_active";
      case BridgeFeedbackState::kArrived: return "goal_arrived";
      case BridgeFeedbackState::kCommandTimeout: return "command_timeout";
      case BridgeFeedbackState::kGoalTimeout: return "goal_timeout";
      case BridgeFeedbackState::kHeartbeatTimeout: return "heartbeat_timeout";
      case BridgeFeedbackState::kMapStale: return "map_stale";
      case BridgeFeedbackState::kPlannerRejected: return "planner_rejected";
      case BridgeFeedbackState::kStopped: return "mandatory_stop";
    }
    return "unknown";
  }

  ros::NodeHandle node_;
  ros::NodeHandle private_node_;
  C5PlannerBridgeCore bridge_;
  ros::Subscriber request_sub_;
  ros::Subscriber map_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber position_command_sub_;
  ros::Subscriber heartbeat_sub_;
  ros::Subscriber mandatory_stop_sub_;
  ros::Publisher goal_pub_;
  ros::Publisher feedback_pub_;
  ros::Publisher stop_pub_;
  ros::Timer tick_timer_;
  bool have_position_;
  BridgePoint position_;
  double map_leaf_size_;
  BridgeFeedbackState last_state_;
  std::uint32_t last_sequence_ = 0;
};

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "c5_planner_bridge");
  diff_planner::C5PlannerBridgeNode node;
  ros::spin();
  return 0;
}
