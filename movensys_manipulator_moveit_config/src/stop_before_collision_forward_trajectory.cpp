#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <pluginlib/class_list_macros.hpp>

#include <moveit/local_planner/feedback_types.hpp>
#include <moveit/local_planner/local_constraint_solver_interface.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_state/robot_state.hpp>

#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace movensys_manipulator_moveit_config
{

namespace
{
constexpr double kStartPointTrajectoryMinDuration = 0.25;
constexpr const char * kStartPointTrajectoryFrame = "start_point_trajectory";
constexpr const char * kReproducedTrajectoryFrame = "reproduced_trajectory";
}  // namespace

class StopBeforeCollisionForwardTrajectory
  : public moveit::hybrid_planning::LocalConstraintSolverInterface
{
public:
  bool initialize(
    const rclcpp::Node::SharedPtr & node,
    const planning_scene_monitor::PlanningSceneMonitorPtr & planning_scene_monitor,
    const std::string & group_name) override
  {
    node_ = node;
    planning_scene_monitor_ = planning_scene_monitor;
    group_name_ = group_name;

    if (!node_ || !planning_scene_monitor_) {
      RCLCPP_ERROR(
        rclcpp::get_logger("stop_before_collision_forward_trajectory"),
        "Failed to initialize StopBeforeCollisionForwardTrajectory");
      return false;
    }

    stop_before_collision_ = node_->declare_parameter<bool>("stop_before_collision", true);
    max_stuck_iterations_ =
      node_->declare_parameter<int>("stop_before_collision_forward_trajectory.max_stuck_iterations", 100);
    stuck_position_tolerance_ =
      node_->declare_parameter<double>("stop_before_collision_forward_trajectory.stuck_position_tolerance", 1.0e-6);
    const auto robot_model = planning_scene_monitor_->getRobotModel();
    joint_group_ = robot_model ? robot_model->getJointModelGroup(group_name_) : nullptr;
    if (!joint_group_) {
      RCLCPP_ERROR(
        node_->get_logger(), "Unable to find joint model group '%s'", group_name_.c_str());
      return false;
    }

    RCLCPP_INFO(
      node_->get_logger(),
      "Initialized StopBeforeCollisionForwardTrajectory for group '%s' (stop_before_collision=%s)",
      group_name_.c_str(), stop_before_collision_ ? "true" : "false");
    return true;
  }

  bool reset() override
  {
    path_invalidation_event_sent_ = false;
    hold_empty_until_new_trajectory_ = false;
    num_iterations_stuck_ = 0;
    prev_waypoint_target_.reset();
    return true;
  }

  moveit_msgs::action::LocalPlanner::Feedback solve(
    const robot_trajectory::RobotTrajectory & local_trajectory,
    const std::shared_ptr<const moveit_msgs::action::LocalPlanner::Goal> /* local_goal */,
    trajectory_msgs::msg::JointTrajectory & local_solution) override
  {
    feedback_.feedback.clear();
    local_solution = trajectory_msgs::msg::JointTrajectory();

    if (local_trajectory.empty()) {
      return feedback_;
    }

    const auto & target_state = local_trajectory.getWayPoint(local_trajectory.getWayPointCount() - 1);
    const double local_duration =
      local_trajectory.getWayPointDurationFromStart(local_trajectory.getWayPointCount() - 1);
    const bool is_start_point_trajectory = local_duration >= kStartPointTrajectoryMinDuration;
    local_solution.joint_names = joint_group_->getActiveJointModelNames();

    if (!is_start_point_trajectory && stop_before_collision_ && isStateColliding(target_state)) {
      hold_empty_until_new_trajectory_ = true;
      if (!path_invalidation_event_sent_) {
        feedback_.feedback = std::string(moveit::hybrid_planning::toString(
          moveit::hybrid_planning::COLLISION_AHEAD));
        path_invalidation_event_sent_ = true;
        RCLCPP_WARN(
          node_->get_logger(),
          "Collision detected ahead; publishing empty JointTrajectory on the local solution topic");
        return feedback_;
      }
      feedback_.feedback.clear();
      return feedback_;
    }

    if (hold_empty_until_new_trajectory_) {
      if (local_duration < kStartPointTrajectoryMinDuration) {
        RCLCPP_DEBUG(
          node_->get_logger(),
          "Holding empty JointTrajectory until a new start-point trajectory is available");
        return feedback_;
      }

      hold_empty_until_new_trajectory_ = false;
    }

    path_invalidation_event_sent_ = false;

    if (isStuck(target_state)) {
      feedback_.feedback = std::string(moveit::hybrid_planning::toString(
        moveit::hybrid_planning::LOCAL_PLANNER_STUCK));
      return feedback_;
    }

    moveit_msgs::msg::RobotTrajectory robot_trajectory_msg;
    local_trajectory.getRobotTrajectoryMsg(robot_trajectory_msg);
    local_solution = robot_trajectory_msg.joint_trajectory;
    local_solution.header.frame_id = is_start_point_trajectory ?
      kStartPointTrajectoryFrame : kReproducedTrajectoryFrame;

    if (local_solution.joint_names.empty()) {
      local_solution.joint_names = joint_group_->getActiveJointModelNames();
    }

    return feedback_;
  }

private:
  bool isStateColliding(const moveit::core::RobotState & state) const
  {
    planning_scene_monitor::LockedPlanningSceneRO planning_scene(planning_scene_monitor_);
    if (!planning_scene) {
      RCLCPP_WARN(node_->get_logger(), "Planning scene unavailable; skipping local collision check");
      return false;
    }

    return planning_scene->isStateColliding(state, group_name_, false);
  }

  bool isStuck(const moveit::core::RobotState & target_state)
  {
    if (!prev_waypoint_target_) {
      prev_waypoint_target_ = std::make_shared<moveit::core::RobotState>(target_state);
      num_iterations_stuck_ = 0;
      return false;
    }

    std::vector<double> previous_positions;
    std::vector<double> target_positions;
    prev_waypoint_target_->copyJointGroupPositions(joint_group_, previous_positions);
    target_state.copyJointGroupPositions(joint_group_, target_positions);

    double max_delta = 0.0;
    for (std::size_t i = 0; i < std::min(previous_positions.size(), target_positions.size()); ++i) {
      max_delta = std::max(max_delta, std::abs(target_positions[i] - previous_positions[i]));
    }

    prev_waypoint_target_ = std::make_shared<moveit::core::RobotState>(target_state);
    if (max_delta <= stuck_position_tolerance_) {
      ++num_iterations_stuck_;
    } else {
      num_iterations_stuck_ = 0;
    }

    return max_stuck_iterations_ > 0 &&
      num_iterations_stuck_ >= static_cast<std::size_t>(max_stuck_iterations_);
  }

  rclcpp::Node::SharedPtr node_;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  std::string group_name_;
  const moveit::core::JointModelGroup * joint_group_ = nullptr;
  moveit_msgs::action::LocalPlanner::Feedback feedback_;
  moveit::core::RobotStatePtr prev_waypoint_target_;
  std::size_t num_iterations_stuck_ = 0;
  int max_stuck_iterations_ = 100;
  double stuck_position_tolerance_ = 1.0e-6;
  bool stop_before_collision_ = true;
  bool path_invalidation_event_sent_ = false;
  bool hold_empty_until_new_trajectory_ = false;
};

}  // namespace movensys_manipulator_moveit_config

PLUGINLIB_EXPORT_CLASS(
  movensys_manipulator_moveit_config::StopBeforeCollisionForwardTrajectory,
  moveit::hybrid_planning::LocalConstraintSolverInterface)
