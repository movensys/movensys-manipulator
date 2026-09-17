#pragma once

#include <cstddef>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <moveit/local_planner/trajectory_operator_interface.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit/robot_trajectory/robot_trajectory.hpp>

namespace movensys_manipulator_moveit_config
{

class TimeIndexedSampler : public moveit::hybrid_planning::TrajectoryOperatorInterface
{
public:
  bool initialize(
    const rclcpp::Node::SharedPtr & node,
    const moveit::core::RobotModelConstPtr & robot_model,
    const std::string & group_name) override;

  moveit_msgs::action::LocalPlanner::Feedback addTrajectorySegment(
    const robot_trajectory::RobotTrajectory & new_trajectory) override;

  moveit_msgs::action::LocalPlanner::Feedback getLocalTrajectory(
    const moveit::core::RobotState & current_state,
    robot_trajectory::RobotTrajectory & local_trajectory) override;

  double getTrajectoryProgress(const moveit::core::RobotState & current_state) override;

  bool reset() override;

private:
  moveit_msgs::action::LocalPlanner::Feedback makeFeedback(const std::string & text) const;
  double getElapsedTime() const;
  double getReferenceTime() const;
  std::size_t findUpperWaypoint(double target_time) const;
  moveit::core::RobotState interpolateStates(
    const moveit::core::RobotState & lower_state,
    const moveit::core::RobotState & upper_state,
    double ratio) const;
  moveit::core::RobotState interpolateWaypoint(double target_time) const;

  rclcpp::Node::SharedPtr node_;
  moveit::core::RobotModelConstPtr robot_model_;
  const moveit::core::JointModelGroup * joint_group_ = nullptr;
  rclcpp::Time trajectory_start_time_;

  double lookahead_time_ = 0.1;
  double output_dt_ = 0.1;
  double goal_reached_tolerance_ = 1.0e-3;
  double first_point_blend_duration_ = 0.5;
  double reference_duration_ = 0.0;
  double trajectory_time_offset_ = 0.0;
  int start_point_trajectory_publish_count_ = 1;
  int start_point_trajectory_publishes_remaining_ = 0;
  bool debug_no_store_reference_trajectory_ = false;
  bool has_reference_trajectory_ = false;
  moveit_msgs::action::LocalPlanner::Feedback feedback_;
};

}  // namespace movensys_manipulator_moveit_config
