#include "movensys_manipulator_moveit_config/time_indexed_sampler.hpp"

#include <algorithm>
#include <limits>
#include <vector>

#include <pluginlib/class_list_macros.hpp>

namespace movensys_manipulator_moveit_config
{

bool TimeIndexedSampler::initialize(
  const rclcpp::Node::SharedPtr & node,
  const moveit::core::RobotModelConstPtr & robot_model,
  const std::string & group_name)
{
  node_ = node;
  robot_model_ = robot_model;
  group_ = group_name;
  joint_group_ = robot_model_ ? robot_model_->getJointModelGroup(group_name) : nullptr;

  if (!node_ || !robot_model_ || !joint_group_) {
    RCLCPP_ERROR(
      rclcpp::get_logger("time_indexed_sampler"),
      "Failed to initialize TimeIndexedSampler for group '%s'", group_name.c_str());
    return false;
  }

  lookahead_time_ = node_->declare_parameter<double>("time_indexed_sampler.lookahead_time", 0.1);
  output_dt_ = node_->declare_parameter<double>("time_indexed_sampler.output_dt", 0.1);
  goal_reached_tolerance_ =
    node_->declare_parameter<double>("time_indexed_sampler.goal_reached_tolerance", 1.0e-3);
  first_point_blend_duration_ =
    node_->declare_parameter<double>("time_indexed_sampler.first_point_blend_duration", 0.5);
  start_point_trajectory_publish_count_ =
    node_->declare_parameter<int>("time_indexed_sampler.start_point_trajectory_publish_count", 20);
  debug_no_store_reference_trajectory_ =
    node_->declare_parameter<bool>("time_indexed_sampler.debug_no_store_reference_trajectory", false);

  RCLCPP_INFO(
    node_->get_logger(),
    "Initialized TimeIndexedSampler for group '%s' "
    "(lookahead_time=%.3f, output_dt=%.3f, first_point_blend_duration=%.3f, "
    "start_point_trajectory_publish_count=%d)",
    group_name.c_str(), lookahead_time_, output_dt_, first_point_blend_duration_,
    start_point_trajectory_publish_count_);
  return true;
}

moveit_msgs::action::LocalPlanner::Feedback TimeIndexedSampler::addTrajectorySegment(
  const robot_trajectory::RobotTrajectory & new_trajectory)
{
  if (new_trajectory.empty()) {
    has_reference_trajectory_ = false;
    reference_trajectory_.reset();
    reference_duration_ = 0.0;
    trajectory_time_offset_ = 0.0;
    start_point_trajectory_publishes_remaining_ = 0;
    RCLCPP_WARN(node_->get_logger(), "TimeIndexedSampler received an empty reference trajectory");
    return feedback_;
  }

  if (debug_no_store_reference_trajectory_) {
    const double duration =
      new_trajectory.getWayPointDurationFromStart(new_trajectory.getWayPointCount() - 1);
    has_reference_trajectory_ = false;
    reference_trajectory_.reset();
    reference_duration_ = 0.0;
    trajectory_time_offset_ = 0.0;
    start_point_trajectory_publishes_remaining_ = 0;
    RCLCPP_WARN(
      node_->get_logger(),
      "TimeIndexedSampler debug_no_store_reference_trajectory=true: received points=%zu, duration=%.3f, not storing",
      new_trajectory.getWayPointCount(), duration);
    return feedback_;
  }

  reference_trajectory_ =
    std::make_shared<robot_trajectory::RobotTrajectory>(new_trajectory, true);
  reference_duration_ =
    reference_trajectory_->getWayPointDurationFromStart(reference_trajectory_->getWayPointCount() - 1);
  trajectory_time_offset_ = 0.0;
  start_point_trajectory_publishes_remaining_ =
    first_point_blend_duration_ > std::numeric_limits<double>::epsilon() ?
    std::max(1, start_point_trajectory_publish_count_) : 0;
  if (start_point_trajectory_publishes_remaining_ == 0) {
    trajectory_start_time_ = node_->now();
  }
  has_reference_trajectory_ = true;

  RCLCPP_INFO(
    node_->get_logger(),
    "TimeIndexedSampler accepted reference trajectory: "
    "points=%zu, duration=%.3f, first_point_blend_duration=%.3f, "
    "start_point_trajectory_publish_count=%d",
    reference_trajectory_->getWayPointCount(),
    reference_duration_, first_point_blend_duration_,
    start_point_trajectory_publishes_remaining_);

  return feedback_;
}

moveit_msgs::action::LocalPlanner::Feedback TimeIndexedSampler::getLocalTrajectory(
  const moveit::core::RobotState & /* current_state */,
  robot_trajectory::RobotTrajectory & local_trajectory)
{
  local_trajectory.clear();
  RCLCPP_DEBUG(node_->get_logger(), "TimeIndexedSampler getLocalTrajectory entered");

  if (!has_reference_trajectory_ || !reference_trajectory_ || reference_trajectory_->empty()) {
    RCLCPP_DEBUG(node_->get_logger(), "TimeIndexedSampler is waiting for a reference trajectory");
    return feedback_;
  }

  if (start_point_trajectory_publishes_remaining_ > 0) {
    local_trajectory.addSuffixWayPoint(
      reference_trajectory_->getWayPoint(0), first_point_blend_duration_);
    --start_point_trajectory_publishes_remaining_;
    if (start_point_trajectory_publishes_remaining_ == 0) {
      trajectory_start_time_ = node_->now();
    }
    RCLCPP_INFO_THROTTLE(
      node_->get_logger(),
      *node_->get_clock(),
      1000,
      "TimeIndexedSampler sampled first reference waypoint as a single %.3f s trajectory",
      first_point_blend_duration_);
    return feedback_;
  }

  const double reference_time = getReferenceTime();
  const double target_time = std::clamp(reference_time + lookahead_time_, 0.0, reference_duration_);

  const std::size_t target_index = findUpperWaypoint(target_time);
  const auto target_state = interpolateWaypoint(target_time);
  RCLCPP_DEBUG(
    node_->get_logger(),
    "TimeIndexedSampler local sample: elapsed=%.3f, offset=%.3f, target_time=%.3f, target_index=%zu/%zu",
    getElapsedTime(), trajectory_time_offset_, target_time, target_index,
    reference_trajectory_->getWayPointCount());
  local_trajectory.addSuffixWayPoint(target_state, output_dt_);

  return feedback_;
}

double TimeIndexedSampler::getTrajectoryProgress(const moveit::core::RobotState & /* current_state */)
{
  RCLCPP_DEBUG(node_->get_logger(), "TimeIndexedSampler getTrajectoryProgress entered");
  if (!has_reference_trajectory_ || !reference_trajectory_ || reference_trajectory_->empty()) {
    return 0.0;
  }

  if (start_point_trajectory_publishes_remaining_ > 0) {
    return 0.0;
  }

  if (reference_duration_ <= std::numeric_limits<double>::epsilon()) {
    return 1.0;
  }
  return std::clamp(getReferenceTime() / reference_duration_, 0.0, 1.0);
}

bool TimeIndexedSampler::reset()
{
  has_reference_trajectory_ = false;
  reference_trajectory_.reset();
  reference_duration_ = 0.0;
  trajectory_time_offset_ = 0.0;
  start_point_trajectory_publishes_remaining_ = 0;
  return true;
}

moveit_msgs::action::LocalPlanner::Feedback TimeIndexedSampler::makeFeedback(
  const std::string & text) const
{
  moveit_msgs::action::LocalPlanner::Feedback feedback;
  feedback.feedback = text;
  return feedback;
}

double TimeIndexedSampler::getElapsedTime() const
{
  if (!node_) {
    return 0.0;
  }
  return std::max(0.0, (node_->now() - trajectory_start_time_).seconds());
}

double TimeIndexedSampler::getReferenceTime() const
{
  return std::clamp(getElapsedTime() + trajectory_time_offset_, 0.0, reference_duration_);
}

std::size_t TimeIndexedSampler::findUpperWaypoint(double target_time) const
{
  const std::size_t count = reference_trajectory_->getWayPointCount();
  for (std::size_t i = 0; i < count; ++i) {
    if (reference_trajectory_->getWayPointDurationFromStart(i) >= target_time) {
      return i;
    }
  }
  return count - 1;
}

moveit::core::RobotState TimeIndexedSampler::interpolateStates(
  const moveit::core::RobotState & lower_state,
  const moveit::core::RobotState & upper_state,
  double ratio) const
{
  ratio = std::clamp(ratio, 0.0, 1.0);

  std::vector<double> lower_positions;
  std::vector<double> upper_positions;
  std::vector<double> lower_velocities;
  std::vector<double> upper_velocities;
  lower_state.copyJointGroupPositions(joint_group_, lower_positions);
  upper_state.copyJointGroupPositions(joint_group_, upper_positions);
  lower_state.copyJointGroupVelocities(joint_group_, lower_velocities);
  upper_state.copyJointGroupVelocities(joint_group_, upper_velocities);

  if (lower_positions.size() != upper_positions.size()) {
    return upper_state;
  }

  std::vector<double> target_positions(lower_positions.size(), 0.0);
  for (std::size_t i = 0; i < target_positions.size(); ++i) {
    target_positions[i] = lower_positions[i] + ratio * (upper_positions[i] - lower_positions[i]);
  }

  moveit::core::RobotState target_state(lower_state);
  target_state.setJointGroupPositions(joint_group_, target_positions);

  if (lower_velocities.size() == upper_velocities.size()) {
    std::vector<double> target_velocities(lower_velocities.size(), 0.0);
    for (std::size_t i = 0; i < target_velocities.size(); ++i) {
      target_velocities[i] = lower_velocities[i] + ratio * (upper_velocities[i] - lower_velocities[i]);
    }
    target_state.setJointGroupVelocities(joint_group_, target_velocities);
  }

  target_state.update();
  return target_state;
}

moveit::core::RobotState TimeIndexedSampler::interpolateWaypoint(double target_time) const
{
  const std::size_t upper_index = findUpperWaypoint(target_time);
  if (upper_index == 0) {
    return reference_trajectory_->getWayPoint(0);
  }

  const std::size_t lower_index = upper_index - 1;
  const double lower_time = reference_trajectory_->getWayPointDurationFromStart(lower_index);
  const double upper_time = reference_trajectory_->getWayPointDurationFromStart(upper_index);
  const double span = upper_time - lower_time;
  const double ratio = span > std::numeric_limits<double>::epsilon() ?
    std::clamp((target_time - lower_time) / span, 0.0, 1.0) : 0.0;

  return interpolateStates(
    reference_trajectory_->getWayPoint(lower_index),
    reference_trajectory_->getWayPoint(upper_index),
    ratio);
}

}  // namespace movensys_manipulator_moveit_config

PLUGINLIB_EXPORT_CLASS(
  movensys_manipulator_moveit_config::TimeIndexedSampler,
  moveit::hybrid_planning::TrajectoryOperatorInterface)
