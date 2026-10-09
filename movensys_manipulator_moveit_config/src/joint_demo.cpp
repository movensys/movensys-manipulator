// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#include <array>
#include <chrono>
#include <cmath>
#include <exception>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#if __has_include(<moveit/move_group_interface/move_group_interface.hpp>)
#include <moveit/move_group_interface/move_group_interface.hpp>
#else
#include <moveit/move_group_interface/move_group_interface.h>
#endif

namespace
{
using namespace std::chrono_literals;
using MoveGroup = moveit::planning_interface::MoveGroupInterface;

constexpr std::array<double, 6> POINT_A_DEG{60.0, -10.0, -60.0, 30.0, 90.0, 0.0};
constexpr std::array<double, 6> POINT_B_DEG{0.0, 0.0, -90.0, 0.0, 90.0, 0.0};
constexpr double DEG_TO_RAD = 3.14159265358979323846 / 180.0;

template<typename T>
T parameter(const rclcpp::Node::SharedPtr & node, const std::string & name, T fallback)
{
  if (!node->has_parameter(name)) {
    node->declare_parameter<T>(name, fallback);
  }
  return node->get_parameter(name).get_value<T>();
}

std::map<std::string, double> jointTarget(const std::array<double, 6> & degrees)
{
  std::map<std::string, double> target;
  for (std::size_t i = 0; i < degrees.size(); ++i) {
    // Axis1..Axis6 correspond to joint1..joint6 in all supported robot models.
    target.emplace("joint" + std::to_string(i + 1), degrees[i] * DEG_TO_RAD);
  }
  return target;
}

bool moveTo(
  MoveGroup & group, const rclcpp::Logger & logger,
  const std::array<double, 6> & degrees, const char * point)
{
  if (!rclcpp::ok()) {
    return false;
  }
  group.setStartStateToCurrentState();
  if (!group.setJointValueTarget(jointTarget(degrees))) {
    RCLCPP_ERROR(logger, "Point %s is outside the robot's joint limits", point);
    return false;
  }
  RCLCPP_INFO(logger, "Moving to point %s", point);
  // move() blocks until planning and trajectory execution have finished.
  const auto result = group.move();
  if (result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(logger, "Move to point %s failed (code %d); ending demo", point, result.val);
    return false;
  }
  RCLCPP_INFO(logger, "Reached point %s", point);
  return true;
}

bool waitAfterMovement(std::chrono::milliseconds duration)
{
  // Use elapsed wall time, including when use_sim_time is enabled. Small sleeps
  // let Ctrl+C end a dwell promptly without sending the next movement.
  const auto deadline = std::chrono::steady_clock::now() + duration;
  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(10ms);
  }
  return rclcpp::ok();
}

bool runDemo(MoveGroup & group, const rclcpp::Logger & logger)
{
  RCLCPP_INFO(logger, "Returning to point B (demo home position)");
  if (!moveTo(group, logger, POINT_B_DEG, "B") || !waitAfterMovement(500ms)) {
    return !rclcpp::ok();
  }
  while (rclcpp::ok()) {
    if (!moveTo(group, logger, POINT_A_DEG, "A") || !waitAfterMovement(1000ms) ||
      !moveTo(group, logger, POINT_B_DEG, "B") || !waitAfterMovement(1000ms))
    {
      return !rclcpp::ok();
    }
  }
  return true;
}
}  // namespace

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rclcpp::Node>("joint_demo", options);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() {executor.spin();});

  int exit_code = 0;
  try {
    const auto velocity = parameter<double>(node, "vel_scale", 0.3);
    const auto acceleration = parameter<double>(node, "acc_scale", 0.3);
    const auto planning_time = parameter<double>(node, "planning_time", 5.0);
    if (!std::isfinite(velocity) || velocity <= 0.0 || velocity > 1.0 ||
      !std::isfinite(acceleration) || acceleration <= 0.0 || acceleration > 1.0 ||
      !std::isfinite(planning_time) || planning_time <= 0.0)
    {
      throw std::invalid_argument(
              "vel_scale/acc_scale must be in (0, 1]; planning_time must be positive");
    }
    MoveGroup group(node, "movensys_manipulator_arm");
    group.setMaxVelocityScalingFactor(velocity);
    group.setMaxAccelerationScalingFactor(acceleration);
    group.setPlanningTime(planning_time);
    exit_code = runDemo(group, node->get_logger()) ? 0 : 1;
  } catch (const std::exception & error) {
    if (rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "Demo failed: %s", error.what());
      exit_code = 1;
    }
  }

  executor.cancel();
  spin_thread.join();
  rclcpp::shutdown();
  return exit_code;
}
