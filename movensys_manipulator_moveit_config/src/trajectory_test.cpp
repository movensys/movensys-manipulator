// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#include <cmath>
#include <map>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "moveit2_client.hpp"

bool runTrajectory(const rclcpp::Node::SharedPtr& node, moveit2_client::MoveIt2Client& client);

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("trajectory_test");

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spin_thread([&executor]() { executor.spin(); });

    // MoveIt client config
    node->declare_parameter("base_name",         "base");
    node->declare_parameter("eef_name",          "eef");
    node->declare_parameter("vel_scale",         0.0);
    node->declare_parameter("acc_scale",         0.0);
    node->declare_parameter("delay_exec",        0.0);
    node->declare_parameter("delay_gripper",     0.0);
    node->declare_parameter("max_step",          0.0);
    node->declare_parameter("planning_time",     0.0);
    node->declare_parameter("timeout",           0.0);
    node->declare_parameter("planning_attempts", 0);
    node->declare_parameter("replan",            true);
    node->declare_parameter("replan_attempts",   0);
    node->declare_parameter("pipeline_id",       "");
    node->declare_parameter("planner_id",        "");

    // Trajectory waypoints
    node->declare_parameter("joint_names",
        std::vector<std::string>{"j1", "j2", "j3", "j4", "j5", "j6"});

    node->declare_parameter("joint_initial_0",        std::vector<double>(1, 0.0));

    node->declare_parameter("cartesian_poses_0",     std::vector<double>(1, 0.0));
    node->declare_parameter("cartesian_poses_1",     std::vector<double>(1, 0.0));

    node->declare_parameter("relative_base_deltas_0", std::vector<double>(1, 0.0));
    node->declare_parameter("relative_base_deltas_1", std::vector<double>(1, 0.0));

    node->declare_parameter("relative_tool_deltas_0", std::vector<double>(1, 0.0));
    node->declare_parameter("relative_tool_deltas_1", std::vector<double>(1, 0.0));

    node->declare_parameter("joint_poses_0",         std::vector<double>(1, 0.0));
    node->declare_parameter("joint_poses_1",         std::vector<double>(1, 0.0));

    node->declare_parameter("joint_movement_poses_0", std::vector<double>(1, 0.0));
    node->declare_parameter("joint_movement_poses_1", std::vector<double>(1, 0.0));

    moveit2_client::MoveIt2Client client(node, "movensys_manipulator_arm");

    client.base_name         = node->get_parameter("base_name").as_string();
    client.eef_name          = node->get_parameter("eef_name").as_string();
    client.vel_scale         = node->get_parameter("vel_scale").as_double();
    client.acc_scale         = node->get_parameter("acc_scale").as_double();
    client.delay_exec        = node->get_parameter("delay_exec").as_double();
    client.delay_gripper     = node->get_parameter("delay_gripper").as_double();
    client.max_step          = node->get_parameter("max_step").as_double();
    client.planning_time     = node->get_parameter("planning_time").as_double();
    client.timeout           = node->get_parameter("timeout").as_double();
    client.planning_attempts = node->get_parameter("planning_attempts").as_int();
    client.replan            = node->get_parameter("replan").as_bool();
    client.replan_attempts   = node->get_parameter("replan_attempts").as_int();
    client.pipeline_id       = node->get_parameter("pipeline_id").as_string();
    client.planner_id        = node->get_parameter("planner_id").as_string();

    RCLCPP_INFO(node->get_logger(),
        "Config: base_name=%s, eef_name=%s, vel_scale=%.2f, acc_scale=%.2f, "
        "max_step=%.2f, planning_time=%.2f, delay_exec=%.2f, delay_gripper=%.2f, timeout=%.2f",
        client.base_name.c_str(), client.eef_name.c_str(),
        client.vel_scale, client.acc_scale, client.max_step,
        client.planning_time, client.delay_exec, client.delay_gripper, client.timeout);

    runTrajectory(node, client);

    executor.cancel();
    spin_thread.join();
    rclcpp::shutdown();
    return 0;
}

static moveit2_client::PoseTarget toPose(const std::vector<double>& v)
{
    return {{v[0], v[1], v[2]}, {v[3], v[4], v[5]}};
}

static std::map<std::string, double> toJointMap(
    const std::vector<double>& v, const std::vector<std::string>& names)
{
    std::map<std::string, double> m;
    for (size_t i = 0; i < names.size(); ++i) m[names[i]] = v[i];
    return m;
}

bool runTrajectory(const rclcpp::Node::SharedPtr& node, moveit2_client::MoveIt2Client& client)
{
    const auto joint_names = node->get_parameter("joint_names").as_string_array();

    RCLCPP_INFO(node->get_logger(), "------- Joint Movement -------");
    if (!client.jointMovement(toJointMap(
            node->get_parameter("joint_initial_0").as_double_array(), joint_names))) {
        RCLCPP_ERROR(node->get_logger(), "Joint Movement failed"); return false; }

    RCLCPP_INFO(node->get_logger(), "------- Absolute Base EEF Cartesian -------");
    if (!client.absoluteBaseEefCartesian(toPose(
            node->get_parameter("cartesian_poses_0").as_double_array()))) {
        RCLCPP_ERROR(node->get_logger(), "Absolute Base EEF Cartesian move failed");
        return false;
    }
    if (!client.absoluteBaseEefCartesian(toPose(
            node->get_parameter("cartesian_poses_1").as_double_array()))) {
        RCLCPP_ERROR(node->get_logger(), "Absolute Base EEF Cartesian move failed");
        return false;
    }

    RCLCPP_INFO(node->get_logger(), "------- Relative Base EEF Cartesian -------");
    if (!client.relativeBaseEefCartesian(toPose(
            node->get_parameter("relative_base_deltas_0").as_double_array()))) {
        RCLCPP_ERROR(node->get_logger(), "Relative Base EEF Cartesian move failed");
        return false;
    }
    if (!client.relativeBaseEefCartesian(toPose(
            node->get_parameter("relative_base_deltas_1").as_double_array()))) {
        RCLCPP_ERROR(node->get_logger(), "Relative Base EEF Cartesian move failed");
        return false;
    }

    RCLCPP_INFO(node->get_logger(), "------- Relative Tool EEF Cartesian -------");
    if (!client.relativeToolEefCartesian(toPose(
            node->get_parameter("relative_tool_deltas_0").as_double_array()))) {
        RCLCPP_ERROR(node->get_logger(), "Relative Tool EEF Cartesian move failed");
        return false;
    }
    if (!client.relativeToolEefCartesian(toPose(
            node->get_parameter("relative_tool_deltas_1").as_double_array()))) {
        RCLCPP_ERROR(node->get_logger(), "Relative Tool EEF Cartesian move failed");
        return false;
    }

    RCLCPP_INFO(node->get_logger(), "------- Joint Movement -------");
    if (!client.jointMovement(toJointMap(
            node->get_parameter("joint_poses_0").as_double_array(), joint_names))) {
        RCLCPP_ERROR(node->get_logger(), "Joint Movement failed"); return false; }
    if (!client.jointMovement(toJointMap(
            node->get_parameter("joint_poses_1").as_double_array(), joint_names))) {
        RCLCPP_ERROR(node->get_logger(), "Joint Movement failed"); return false; }

    RCLCPP_INFO(node->get_logger(), "------- Absolute Base EEF Joint Movement -------");
    if (!client.absoluteBaseEefJointMovement(toPose(
            node->get_parameter("joint_movement_poses_0").as_double_array()))) {
        RCLCPP_ERROR(node->get_logger(), "Absolute Base EEF Joint Movement failed");
        return false;
    }
    if (!client.absoluteBaseEefJointMovement(toPose(
            node->get_parameter("joint_movement_poses_1").as_double_array()))) {
        RCLCPP_ERROR(node->get_logger(), "Absolute Base EEF Joint Movement failed");
        return false;
    }

    RCLCPP_INFO(node->get_logger(), "------- Gripper Service -------");
    client.setGripper(true);
    client.setGripper(false);

    RCLCPP_INFO(node->get_logger(), "All movements completed!");
    return true;
}
