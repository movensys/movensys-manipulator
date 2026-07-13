// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#include <chrono>
#include <cmath>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "moveit2_client.hpp"

bool runNvbloxDemo(const rclcpp::Node::SharedPtr& node, moveit2_client::MoveIt2Client& client);

static moveit2_client::PoseTarget toPose(const std::vector<double>& v)
{
    return {{v[0], v[1], v[2]}, {v[3], v[4], v[5]}};
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("obstacle_avoidance_cpp");

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spin_thread([&executor]() { executor.spin(); });

    // MoveIt client config
    node->declare_parameter("base_name",         "world_manipulator");
    node->declare_parameter("eef_name",          "Link6");
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
    node->declare_parameter("delay_nvblox",           0.0);
    node->declare_parameter("joint_movement_poses_0", std::vector<double>(1, 0.0));
    node->declare_parameter("joint_movement_poses_1", std::vector<double>(1, 0.0));
    node->declare_parameter("joint_movement_poses_2", std::vector<double>(1, 0.0));

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
        "max_step=%.2f, planning_time=%.2f, delay_exec=%.2f, delay_gripper=%.2f, timeout=%.2f, "
        "planning_attempts=%d, replan=%s, replan_attempts=%d",
        client.base_name.c_str(), client.eef_name.c_str(),
        client.vel_scale, client.acc_scale, client.max_step,
        client.planning_time, client.delay_exec, client.delay_gripper, client.timeout,
        client.planning_attempts, client.replan ? "true" : "false", client.replan_attempts);

    runNvbloxDemo(node, client);

    executor.cancel();
    spin_thread.join();
    rclcpp::shutdown();
    return 0;
}

bool runNvbloxDemo(const rclcpp::Node::SharedPtr& node, moveit2_client::MoveIt2Client& client) {
    double delay_nvblox = node->get_parameter("delay_nvblox").as_double();

    RCLCPP_INFO(node->get_logger(), "------- Absolute Base EEF Joint Movement -------");
    if (!client.absoluteBaseEefJointMovement(toPose(
            node->get_parameter("joint_movement_poses_0").as_double_array()))) {
        RCLCPP_ERROR(node->get_logger(), "Joint Movement failed");
        return false;
    }

    if (!client.absoluteBaseEefJointMovement(toPose(
            node->get_parameter("joint_movement_poses_2").as_double_array()))) {
        RCLCPP_ERROR(node->get_logger(), "Move A failed");
        return false;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(delay_nvblox * 1000)));

    for (int i = 0; i < 2; i++) {
        if (!client.absoluteBaseEefJointMovement(toPose(
                node->get_parameter("joint_movement_poses_1").as_double_array()))) {
            RCLCPP_ERROR(node->get_logger(), "Move A failed");
            return false;
        }
        if (!client.absoluteBaseEefJointMovement(toPose(
                node->get_parameter("joint_movement_poses_2").as_double_array()))) {
            RCLCPP_ERROR(node->get_logger(), "Move A failed");
            return false;
        }
    }

    RCLCPP_INFO(node->get_logger(), "------- NVBLOX DEMO COMPLETE -------");
    return true;
}
