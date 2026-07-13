// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#include <cmath>
#include <map>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "moveit2_client.hpp"

bool runCoverage(const rclcpp::Node::SharedPtr& node, moveit2_client::MoveIt2Client& client);

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<rclcpp::Node>("coverage_pose", node_options);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spin_thread([&executor]() { executor.spin(); });

    // All parameters (MoveIt client config + coverage waypoints + stamp config)
    // are auto-declared from yaml overrides (moveit2_client.yaml and coverage_pose.yaml).

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
    // pipeline_id / planner_id are auto-declared from the parameter overrides
    // (moveit2_client.yaml) via automatically_declare_parameters_from_overrides.
    client.pipeline_id       = node->get_parameter("pipeline_id").as_string();
    client.planner_id        = node->get_parameter("planner_id").as_string();

    RCLCPP_INFO(node->get_logger(),
        "Config: base_name=%s, eef_name=%s, vel_scale=%.2f, acc_scale=%.2f, "
        "max_step=%.2f, planning_time=%.2f, delay_exec=%.2f, delay_gripper=%.2f, timeout=%.2f",
        client.base_name.c_str(), client.eef_name.c_str(),
        client.vel_scale, client.acc_scale, client.max_step,
        client.planning_time, client.delay_exec, client.delay_gripper, client.timeout);

    runCoverage(node, client);

    executor.cancel();
    spin_thread.join();
    rclcpp::shutdown();
    return 0;
}

struct StampPose { double x, y, z, R, P, Y; };

static moveit2_client::PoseTarget toPose(const StampPose& p)
{
    return {{p.x, p.y, p.z}, {p.R, p.P, p.Y}};
}

static std::map<std::string, double> toJointMap(
    const std::vector<double>& v, const std::vector<std::string>& names)
{
    std::map<std::string, double> m;
    for (size_t i = 0; i < names.size(); ++i) m[names[i]] = v[i];
    return m;
}

bool runCoverage(const rclcpp::Node::SharedPtr& node, moveit2_client::MoveIt2Client& client)
{
    const auto joint_names = node->get_parameter("joint_names").as_string_array();

    RCLCPP_INFO(node->get_logger(), "------- Initial Joint Movement -------");
    if (!client.jointMovement(toJointMap(
            node->get_parameter("joint_initial_0").as_double_array(), joint_names))) {
        RCLCPP_ERROR(node->get_logger(), "Initial Joint Movement failed"); return false; }

    // Read waypoints
    std::vector<StampPose> waypoints;
    for (size_t i = 0; ; ++i) {
        const std::string name = "coverage_poses_" + std::to_string(i);
        if (!node->has_parameter(name)) break;
        const auto v = node->get_parameter(name).as_double_array();
        waypoints.push_back({v[0], v[1], v[2], v[3], v[4], v[5]});
    }
    if (waypoints.empty()) {
        RCLCPP_ERROR(node->get_logger(), "No coverage_poses_N parameters found"); return false;
    }

    const double stamp_step   = node->get_parameter("stamp_step_size").as_double();
    const double stamp_z_down = node->get_parameter("stamp_z_down").as_double();

    // Build full sequence of stamp positions: each waypoint, plus intermediate
    // positions every stamp_step along each segment.
    std::vector<StampPose> stamps;
    for (size_t i = 0; i < waypoints.size(); ++i) {
        stamps.push_back(waypoints[i]);
        if (i + 1 >= waypoints.size()) break;
        const auto& a = waypoints[i];
        const auto& b = waypoints[i + 1];
        const double dx = b.x - a.x;
        const double dy = b.y - a.y;
        const double dist = std::sqrt(dx*dx + dy*dy);
        const int n = std::max(1, static_cast<int>(std::ceil(dist / stamp_step)));
        for (int k = 1; k < n; ++k) {
            const double f = static_cast<double>(k) / n;
            stamps.push_back({a.x + dx*f, a.y + dy*f, a.z, a.R, a.P, a.Y});
        }
    }

    RCLCPP_INFO(node->get_logger(),
        "------- Stamping Coverage: %zu waypoints expanded to %zu stamp positions "
        "(step=%.3f m, z_down=%.3f m) -------",
        waypoints.size(), stamps.size(), stamp_step, stamp_z_down);

    // Pattern: move to (xy, z_up) -> down to z_down -> up to z_up -> next.
    for (size_t i = 0; i < stamps.size(); ++i) {
        const auto& s = stamps[i];
        RCLCPP_INFO(node->get_logger(),
            "Stamp %zu/%zu @ (%.3f, %.3f) z_up=%.3f", i + 1, stamps.size(), s.x, s.y, s.z);

        if (!client.absoluteBaseEefCartesian(toPose({s.x, s.y, s.z, s.R, s.P, s.Y}))) {
            RCLCPP_ERROR(node->get_logger(), "Move to stamp %zu (up) failed", i); return false;
        }
        if (!client.absoluteBaseEefCartesian(toPose({s.x, s.y, stamp_z_down, s.R, s.P, s.Y}))) {
            RCLCPP_ERROR(node->get_logger(), "Drop at stamp %zu failed", i); return false;
        }
        if (!client.absoluteBaseEefCartesian(toPose({s.x, s.y, s.z, s.R, s.P, s.Y}))) {
            RCLCPP_ERROR(node->get_logger(), "Lift at stamp %zu failed", i); return false;
        }
    }

    RCLCPP_INFO(node->get_logger(), "Coverage sweep completed!");
    return true;
}
