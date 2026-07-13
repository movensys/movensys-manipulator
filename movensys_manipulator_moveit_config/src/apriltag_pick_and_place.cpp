// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#include <cstdlib>

#include <chrono>
#include <cmath>
#include <map>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "moveit2_client.hpp"

bool runAprilTagPickPlace(const rclcpp::Node::SharedPtr& node,
                          moveit2_client::MoveIt2Client& client);

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("apriltag_pick_and_place");

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

    // AprilTag pick-and-place config
    node->declare_parameter<bool>("target_spawn",  true);
    node->declare_parameter("camera_hand_link",    std::string("camera_hand_color_optical_frame"));
    node->declare_parameter("tag_ids",
        std::vector<std::string>{"tag36h11:5", "tag36h11:9", "tag36h11:7", "tag36h11:11"});
    node->declare_parameter("target_topics",
        std::vector<std::string>{"/target1_sub", "/target2_sub", "/target3_sub", "/target4_sub"});

    node->declare_parameter("z_target_pose_spawn",       0.0);
    node->declare_parameter("tag_down_z",          0.0);
    node->declare_parameter("tag_tol_pose",        0.0);
    node->declare_parameter("tag_tol_orientation", 0.0);
    node->declare_parameter("tag_to_target_x",     0.0);
    node->declare_parameter("tag_to_target_y",     0.0);
    node->declare_parameter("tag_to_target_yaw",   0.0);

    node->declare_parameter("joint_names",
        std::vector<std::string>{"j1", "j2", "j3", "j4", "j5", "j6"});
    node->declare_parameter("initial_pose", std::vector<double>(6, 0.0));
    node->declare_parameter("scan_pose",    std::vector<double>(6, 0.0));

    node->declare_parameter("box_up_0",   std::vector<double>(6, 0.0));
    node->declare_parameter("box_down_0", std::vector<double>(6, 0.0));
    node->declare_parameter("box_up_1",   std::vector<double>(6, 0.0));
    node->declare_parameter("box_down_1", std::vector<double>(6, 0.0));
    node->declare_parameter("box_up_2",   std::vector<double>(6, 0.0));
    node->declare_parameter("box_down_2", std::vector<double>(6, 0.0));
    node->declare_parameter("box_up_3",   std::vector<double>(6, 0.0));
    node->declare_parameter("box_down_3", std::vector<double>(6, 0.0));

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

    runAprilTagPickPlace(node, client);

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

bool runAprilTagPickPlace(const rclcpp::Node::SharedPtr& node,
                          moveit2_client::MoveIt2Client& client)
{
    const bool   target_spawn        = node->get_parameter("target_spawn").as_bool();
    const auto   camera_hand_link    = node->get_parameter("camera_hand_link").as_string();
    const auto   tag_ids             = node->get_parameter("tag_ids").as_string_array();
    const auto   target_topic_name   = node->get_parameter("target_topics").as_string_array();

    const double z_target_pose_spawn = node->get_parameter("z_target_pose_spawn").as_double();
    const double tag_down_z          = node->get_parameter("tag_down_z").as_double();
    const double tag_tol_pose        = node->get_parameter("tag_tol_pose").as_double();
    const double tag_tol_orientation = node->get_parameter("tag_tol_orientation").as_double();
    const double tag_to_target_x     = node->get_parameter("tag_to_target_x").as_double();
    const double tag_to_target_y     = node->get_parameter("tag_to_target_y").as_double();
    const double tag_to_target_yaw   = node->get_parameter("tag_to_target_yaw").as_double();

    const auto joint_names  = node->get_parameter("joint_names").as_string_array();
    const auto initial_pose = toJointMap(
        node->get_parameter("initial_pose").as_double_array(), joint_names);
    const auto scan_pose    = toPose(node->get_parameter("scan_pose").as_double_array());

    const std::vector<moveit2_client::PoseTarget> box_up = {
        toPose(node->get_parameter("box_up_0").as_double_array()),
        toPose(node->get_parameter("box_up_1").as_double_array()),
        toPose(node->get_parameter("box_up_2").as_double_array()),
        toPose(node->get_parameter("box_up_3").as_double_array()),
    };
    const std::vector<moveit2_client::PoseTarget> box_down = {
        toPose(node->get_parameter("box_down_0").as_double_array()),
        toPose(node->get_parameter("box_down_1").as_double_array()),
        toPose(node->get_parameter("box_down_2").as_double_array()),
        toPose(node->get_parameter("box_down_3").as_double_array()),
    };

    RCLCPP_INFO(node->get_logger(), "------- START CYCLE -------");

    if (!client.jointMovement(initial_pose)) {
        RCLCPP_ERROR(node->get_logger(), "Move to initial pose failed"); return false; }

    if (!client.setGripper(false)) {
        RCLCPP_ERROR(node->get_logger(), "Failed to open gripper"); return false; }

    for (size_t i = 0; i < tag_ids.size(); ++i) {
        RCLCPP_INFO(node->get_logger(), "[%zu/%zu] Running target id = %s",
            i + 1, tag_ids.size(), tag_ids[i].c_str());

        if (!client.absoluteBaseEefJointMovement(scan_pose)) {
            RCLCPP_ERROR(node->get_logger(), "Move to scan pose failed"); return false; }

        double x_tag = 0.0, y_tag = 0.0, yaw_tag = 0.0;

        while (rclcpp::ok()) {
            auto tf_result = client.lookupTF(camera_hand_link, tag_ids[i]);

            if (!tf_result.has_value()) {
                RCLCPP_WARN(node->get_logger(), "No TF for tag, retrying...");
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }

            x_tag   = tf_result->x;
            y_tag   = tf_result->y;
            yaw_tag = tf_result->yaw;

            if (std::abs(x_tag) < tag_tol_pose && std::abs(y_tag) < tag_tol_pose &&
                std::abs(yaw_tag) < tag_tol_orientation) {
                RCLCPP_INFO(node->get_logger(), "##### Converged to tag. #####");
                break;
            }

            moveit2_client::PoseTarget delta_pose = {{x_tag, y_tag, 0.0}, {0.0, 0.0, yaw_tag}};
            if (!client.relativeToolEefCartesian(delta_pose)) {
                RCLCPP_ERROR(node->get_logger(), "Tracking movement failed"); return false; }
        }

        if (!rclcpp::ok()) {
            RCLCPP_WARN(node->get_logger(), "Shutdown requested, aborting.");
            return false;
        }

        moveit2_client::PoseTarget target_offset = {
            {tag_to_target_x + x_tag, tag_to_target_y + y_tag, 0.0},
            {0.0, 0.0, tag_to_target_yaw + yaw_tag}
        };
        if (!client.relativeToolEefCartesian(target_offset)) {
            RCLCPP_ERROR(node->get_logger(), "Move to target center failed"); return false; }

        if (target_spawn) {
            auto eef_tf = client.getCurrentEefPose();
            if (!eef_tf.has_value()) {
                RCLCPP_ERROR(node->get_logger(), "Failed to get current EEF pose"); return false; }

            double x_eef = -eef_tf->y;
            double y_eef =  eef_tf->x;

            std::string pose_str =
                "{position: {x: "    + std::to_string(x_eef) +
                ", y: "              + std::to_string(y_eef) +
                ", z: "              + std::to_string(z_target_pose_spawn) + "}" +
                ", orientation: {x: "+ std::to_string(eef_tf->qx) +
                ", y: "              + std::to_string(eef_tf->qy) +
                ", z: "              + std::to_string(eef_tf->qz) +
                ", w: "              + std::to_string(eef_tf->qw) + "}}";

            RCLCPP_INFO(node->get_logger(),
                "Teleport target pose in Isaac Sim: %s", pose_str.c_str());

            std::string cmd = "ros2 topic pub -1 " + target_topic_name[i] +
                              " geometry_msgs/msg/Pose \"" + pose_str + "\"";
            int result = std::system(cmd.c_str());
            RCLCPP_INFO(node->get_logger(), "ros2 topic pub result: %d", result);
        }

        moveit2_client::PoseTarget down_delta = {{0.0, 0.0, -tag_down_z}, {0.0, 0.0, 0.0}};
        if (!client.relativeBaseEefCartesian(down_delta)) {
            RCLCPP_ERROR(node->get_logger(), "Move down failed"); return false; }

        if (!client.setGripper(true)) {
            RCLCPP_ERROR(node->get_logger(), "Failed to close gripper"); return false; }

        moveit2_client::PoseTarget up_delta = {{0.0, 0.0, tag_down_z}, {0.0, 0.0, 0.0}};
        if (!client.relativeBaseEefCartesian(up_delta)) {
            RCLCPP_ERROR(node->get_logger(), "Move up failed"); return false; }

        if (!client.absoluteBaseEefJointMovement(box_up[i])) {
            RCLCPP_ERROR(node->get_logger(), "Move to box up pose failed"); return false; }

        if (!client.absoluteBaseEefCartesian(box_down[i])) {
            RCLCPP_ERROR(node->get_logger(), "Move to box down pose failed"); return false; }

        if (!client.setGripper(false)) {
            RCLCPP_ERROR(node->get_logger(), "Failed to open gripper"); return false; }

        if (!client.absoluteBaseEefCartesian(box_up[i])) {
            RCLCPP_ERROR(node->get_logger(),
                "Move to box up pose (return) failed"); return false; }
    }

    RCLCPP_INFO(node->get_logger(), "------- STOP CYCLE -------");
    return true;
}
