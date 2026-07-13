// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#include <chrono>
#include <cmath>
#include <map>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "moveit2_client.hpp"

struct BoxPose {
    moveit2_client::PoseTarget up;
    moveit2_client::PoseTarget down;
};

static moveit2_client::PoseTarget toPose(const std::vector<double>& v)
{
    return {{v[0], v[1], v[2]}, {v[3], v[4], v[5]}};
}

bool runYoloPickPlace(const rclcpp::Node::SharedPtr& node,
                      moveit2_client::MoveIt2Client& client);

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("yolo_trajectory_cpp");

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spin_thread([&executor]() { executor.spin(); });

    // MoveIt client config (shared across trajectory nodes via moveit2_client.yaml)
    node->declare_parameter("base_name",         "base");
    node->declare_parameter("eef_name",          "eef");
    node->declare_parameter("vel_scale",         0.1);
    node->declare_parameter("acc_scale",         0.1);
    node->declare_parameter("delay_exec",        0.5);
    node->declare_parameter("delay_gripper",     1.0);
    node->declare_parameter("max_step",          0.1);
    node->declare_parameter("planning_time",     1.0);
    node->declare_parameter("timeout",           2.0);
    node->declare_parameter("planning_attempts", 5);
    node->declare_parameter("replan",            true);
    node->declare_parameter("replan_attempts",   5);
    node->declare_parameter("pipeline_id",       "");
    node->declare_parameter("planner_id",        "");

    // YOLO pick-and-place config
    node->declare_parameter<std::string>("camera_frame", "camera_hand_link_color_optical_frame");
    node->declare_parameter<std::string>("tf_prefix",    "yolo_cube_");
    node->declare_parameter<std::vector<std::string>>(
        "cube_classes", std::vector<std::string>{"green", "blue", "yellow", "red"});

    node->declare_parameter("tol_pose",          0.03);
    node->declare_parameter("tol_orientation",   0.05);
    node->declare_parameter("descent_z",         0.21);
    node->declare_parameter("cam_to_grip_x",     0.015);
    node->declare_parameter("cam_to_grip_y",    -0.075);
    node->declare_parameter("cam_to_grip_yaw",   0.0);
    node->declare_parameter("max_wait_retries",  100);
    node->declare_parameter("tf_poll_period_ms", 100);

    node->declare_parameter("initial_pose", std::vector<double>(6, 0.0));
    node->declare_parameter("scan_pose",    std::vector<double>(6, 0.0));

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
        "[yolo_trajectory] Config: base=%s eef=%s vel=%.2f acc=%.2f max_step=%.2f "
        "planning_time=%.2f timeout=%.2f",
        client.base_name.c_str(), client.eef_name.c_str(),
        client.vel_scale, client.acc_scale, client.max_step,
        client.planning_time, client.timeout);

    runYoloPickPlace(node, client);

    executor.cancel();
    spin_thread.join();
    rclcpp::shutdown();
    return 0;
}

bool runYoloPickPlace(const rclcpp::Node::SharedPtr& node,
                      moveit2_client::MoveIt2Client& client)
{
    const auto camera_frame = node->get_parameter("camera_frame").as_string();
    const auto tf_prefix    = node->get_parameter("tf_prefix").as_string();
    const auto cube_classes = node->get_parameter("cube_classes").as_string_array();

    const double tol_pose        = node->get_parameter("tol_pose").as_double();
    const double tol_orientation = node->get_parameter("tol_orientation").as_double();
    const double descent_z       = node->get_parameter("descent_z").as_double();
    const double cam_to_grip_x   = node->get_parameter("cam_to_grip_x").as_double();
    const double cam_to_grip_y   = node->get_parameter("cam_to_grip_y").as_double();
    const double cam_to_grip_yaw = node->get_parameter("cam_to_grip_yaw").as_double();
    const int    max_wait_retries   = node->get_parameter("max_wait_retries").as_int();
    const int    tf_poll_period_ms  = node->get_parameter("tf_poll_period_ms").as_int();

    const auto initial_pose = toPose(
        node->get_parameter("initial_pose").as_double_array());
    const auto scan_pose = toPose(
        node->get_parameter("scan_pose").as_double_array());

    // Per-cube drop-box poses: declared dynamically from cube_classes.
    std::map<std::string, BoxPose> box_poses;
    for (const auto& cls : cube_classes) {
        const std::string up_key   = "box_" + cls + "_up";
        const std::string down_key = "box_" + cls + "_down";
        node->declare_parameter(up_key,   std::vector<double>(6, 0.0));
        node->declare_parameter(down_key, std::vector<double>(6, 0.0));
        box_poses[cls] = {
            toPose(node->get_parameter(up_key).as_double_array()),
            toPose(node->get_parameter(down_key).as_double_array())
        };
    }

    RCLCPP_INFO(node->get_logger(), "======= YOLO PICK-AND-PLACE START =======");

    if (!client.absoluteBaseEefCartesian(initial_pose)) {
        RCLCPP_ERROR(node->get_logger(), "Move to initial pose failed");
        return false;
    }

    if (!client.setGripper(false)) {
        RCLCPP_ERROR(node->get_logger(), "Open gripper failed");
        return false;
    }

    for (size_t i = 0; i < cube_classes.size(); ++i) {
        const std::string& cls      = cube_classes[i];
        const std::string  tf_child = tf_prefix + cls;

        RCLCPP_INFO(node->get_logger(),
            "[%zu/%zu] ── cube: %s  (TF child: %s) ──",
            i + 1, cube_classes.size(), cls.c_str(), tf_child.c_str());

        if (!client.absoluteBaseEefCartesian(scan_pose)) {
            RCLCPP_ERROR(node->get_logger(), "Move to scan pose failed");
            return false;
        }

        // Wait for the TF frame to appear; skip this cube if it never does.
        bool tf_found = false;
        for (int retries = 0; retries < max_wait_retries; ++retries) {
            auto tf = client.lookupTF(camera_frame, tf_child);
            if (tf.has_value()) {
                RCLCPP_INFO(node->get_logger(),
                    "TF frame '%s' acquired after %d poll(s).",
                    tf_child.c_str(), retries + 1);
                tf_found = true;
                break;
            }
            std::this_thread::sleep_for(
                std::chrono::milliseconds(tf_poll_period_ms));
        }
        if (!tf_found) {
            RCLCPP_WARN(node->get_logger(),
                "TF frame '%s' not seen after %d retries — skipping cube.",
                tf_child.c_str(), max_wait_retries);
            continue;
        }

        // Visual-servoing loop: centre camera over cube in tool frame.
        double x_tag = 0.0, y_tag = 0.0, yaw_tag = 0.0;
        while (true) {
            auto tf = client.lookupTF(camera_frame, tf_child);

            if (!tf.has_value()) {
                RCLCPP_WARN(node->get_logger(),
                    "Lost TF for '%s', retrying …", tf_child.c_str());
                std::this_thread::sleep_for(
                    std::chrono::milliseconds(tf_poll_period_ms));
                continue;
            }

            x_tag   = tf->x;
            y_tag   = tf->y;
            yaw_tag = tf->yaw;

            RCLCPP_INFO(node->get_logger(),
                "  %s offset  x=%.3f  y=%.3f  yaw=%.3f",
                cls.c_str(), x_tag, y_tag, yaw_tag);

            if (std::abs(x_tag)   < tol_pose        &&
                std::abs(y_tag)   < tol_pose        &&
                std::abs(yaw_tag) < tol_orientation)
            {
                RCLCPP_INFO(node->get_logger(),
                    "  ##### Converged on %s #####", cls.c_str());
                break;
            }

            moveit2_client::PoseTarget delta = {
                {x_tag, y_tag, 0.0}, {0.0, 0.0, yaw_tag}};
            if (!client.relativeToolEefCartesian(delta)) {
                RCLCPP_ERROR(node->get_logger(),
                    "Tracking move failed for %s", cls.c_str());
                return false;
            }
        }

        moveit2_client::PoseTarget grip_offset = {
            {cam_to_grip_x + x_tag,
             cam_to_grip_y + y_tag,
             0.0},
            {0.0, 0.0, cam_to_grip_yaw + yaw_tag}
        };
        if (!client.relativeToolEefCartesian(grip_offset)) {
            RCLCPP_ERROR(node->get_logger(),
                "Camera-to-gripper offset move failed for %s", cls.c_str());
            return false;
        }

        moveit2_client::PoseTarget down = {{0.0, 0.0, -descent_z}, {0.0, 0.0, 0.0}};
        if (!client.relativeBaseEefCartesian(down)) {
            RCLCPP_ERROR(node->get_logger(), "Descend failed for %s", cls.c_str());
            return false;
        }

        if (!client.setGripper(true)) {
            RCLCPP_ERROR(node->get_logger(), "Close gripper failed for %s", cls.c_str());
            return false;
        }

        moveit2_client::PoseTarget up = {{0.0, 0.0, descent_z}, {0.0, 0.0, 0.0}};
        if (!client.relativeBaseEefCartesian(up)) {
            RCLCPP_ERROR(node->get_logger(), "Lift failed for %s", cls.c_str());
            return false;
        }

        const auto it = box_poses.find(cls);
        if (it == box_poses.end()) {
            RCLCPP_ERROR(node->get_logger(),
                "No drop-box defined for '%s' — check yolo_trajectory.yaml",
                cls.c_str());
            return false;
        }
        const auto& box = it->second;

        if (!client.absoluteBaseEefCartesian(box.up)) {
            RCLCPP_ERROR(node->get_logger(), "Move to box-up failed for %s", cls.c_str());
            return false;
        }
        if (!client.absoluteBaseEefCartesian(box.down)) {
            RCLCPP_ERROR(node->get_logger(), "Move to box-down failed for %s", cls.c_str());
            return false;
        }
        if (!client.setGripper(false)) {
            RCLCPP_ERROR(node->get_logger(), "Open gripper failed for %s", cls.c_str());
            return false;
        }
        if (!client.absoluteBaseEefCartesian(box.up)) {
            RCLCPP_ERROR(node->get_logger(), "Retract from box failed for %s", cls.c_str());
            return false;
        }

        RCLCPP_INFO(node->get_logger(), "  %s placed successfully.", cls.c_str());
    }

    if (!client.absoluteBaseEefCartesian(scan_pose)) {
        RCLCPP_ERROR(node->get_logger(), "Return to scan pose failed");
        return false;
    }

    RCLCPP_INFO(node->get_logger(), "======= YOLO PICK-AND-PLACE COMPLETE =======");
    return true;
}
