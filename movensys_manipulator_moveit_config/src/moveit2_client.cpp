// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#include "moveit2_client.hpp"

#include <Eigen/Geometry>

#include <chrono>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace moveit2_client {

MoveIt2Client::MoveIt2Client(const rclcpp::Node::SharedPtr& node, const std::string& group_name)
    : node_(node), group_name_(group_name){
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        node_, group_name);
    gripper_client_ = node_->create_client<std_srvs::srv::SetBool>("/wmx/set_gripper");

    sequence_client_ = rclcpp_action::create_client<moveit_msgs::action::MoveGroupSequence>(
        node_, "/sequence_move_group");

    tf_sub_ = node_->create_subscription<tf2_msgs::msg::TFMessage>(
        "/tf", 10,
        std::bind(&MoveIt2Client::tfCallback, this, std::placeholders::_1));

    RCLCPP_INFO(node_->get_logger(),
        "MoveIt2Client initialized for group: %s", group_name.c_str());
}

void MoveIt2Client::tfCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(tf_mutex_);
    for (const auto& transform : msg->transforms) {
        std::string key = transform.header.frame_id + "->" + transform.child_frame_id;
        tf_cache_[key] = transform;
    }
}

geometry_msgs::msg::Pose MoveIt2Client::createPose(const PoseTarget& target){
    geometry_msgs::msg::Pose pose;
    pose.position.x = target.pos[0];
    pose.position.y = target.pos[1];
    pose.position.z = target.pos[2];

    tf2::Quaternion q;
    q.setRPY(target.ori[0], target.ori[1], target.ori[2]);
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();

    return pose;
}

void MoveIt2Client::logCurrentState(){
    auto robot_state = move_group_->getCurrentState(timeout);
    if (!robot_state) {
        RCLCPP_WARN(node_->get_logger(), "Could not get current state for logging");
        return;
    }

    // Log joint values
    std::vector<double> joint_values;
    robot_state->copyJointGroupPositions(move_group_->getName(), joint_values);
    auto joint_names = move_group_->getJointNames();
    std::string joints_str = "{ ";
    for (size_t i = 0; i < joint_names.size() && i < joint_values.size(); ++i) {
        joints_str += joint_names[i] + ": " + std::to_string(joint_values[i]) + ", ";
    }
    if (!joint_names.empty()) {
        joints_str = joints_str.substr(0, joints_str.length() - 2);
    }
    joints_str += " }";

    // Log EEF state
    const Eigen::Isometry3d& eef_transform = robot_state->getGlobalLinkTransform(eef_name);
    Eigen::Vector3d pos = eef_transform.translation();
    Eigen::Quaterniond quat(eef_transform.rotation());
    tf2::Quaternion q(quat.x(), quat.y(), quat.z(), quat.w());
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    RCLCPP_INFO(node_->get_logger(), "Current joint: %s", joints_str.c_str());
    RCLCPP_INFO(node_->get_logger(),
                "Current EEF: pos=[%.3f, %.3f, %.3f], rpy=[%.3f, %.3f, %.3f]",
                pos.x(), pos.y(), pos.z(), roll, pitch, yaw);
}

bool MoveIt2Client::jointMovement(const std::map<std::string, double>& joint_targets){
    std::string targets_str = "{ ";
    for (const auto& [name, value] : joint_targets) {
        targets_str += name + ": " + std::to_string(value) + ", ";
    }
    if (!joint_targets.empty()) {
        targets_str = targets_str.substr(0, targets_str.length() - 2);
    }
    targets_str += " }";

    move_group_->setEndEffectorLink(eef_name);
    move_group_->setPlanningTime(planning_time);
    move_group_->setMaxVelocityScalingFactor(vel_scale);
    move_group_->setMaxAccelerationScalingFactor(acc_scale);
    move_group_->setNumPlanningAttempts(planning_attempts);
    move_group_->allowReplanning(replan);

    logCurrentState();
    RCLCPP_INFO(node_->get_logger(), "Sending joint movement = %s", targets_str.c_str());

    move_group_->setJointValueTarget(joint_targets);

    moveit::core::MoveItErrorCode result;
    for (int attempt = 0; attempt <= replan_attempts; ++attempt) {
        result = move_group_->move();
        if (result == moveit::core::MoveItErrorCode::SUCCESS) {
            break;
        }
        if (attempt < replan_attempts) {
            RCLCPP_WARN(node_->get_logger(),
                "Joint movement attempt %d failed, retrying...", attempt + 1);
        }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(delay_exec * 1000)));

    return result == moveit::core::MoveItErrorCode::SUCCESS;
}

bool MoveIt2Client::relativeJointMovement(const std::map<std::string, double>& joint_deltas){
    auto robot_state = move_group_->getCurrentState(timeout);
    if (!robot_state) {
        RCLCPP_ERROR(node_->get_logger(), "relativeJointMovement: getCurrentState failed");
        return false;
    }

    std::vector<double> current_values;
    robot_state->copyJointGroupPositions(move_group_->getName(), current_values);
    auto joint_names = move_group_->getJointNames();

    std::map<std::string, double> targets;
    for (size_t i = 0; i < joint_names.size(); ++i) {
        targets[joint_names[i]] = current_values[i];
    }
    for (const auto& [name, delta] : joint_deltas) {
        auto it = targets.find(name);
        if (it != targets.end()) {
            it->second += delta;
        } else {
            RCLCPP_WARN(node_->get_logger(),
                "relativeJointMovement: unknown joint '%s', skipping", name.c_str());
        }
    }

    RCLCPP_INFO(node_->get_logger(),
        "Sending relative joint movement for %zu joints", joint_deltas.size());
    return jointMovement(targets);
}

bool MoveIt2Client::absoluteBaseEefJointMovement(const PoseTarget& target){
    move_group_->setEndEffectorLink(eef_name);
    move_group_->setPlanningTime(planning_time);
    move_group_->setMaxVelocityScalingFactor(vel_scale);
    move_group_->setMaxAccelerationScalingFactor(acc_scale);
    move_group_->setNumPlanningAttempts(planning_attempts);
    move_group_->allowReplanning(replan);

    logCurrentState();
    RCLCPP_INFO(node_->get_logger(),
                "Sending absolute base eef joint movement = "
                "pos: [%.3f, %.3f, %.3f], ori: [%.3f, %.3f, %.3f]",
                target.pos[0], target.pos[1], target.pos[2],
                target.ori[0], target.ori[1], target.ori[2]);

    geometry_msgs::msg::Pose pose = createPose(target);
    move_group_->setPoseTarget(pose);

    moveit::core::MoveItErrorCode result;
    for (int attempt = 0; attempt <= replan_attempts; ++attempt) {
        result = move_group_->move();
        if (result == moveit::core::MoveItErrorCode::SUCCESS) {
            break;
        }
        if (attempt < replan_attempts) {
            RCLCPP_WARN(node_->get_logger(),
                "Absolute base eef joint movement attempt %d failed, retrying...",
                attempt + 1);
        }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(delay_exec * 1000)));

    return result == moveit::core::MoveItErrorCode::SUCCESS;
}

bool MoveIt2Client::absoluteBaseEefCartesian(const PoseTarget& target){
    move_group_->setEndEffectorLink(eef_name);
    move_group_->setPlanningTime(planning_time);

    logCurrentState();
    RCLCPP_INFO(node_->get_logger(),
                "Sending absolute base eef cartesian = "
                "pos: [%.3f, %.3f, %.3f], ori: [%.3f, %.3f, %.3f]",
                target.pos[0], target.pos[1], target.pos[2],
                target.ori[0], target.ori[1], target.ori[2]);

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(createPose(target));

    moveit_msgs::msg::RobotTrajectory trajectory_msg;

#ifdef MOVEIT2_JAZZY
    double fraction = move_group_->computeCartesianPath(waypoints, max_step, trajectory_msg);
#elif defined(MOVEIT2_HUMBLE)
    double fraction = move_group_->computeCartesianPath(waypoints, max_step, 0.0, trajectory_msg);
#endif

    if (fraction < 1.0) {
        RCLCPP_WARN(node_->get_logger(),
            "Cartesian path fraction: %.2f (incomplete)", fraction);
        RCLCPP_ERROR(node_->get_logger(), "Cartesian path planning failed");
        return false;
    }

    // Apply velocity and acceleration scaling to the Cartesian trajectory
    robot_trajectory::RobotTrajectory robot_traj(
        move_group_->getRobotModel(), move_group_->getName());
    auto current_state = move_group_->getCurrentState(timeout);
    if (!current_state) {
        RCLCPP_ERROR(node_->get_logger(),
            "Failed to get current state for trajectory processing");
        return false;
    }
    robot_traj.setRobotTrajectoryMsg(*current_state, trajectory_msg);

    trajectory_processing::TimeOptimalTrajectoryGeneration totg;
    if (!totg.computeTimeStamps(robot_traj, vel_scale, acc_scale)) {
        RCLCPP_ERROR(node_->get_logger(),
            "Failed to compute time stamps for Cartesian trajectory");
        return false;
    }

    moveit_msgs::msg::RobotTrajectory scaled_trajectory;
    robot_traj.getRobotTrajectoryMsg(scaled_trajectory);

    auto result = move_group_->execute(scaled_trajectory);
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(delay_exec * 1000)));

    return result == moveit::core::MoveItErrorCode::SUCCESS;
}

std::optional<PoseTarget> MoveIt2Client::baseDeltaToAbsolute(const PoseTarget& delta){
    move_group_->setEndEffectorLink(eef_name);
    auto robot_state = move_group_->getCurrentState(timeout);
    if (!robot_state) {
        RCLCPP_ERROR(node_->get_logger(), "baseDeltaToAbsolute: getCurrentState failed");
        return std::nullopt;
    }

    const Eigen::Isometry3d& eef_transform = robot_state->getGlobalLinkTransform(eef_name);
    Eigen::Vector3d pos = eef_transform.translation();
    Eigen::Quaterniond quat(eef_transform.rotation());

    double x = pos.x(), y = pos.y(), z = pos.z();
    double roll, pitch, yaw;
    tf2::Quaternion q(quat.x(), quat.y(), quat.z(), quat.w());
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    PoseTarget absolute_target;
    absolute_target.pos = {x + delta.pos[0], y + delta.pos[1], z + delta.pos[2]};
    absolute_target.ori = {roll + delta.ori[0], pitch + delta.ori[1], yaw + delta.ori[2]};
    return absolute_target;
}

std::optional<PoseTarget> MoveIt2Client::toolDeltaToAbsolute(const PoseTarget& delta){
    move_group_->setEndEffectorLink(eef_name);
    auto robot_state = move_group_->getCurrentState(timeout);
    if (!robot_state) {
        RCLCPP_ERROR(node_->get_logger(), "toolDeltaToAbsolute: getCurrentState failed");
        return std::nullopt;
    }

    Eigen::Isometry3d T_base_eef = robot_state->getGlobalLinkTransform(eef_name);

    Eigen::Isometry3d T_delta = Eigen::Isometry3d::Identity();
    T_delta.translation() = Eigen::Vector3d(delta.pos[0], delta.pos[1], delta.pos[2]);
    T_delta.linear() =
        (Eigen::AngleAxisd(delta.ori[2], Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(delta.ori[1], Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(delta.ori[0], Eigen::Vector3d::UnitX())).toRotationMatrix();

    Eigen::Isometry3d T_target = T_base_eef * T_delta;
    Eigen::Vector3d pos = T_target.translation();
    Eigen::Quaterniond q_target(T_target.rotation());

    tf2::Quaternion tf_q(q_target.x(), q_target.y(), q_target.z(), q_target.w());
    double target_roll, target_pitch, target_yaw;
    tf2::Matrix3x3(tf_q).getRPY(target_roll, target_pitch, target_yaw);

    PoseTarget absolute_target;
    absolute_target.pos = {pos.x(), pos.y(), pos.z()};
    absolute_target.ori = {target_roll, target_pitch, target_yaw};
    return absolute_target;
}

bool MoveIt2Client::relativeBaseEefCartesian(const PoseTarget& delta){
    RCLCPP_INFO(node_->get_logger(),
                "Sending relative base eef cartesian = "
                "pos: [%.3f, %.3f, %.3f], ori: [%.3f, %.3f, %.3f]",
                delta.pos[0], delta.pos[1], delta.pos[2],
                delta.ori[0], delta.ori[1], delta.ori[2]);

    auto absolute_target = baseDeltaToAbsolute(delta);
    if (!absolute_target) {
        return false;
    }
    return absoluteBaseEefCartesian(*absolute_target);
}

bool MoveIt2Client::relativeToolEefCartesian(const PoseTarget& delta){
    RCLCPP_INFO(node_->get_logger(),
                "Sending relative tool eef cartesian = "
                "pos: [%.3f, %.3f, %.3f], ori: [%.3f, %.3f, %.3f]",
                delta.pos[0], delta.pos[1], delta.pos[2],
                delta.ori[0], delta.ori[1], delta.ori[2]);

    auto absolute_target = toolDeltaToAbsolute(delta);
    if (!absolute_target) {
        return false;
    }
    return absoluteBaseEefCartesian(*absolute_target);
}

bool MoveIt2Client::blendedLinSequence(const std::vector<PoseTarget>& waypoints,
                                       const std::vector<double>& blend_radii){
    if (waypoints.empty() || waypoints.size() != blend_radii.size()) {
        RCLCPP_ERROR(node_->get_logger(),
            "blendedLinSequence: waypoints (%zu) and blend_radii (%zu) size mismatch",
            waypoints.size(), blend_radii.size());
        return false;
    }
    if (blend_radii.back() != 0.0) {
        RCLCPP_ERROR(node_->get_logger(),
            "blendedLinSequence: last blend_radius must be 0.0 (got %.3f)",
            blend_radii.back());
        return false;
    }

    if (!sequence_client_->wait_for_action_server(std::chrono::duration<double>(timeout))) {
        RCLCPP_ERROR(node_->get_logger(),
            "blendedLinSequence: /sequence_move_group action server not available. "
            "Is the Pilz MoveGroupSequenceAction capability loaded on move_group?");
        return false;
    }

    move_group_->setEndEffectorLink(eef_name);

    moveit_msgs::msg::MotionSequenceRequest seq_req;
    for (size_t i = 0; i < waypoints.size(); ++i) {
        moveit_msgs::msg::MotionSequenceItem item;
        item.blend_radius = blend_radii[i];

        auto& req = item.req;
        req.group_name                      = group_name_;
        req.pipeline_id                     = "pilz_industrial_motion_planner";
        req.planner_id                      = "LIN";
        req.num_planning_attempts           = planning_attempts;
        req.allowed_planning_time           = planning_time;
        req.max_velocity_scaling_factor     = vel_scale;
        req.max_acceleration_scaling_factor = acc_scale;

        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = base_name;
        pose_stamped.pose = createPose(waypoints[i]);

        req.goal_constraints.push_back(
            kinematic_constraints::constructGoalConstraints(
                eef_name, pose_stamped, goal_tol_pos, goal_tol_ang));

        // start_state left default (empty) on the first item so the server uses
        // the current state; Pilz chains each subsequent item's start to the
        // previous item's goal.
        seq_req.items.push_back(item);
    }

    moveit_msgs::action::MoveGroupSequence::Goal goal;
    goal.request = seq_req;
    goal.planning_options.plan_only = false;

    RCLCPP_INFO(node_->get_logger(),
        "Sending blended LIN sequence with %zu waypoints", waypoints.size());

    auto goal_handle_future = sequence_client_->async_send_goal(goal);
    if (goal_handle_future.wait_for(std::chrono::duration<double>(timeout)) !=
            std::future_status::ready) {
        RCLCPP_ERROR(node_->get_logger(), "blendedLinSequence: send_goal timed out");
        return false;
    }
    auto goal_handle = goal_handle_future.get();
    if (!goal_handle) {
        RCLCPP_ERROR(node_->get_logger(), "blendedLinSequence: goal was rejected");
        return false;
    }

    auto result_future = sequence_client_->async_get_result(goal_handle);
    // Sequence execution can take much longer than a single planning call, so
    // wait without the short service timeout.
    if (result_future.wait_for(std::chrono::seconds(120)) != std::future_status::ready) {
        RCLCPP_ERROR(node_->get_logger(), "blendedLinSequence: result timed out");
        return false;
    }

    auto wrapped_result = result_future.get();
    const bool ok =
        wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED &&
        wrapped_result.result &&
        wrapped_result.result->response.error_code.val ==
            moveit_msgs::msg::MoveItErrorCodes::SUCCESS;

    if (!ok) {
        int32_t err = wrapped_result.result
            ? wrapped_result.result->response.error_code.val : 0;
        RCLCPP_ERROR(node_->get_logger(),
            "blendedLinSequence failed (action code=%d, moveit error=%d)",
            static_cast<int>(wrapped_result.code), err);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(delay_exec * 1000)));
    return ok;
}

std::optional<TFResult> MoveIt2Client::lookupTF(const std::string& parent_frame,
                                                 const std::string& child_frame) {
    std::string key = parent_frame + "->" + child_frame;
    geometry_msgs::msg::TransformStamped transform;

    {
        std::lock_guard<std::mutex> lock(tf_mutex_);
        auto it = tf_cache_.find(key);
        if (it == tf_cache_.end()) {
            RCLCPP_WARN(node_->get_logger(),
                "TF lookup failed: %s not found in cache", key.c_str());
            return std::nullopt;
        }
        transform = it->second;
    }

    TFResult result;
    result.x = transform.transform.translation.x;
    result.y = transform.transform.translation.y;
    result.z = transform.transform.translation.z;
    result.qx = transform.transform.rotation.x;
    result.qy = transform.transform.rotation.y;
    result.qz = transform.transform.rotation.z;
    result.qw = transform.transform.rotation.w;

    tf2::Quaternion q(result.qx, result.qy, result.qz, result.qw);
    tf2::Matrix3x3(q).getRPY(result.roll, result.pitch, result.yaw);

    RCLCPP_INFO(node_->get_logger(),
                "\nTF found\n"
                "Link: parent= %s child= %s\n"
                "Translation: x=%.3f, y=%.3f, z=%.3f\n"
                "Euler: roll=%.3f, pitch=%.3f, yaw=%.3f\n"
                "Quaternion: qx=%.3f, qy=%.3f, qz=%.3f, qw=%.3f",
                parent_frame.c_str(), child_frame.c_str(),
                result.x, result.y, result.z,
                result.roll, result.pitch, result.yaw,
                result.qx, result.qy, result.qz, result.qw);

    return result;
}

std::optional<TFResult> MoveIt2Client::getCurrentEefPose() {
    move_group_->setEndEffectorLink(eef_name);
    auto robot_state = move_group_->getCurrentState(timeout);
    if (!robot_state) {
        RCLCPP_ERROR(node_->get_logger(), "getCurrentState failed");
        return std::nullopt;
    }

    const Eigen::Isometry3d& eef_transform = robot_state->getGlobalLinkTransform(eef_name);
    Eigen::Vector3d pos = eef_transform.translation();
    Eigen::Quaterniond quat(eef_transform.rotation());

    TFResult result;
    result.x = pos.x();
    result.y = pos.y();
    result.z = pos.z();
    result.qx = quat.x();
    result.qy = quat.y();
    result.qz = quat.z();
    result.qw = quat.w();

    tf2::Quaternion q(result.qx, result.qy, result.qz, result.qw);
    tf2::Matrix3x3(q).getRPY(result.roll, result.pitch, result.yaw);

    return result;
}

bool MoveIt2Client::setGripper(bool gripper_value){
    if (!gripper_client_->wait_for_service(std::chrono::duration<double>(timeout))) {
        RCLCPP_WARN(node_->get_logger(), "Gripper service not available");
        return false;
    }

    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = gripper_value;

    auto future = gripper_client_->async_send_request(request);

    auto status = future.wait_for(std::chrono::duration<double>(timeout));
    if (status != std::future_status::ready) {
        RCLCPP_ERROR(node_->get_logger(), "Gripper service call timeout");
        return false;
    }

    auto result = future.get();
    if (result->success) {
        RCLCPP_INFO(node_->get_logger(), "Gripper: %s", gripper_value ? "Close" : "Open");
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(delay_gripper * 1000)));
    return result->success;
}

}  // namespace moveit2_client
