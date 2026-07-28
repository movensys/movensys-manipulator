// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#include "moveit2_api.hpp"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

MoveIt2ApiNode::MoveIt2ApiNode(){
    node_ = std::make_shared<rclcpp::Node>("moveit2_api");

    node_->declare_parameter("base_name",         "base");
    node_->declare_parameter("eef_name",         "eef");
    node_->declare_parameter("vel_scale",         0.0);
    node_->declare_parameter("acc_scale",         0.0);
    node_->declare_parameter("delay_exec",        0.0);
    node_->declare_parameter("max_step",          0.0);
    node_->declare_parameter("planning_time",     0.0);
    node_->declare_parameter("timeout",           0.0);
    node_->declare_parameter("planning_attempts", 0);
    node_->declare_parameter("replan",            true);
    node_->declare_parameter("replan_attempts",   0);

    client_ = std::make_shared<moveit2_client::MoveIt2Client>(node_, "movensys_manipulator_arm");

    client_->base_name         = node_->get_parameter("base_name").as_string();
    client_->eef_name         = node_->get_parameter("eef_name").as_string();
    client_->vel_scale         = node_->get_parameter("vel_scale").as_double();
    client_->acc_scale         = node_->get_parameter("acc_scale").as_double();
    client_->delay_exec        = node_->get_parameter("delay_exec").as_double();
    client_->max_step          = node_->get_parameter("max_step").as_double();
    client_->planning_time     = node_->get_parameter("planning_time").as_double();
    client_->timeout           = node_->get_parameter("timeout").as_double();
    client_->planning_attempts = node_->get_parameter("planning_attempts").as_int();
    client_->replan            = node_->get_parameter("replan").as_bool();
    client_->replan_attempts   = node_->get_parameter("replan_attempts").as_int();

    // Live vel_scale / acc_scale updates via ROS2 parameter service
    param_cb_handle_ = node_->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& params)
        -> rcl_interfaces::msg::SetParametersResult {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            for (const auto& p : params) {
                if (p.get_name() == "vel_scale") {
                    client_->vel_scale = p.as_double();
                    RCLCPP_INFO(node_->get_logger(),
                        "vel_scale updated to %.3f", client_->vel_scale);
                } else if (p.get_name() == "acc_scale") {
                    client_->acc_scale = p.as_double();
                    RCLCPP_INFO(node_->get_logger(),
                        "acc_scale updated to %.3f", client_->acc_scale);
                }
            }
            return result;
        });

    // Serialize movement commands so they don't interleave
    cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    abs_base_cart_srv_ = node_->create_service<MovePose>(
        "/wmx/moveit2/absolute_base_eef_cartesian",
        std::bind(&MoveIt2ApiNode::onAbsoluteBaseEefCartesian, this,
        std::placeholders::_1, std::placeholders::_2),
        SERVICES_QOS, cb_group_);

    rel_base_cart_srv_ = node_->create_service<MovePose>(
        "/wmx/moveit2/relative_base_eef_cartesian",
        std::bind(&MoveIt2ApiNode::onRelativeBaseEefCartesian, this,
        std::placeholders::_1, std::placeholders::_2),
        SERVICES_QOS, cb_group_);

    rel_tool_cart_srv_ = node_->create_service<MovePose>(
        "/wmx/moveit2/relative_tool_eef_cartesian",
        std::bind(&MoveIt2ApiNode::onRelativeToolEefCartesian, this,
        std::placeholders::_1, std::placeholders::_2),
        SERVICES_QOS, cb_group_);

    abs_base_joint_srv_ = node_->create_service<MovePose>(
        "/wmx/moveit2/absolute_base_eef_joint_movement",
        std::bind(&MoveIt2ApiNode::onAbsoluteBaseEefJointMovement, this,
        std::placeholders::_1, std::placeholders::_2),
        SERVICES_QOS, cb_group_);

    joint_mov_srv_ = node_->create_service<MoveJoints>(
        "/wmx/moveit2/joint_movement",
        std::bind(&MoveIt2ApiNode::onJointMovement, this,
        std::placeholders::_1, std::placeholders::_2),
        SERVICES_QOS, cb_group_);

    rel_joint_mov_srv_ = node_->create_service<MoveJoints>(
        "/wmx/moveit2/relative_joint_movement",
        std::bind(&MoveIt2ApiNode::onRelativeJointMovement, this,
        std::placeholders::_1, std::placeholders::_2),
        SERVICES_QOS, cb_group_);

    get_eef_pose_srv_ = node_->create_service<GetEefPose>(
        "/wmx/moveit2/get_eef_pose",
        std::bind(&MoveIt2ApiNode::onGetEefPose, this,
        std::placeholders::_1, std::placeholders::_2),
        SERVICES_QOS, cb_group_);

    // EEF pose publisher — separate callback group so it doesn't block movements
    pub_cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    eef_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/wmx/moveit2/eef_pose", 10);
    eef_rpy_pub_  = node_->create_publisher<geometry_msgs::msg::Vector3Stamped>(
        "/wmx/moveit2/eef_rpy", 10);

    eef_pose_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(10),  // 100 Hz
        std::bind(&MoveIt2ApiNode::publishEefPose, this),
        pub_cb_group_);

    RCLCPP_INFO(node_->get_logger(), "MoveIt2 API node ready. Services:");
    RCLCPP_INFO(node_->get_logger(),
        "  /wmx/moveit2/absolute_base_eef_cartesian      [MovePose]");
    RCLCPP_INFO(node_->get_logger(),
        "  /wmx/moveit2/relative_base_eef_cartesian      [MovePose]");
    RCLCPP_INFO(node_->get_logger(),
        "  /wmx/moveit2/relative_tool_eef_cartesian      [MovePose]");
    RCLCPP_INFO(node_->get_logger(),
        "  /wmx/moveit2/absolute_base_eef_joint_movement [MovePose]");
    RCLCPP_INFO(node_->get_logger(),
        "  /wmx/moveit2/joint_movement                   [MoveJoints]");
    RCLCPP_INFO(node_->get_logger(),
        "  /wmx/moveit2/relative_joint_movement          [MoveJoints]");
    RCLCPP_INFO(node_->get_logger(),
        "  /wmx/moveit2/get_eef_pose                     [GetEefPose]");
    RCLCPP_INFO(node_->get_logger(),
        "  (gripper: call /wmx/set_gripper directly       [SetBool])");
    RCLCPP_INFO(node_->get_logger(), "Publishers:");
    RCLCPP_INFO(node_->get_logger(),
        "  /wmx/moveit2/eef_pose [PoseStamped]     @ 100 Hz");
    RCLCPP_INFO(node_->get_logger(),
        "  /wmx/moveit2/eef_rpy  [Vector3Stamped]  @ 100 Hz  (x=roll, y=pitch, z=yaw rad)");
}

rclcpp::Node::SharedPtr MoveIt2ApiNode::get_node(){
    return node_;
}

moveit2_client::PoseTarget MoveIt2ApiNode::toPoseTarget(const MovePose::Request::SharedPtr& req){
    moveit2_client::PoseTarget t;
    t.pos = {req->pos[0], req->pos[1], req->pos[2]};
    t.ori = {req->ori[0], req->ori[1], req->ori[2]};
    return t;
}

void MoveIt2ApiNode::onAbsoluteBaseEefCartesian(const MovePose::Request::SharedPtr req,
                                                MovePose::Response::SharedPtr res){
    RCLCPP_INFO(node_->get_logger(),
        "[svc] absolute_base_eef_cartesian pos=[%.3f,%.3f,%.3f] ori=[%.3f,%.3f,%.3f]",
        req->pos[0], req->pos[1], req->pos[2],
        req->ori[0], req->ori[1], req->ori[2]);
    res->success = client_->absoluteBaseEefCartesian(toPoseTarget(req));
    res->message = res->success ? "success" : "failed";
}

void MoveIt2ApiNode::onRelativeBaseEefCartesian(const MovePose::Request::SharedPtr req,
                                                MovePose::Response::SharedPtr res){
    RCLCPP_INFO(node_->get_logger(),
        "[svc] relative_base_eef_cartesian delta pos=[%.3f,%.3f,%.3f] ori=[%.3f,%.3f,%.3f]",
        req->pos[0], req->pos[1], req->pos[2],
        req->ori[0], req->ori[1], req->ori[2]);
    res->success = client_->relativeBaseEefCartesian(toPoseTarget(req));
    res->message = res->success ? "success" : "failed";
}

void MoveIt2ApiNode::onRelativeToolEefCartesian(const MovePose::Request::SharedPtr req,
                                                MovePose::Response::SharedPtr res){
    RCLCPP_INFO(node_->get_logger(),
        "[svc] relative_tool_eef_cartesian delta pos=[%.3f,%.3f,%.3f] ori=[%.3f,%.3f,%.3f]",
        req->pos[0], req->pos[1], req->pos[2],
        req->ori[0], req->ori[1], req->ori[2]);
    res->success = client_->relativeToolEefCartesian(toPoseTarget(req));
    res->message = res->success ? "success" : "failed";
}

void MoveIt2ApiNode::onAbsoluteBaseEefJointMovement(const MovePose::Request::SharedPtr req,
                                                    MovePose::Response::SharedPtr res){
    RCLCPP_INFO(node_->get_logger(),
        "[svc] absolute_base_eef_joint_movement pos=[%.3f,%.3f,%.3f] ori=[%.3f,%.3f,%.3f]",
        req->pos[0], req->pos[1], req->pos[2],
        req->ori[0], req->ori[1], req->ori[2]);
    res->success = client_->absoluteBaseEefJointMovement(toPoseTarget(req));
    res->message = res->success ? "success" : "failed";
}

void MoveIt2ApiNode::onJointMovement(const MoveJoints::Request::SharedPtr req,
                                    MoveJoints::Response::SharedPtr res){
    if (req->joint_names.size() != req->joint_values.size()) {
        res->success = false;
        res->message = "joint_names and joint_values size mismatch";
        RCLCPP_ERROR(node_->get_logger(),
            "[svc] joint_movement: %s", res->message.c_str());
        return;
    }

    std::map<std::string, double> joints;
    for (size_t i = 0; i < req->joint_names.size(); ++i) {
        joints[req->joint_names[i]] = req->joint_values[i];
    }

    RCLCPP_INFO(node_->get_logger(), "[svc] joint_movement: %zu joints", joints.size());
    res->success = client_->jointMovement(joints);
    res->message = res->success ? "success" : "failed";
}

void MoveIt2ApiNode::onRelativeJointMovement(const MoveJoints::Request::SharedPtr req,
                                             MoveJoints::Response::SharedPtr res){
    if (req->joint_names.size() != req->joint_values.size()) {
        res->success = false;
        res->message = "joint_names and joint_values size mismatch";
        RCLCPP_ERROR(node_->get_logger(),
            "[svc] relative_joint_movement: %s", res->message.c_str());
        return;
    }

    std::map<std::string, double> deltas;
    for (size_t i = 0; i < req->joint_names.size(); ++i) {
        deltas[req->joint_names[i]] = req->joint_values[i];
    }

    RCLCPP_INFO(node_->get_logger(),
        "[svc] relative_joint_movement: %zu joints", deltas.size());
    res->success = client_->relativeJointMovement(deltas);
    res->message = res->success ? "success" : "failed";
}

void MoveIt2ApiNode::onGetEefPose(const GetEefPose::Request::SharedPtr /*req*/,
                                  GetEefPose::Response::SharedPtr res){
    auto result = client_->getCurrentEefPose();
    if (!result) {
        res->success = false;
        res->message = "failed to get current EEF pose";
        return;
    }
    res->pos = {result->x, result->y, result->z};
    res->rpy = {result->roll, result->pitch, result->yaw};
    res->success = true;
    res->message = "success";
}

void MoveIt2ApiNode::publishEefPose(){
    auto result = client_->getCurrentEefPose();
    if (!result) {
        return;
    }

    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp       = node_->now();
    msg.header.frame_id    = client_->base_name;
    msg.pose.position.x    = result->x;
    msg.pose.position.y    = result->y;
    msg.pose.position.z    = result->z;
    msg.pose.orientation.x = result->qx;
    msg.pose.orientation.y = result->qy;
    msg.pose.orientation.z = result->qz;
    msg.pose.orientation.w = result->qw;

    eef_pose_pub_->publish(msg);

    tf2::Quaternion q(result->qx, result->qy, result->qz, result->qw);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    geometry_msgs::msg::Vector3Stamped rpy_msg;
    rpy_msg.header = msg.header;
    rpy_msg.vector.x = roll;
    rpy_msg.vector.y = pitch;
    rpy_msg.vector.z = yaw;
    eef_rpy_pub_->publish(rpy_msg);
}

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);

    MoveIt2ApiNode api_node;

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(api_node.get_node());

    executor.spin();

    rclcpp::shutdown();
    return 0;
}
