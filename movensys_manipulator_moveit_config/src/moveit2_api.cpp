// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#include "moveit2_api.hpp"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <Eigen/Geometry>

#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

using namespace std::chrono_literals;

namespace {
// moveit_msgs/srv/ServoCommandType: 0=JOINT_JOG, 1=TWIST, 2=POSE.
constexpr int8_t CMD_POSE = 2;
}  // namespace

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

    // Servo POSE bridge — own callback group so a blocked service call here
    // cannot stall the movement services or the EEF publisher.
    servo_cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions servo_sub_opts;
    servo_sub_opts.callback_group = servo_cb_group_;

    pose_target_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/servo_node/pose_target_cmds", 10);

#ifdef HAVE_SERVO_COMMAND_TYPE
    servo_switch_client_ = node_->create_client<moveit_msgs::srv::ServoCommandType>(
        "/servo_node/switch_command_type", SERVICES_QOS, servo_cb_group_);
#endif

    tool_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/wmx/servo_node/tool_pose", 10,
        std::bind(&MoveIt2ApiNode::onToolPose, this, std::placeholders::_1),
        servo_sub_opts);

    exec_active_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        "/moveit2_trajectory/execution_active", rclcpp::QoS(1).transient_local(),
        std::bind(&MoveIt2ApiNode::onExecActive, this, std::placeholders::_1),
        servo_sub_opts);

    // Servo halts on incoming_command_timeout (0.1 s), so the target has to be
    // restreamed rather than published once.
    pose_target_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(20),  // 50 Hz
        std::bind(&MoveIt2ApiNode::publishPoseTarget, this),
        servo_cb_group_);

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
    RCLCPP_INFO(node_->get_logger(), "Subscribers:");
    RCLCPP_INFO(node_->get_logger(),
        "  /wmx/servo_node/tool_pose [PoseStamped] tool-frame target -> Servo POSE mode");
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

void MoveIt2ApiNode::switchServoCommandType(int8_t type){
#ifdef HAVE_SERVO_COMMAND_TYPE
    if (!servo_switch_client_->wait_for_service(1s)) {
        RCLCPP_WARN(node_->get_logger(),
            "/servo_node/switch_command_type not available — is servo_node running?");
        return;
    }
    auto req = std::make_shared<moveit_msgs::srv::ServoCommandType::Request>();
    req->command_type = type;
    servo_switch_client_->async_send_request(req);
    RCLCPP_INFO(node_->get_logger(), "[servo] switched command type to POSE");
#else
    (void)type;
    RCLCPP_WARN(node_->get_logger(),
        "ServoCommandType unavailable on this distro — switch Servo to POSE mode manually");
#endif
}

// Anchors the streamed target on the arm's current pose, so nothing moves until
// a tool_pose command arrives.
bool MoveIt2ApiNode::seedPoseTarget(){
    auto current = client_->getCurrentEefPose();
    if (!current) {
        RCLCPP_WARN(node_->get_logger(), "Could not read current EEF pose to seed POSE target");
        return false;
    }

    std::lock_guard<std::mutex> lock(pose_target_mtx_);
    pose_target_.header.frame_id    = client_->base_name;
    pose_target_.pose.position.x    = current->x;
    pose_target_.pose.position.y    = current->y;
    pose_target_.pose.position.z    = current->z;
    pose_target_.pose.orientation.x = current->qx;
    pose_target_.pose.orientation.y = current->qy;
    pose_target_.pose.orientation.z = current->qz;
    pose_target_.pose.orientation.w = current->qw;
    pose_target_valid_ = true;
    return true;
}

// The incoming pose is a target expressed in the tool frame — the offset from
// where the EEF is now. Servo's pose_target_cmds is base-frame only, so compose
// T_base_target = T_base_eef * T_eef_target, matching relativeToolEefCartesian.
void MoveIt2ApiNode::onToolPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
    auto current = client_->getCurrentEefPose();
    if (!current) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
            "Dropped tool_pose: current EEF pose unavailable");
        return;
    }

    Eigen::Isometry3d T_base_eef = Eigen::Isometry3d::Identity();
    T_base_eef.translation() = Eigen::Vector3d(current->x, current->y, current->z);
    T_base_eef.linear() =
        Eigen::Quaterniond(current->qw, current->qx, current->qy, current->qz)
            .normalized().toRotationMatrix();

    Eigen::Quaterniond q_delta(
        msg->pose.orientation.w, msg->pose.orientation.x,
        msg->pose.orientation.y, msg->pose.orientation.z);
    if (q_delta.norm() < 1e-9) {
        // An all-zero quaternion means "no rotation" rather than an invalid one.
        q_delta = Eigen::Quaterniond::Identity();
    }
    q_delta.normalize();

    Eigen::Isometry3d T_eef_target = Eigen::Isometry3d::Identity();
    T_eef_target.translation() =
        Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    T_eef_target.linear() = q_delta.toRotationMatrix();

    const Eigen::Isometry3d T_base_target = T_base_eef * T_eef_target;
    const Eigen::Vector3d pos = T_base_target.translation();
    const Eigen::Quaterniond ori(T_base_target.rotation());

    const bool first_command = !pose_target_valid_.exchange(true);
    {
        std::lock_guard<std::mutex> lock(pose_target_mtx_);
        pose_target_.header.frame_id    = client_->base_name;
        pose_target_.pose.position.x    = pos.x();
        pose_target_.pose.position.y    = pos.y();
        pose_target_.pose.position.z    = pos.z();
        pose_target_.pose.orientation.x = ori.x();
        pose_target_.pose.orientation.y = ori.y();
        pose_target_.pose.orientation.z = ori.z();
        pose_target_.pose.orientation.w = ori.w();
    }

    RCLCPP_INFO(node_->get_logger(),
        "[servo] tool_pose delta=[%.4f,%.4f,%.4f] -> %s target=[%.4f,%.4f,%.4f]",
        msg->pose.position.x, msg->pose.position.y, msg->pose.position.z,
        client_->base_name.c_str(), pos.x(), pos.y(), pos.z());

    if (first_command) {
        switchServoCommandType(CMD_POSE);
    }
}

void MoveIt2ApiNode::onExecActive(const std_msgs::msg::Bool::SharedPtr msg){
    const bool was_active = exec_active_.exchange(msg->data);
    if (was_active == msg->data) {
        return;
    }

    RCLCPP_INFO(node_->get_logger(), "[servo] POSE streaming %s (move_group execution %s)",
        msg->data ? "paused" : "resumed", msg->data ? "started" : "finished");

    if (!msg->data && pose_target_valid_.load()) {
        seedPoseTarget();
    }
}

void MoveIt2ApiNode::publishPoseTarget(){
    if (exec_active_.load() || !pose_target_valid_.load()) {
        return;
    }

    geometry_msgs::msg::PoseStamped msg;
    {
        std::lock_guard<std::mutex> lock(pose_target_mtx_);
        msg = pose_target_;
    }
    msg.header.stamp = node_->now();
    pose_target_pub_->publish(msg);
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
