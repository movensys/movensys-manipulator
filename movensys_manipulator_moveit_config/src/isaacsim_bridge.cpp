// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#include <atomic>
#include <chrono>
#include <functional>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "control_msgs/msg/joint_jog.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

using FollowJT = control_msgs::action::FollowJointTrajectory;
using GoalHandleFJT = rclcpp_action::ServerGoalHandle<FollowJT>;
using namespace std::chrono_literals;

// Runs its callback when the enclosing scope exits (normal return or exception),
// so servo is always re-enabled even if trajectory execution bails out early.
struct ScopeExit {
    std::function<void()> fn;
    ~ScopeExit() { fn(); }
};

class IsaacSimBridge : public rclcpp::Node {
public:
    std::vector<std::string> joint_names_;
    std::vector<std::string> gripper_joint_names_;

    double gripper_state_ = 0.0;
    double gripper_open_  = 0.0;
    double gripper_close_ = 0.0;
    sensor_msgs::msg::JointState last_joint_state_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr    pub_joint_state_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_state_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr            set_gripper_srv_;
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr sub_servo_;

    IsaacSimBridge();

private:
    rclcpp_action::Server<FollowJT>::SharedPtr action_server_;

    // Servo is rejected while a move_group trajectory executes, accepted otherwise.
    std::atomic<bool> in_execution_{false};
    rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr pub_servo_reset_;

    // Broadcasts execution state so keyboard_teleop can pause/re-anchor POSE mode.
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_exec_active_;

    void cbJointStates(const sensor_msgs::msg::JointState::SharedPtr msg);
    void cbServoCommand(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);
    void resetServo();
    void publishExecActive(bool active);

    void setGripper(const std::shared_ptr<std_srvs::srv::SetBool::Request>  request,
                          std::shared_ptr<std_srvs::srv::SetBool::Response> response);

    rclcpp_action::GoalResponse   handle_goal(const rclcpp_action::GoalUUID&,
                                              std::shared_ptr<const FollowJT::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleFJT> goal_handle);
    void                          handle_accepted(const std::shared_ptr<GoalHandleFJT> goal_handle);
    void                          execute(const std::shared_ptr<GoalHandleFJT> goal_handle);
};


IsaacSimBridge::IsaacSimBridge() : Node("isaacsim_bridge")
{
    // Declare parameters
    this->declare_parameter("joint_names",
        std::vector<std::string>{"j1", "j2", "j3", "j4", "j5", "j6"});
    this->declare_parameter("gripper_joint_names", std::vector<std::string>{});
    this->declare_parameter("gripper_open",  0.000);
    this->declare_parameter("gripper_close", 0.000);
    this->declare_parameter("joint_command_topic", "/joint_command_topic/no_topic");
    this->declare_parameter("joint_states_topic",  "/joint_states_topic/no_topic");
    this->declare_parameter("set_gripper_service", "/set_gripper_service/no_topic");
    this->declare_parameter("action_name", "/action_name/no_action");
    this->declare_parameter("servo_command_topic", "/servo_command_topic/no_topic");

    // Fetch parameters
    joint_names_         = this->get_parameter("joint_names").as_string_array();
    gripper_joint_names_ = this->get_parameter("gripper_joint_names").as_string_array();
    gripper_open_        = this->get_parameter("gripper_open").as_double();
    gripper_close_       = this->get_parameter("gripper_close").as_double();
    const auto joint_command_topic = this->get_parameter("joint_command_topic").as_string();
    const auto joint_states_topic  = this->get_parameter("joint_states_topic").as_string();
    const auto set_gripper_service = this->get_parameter("set_gripper_service").as_string();
    const auto action_name         = this->get_parameter("action_name").as_string();
    const auto servo_command_topic = this->get_parameter("servo_command_topic").as_string();

    // Create interfaces
    pub_joint_state_ = this->create_publisher<sensor_msgs::msg::JointState>(
        joint_command_topic, 10);
    sub_joint_state_ = this->create_subscription<sensor_msgs::msg::JointState>(
        joint_states_topic, 10,
        std::bind(&IsaacSimBridge::cbJointStates, this, std::placeholders::_1));

    set_gripper_srv_ = this->create_service<std_srvs::srv::SetBool>(
        set_gripper_service,
        std::bind(&IsaacSimBridge::setGripper, this,
                  std::placeholders::_1, std::placeholders::_2));

    action_server_ = rclcpp_action::create_server<FollowJT>(this,
        action_name,
        std::bind(&IsaacSimBridge::handle_goal,     this,
                  std::placeholders::_1, std::placeholders::_2),
        std::bind(&IsaacSimBridge::handle_cancel,   this, std::placeholders::_1),
        std::bind(&IsaacSimBridge::handle_accepted, this, std::placeholders::_1));

    sub_servo_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
        servo_command_topic, 10,
        std::bind(&IsaacSimBridge::cbServoCommand, this, std::placeholders::_1));

    pub_servo_reset_ = this->create_publisher<control_msgs::msg::JointJog>(
        "/servo_node/delta_joint_cmds", 10);

    // Latched so a keyboard_teleop that starts after a trajectory still sees state.
    pub_exec_active_ = this->create_publisher<std_msgs::msg::Bool>(
        "/moveit2_trajectory/execution_active", rclcpp::QoS(1).transient_local());
    publishExecActive(false);

    RCLCPP_INFO(this->get_logger(), "isaacsim_bridge is ready");
}

void IsaacSimBridge::cbJointStates(const sensor_msgs::msg::JointState::SharedPtr msg){
    last_joint_state_ = *msg;
}

void IsaacSimBridge::cbServoCommand(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg){
    if (msg->points.empty() || in_execution_.load()) {
        return;  // reject servo while a move_group plan is executing
    }
    const auto& pt = msg->points.back();

    sensor_msgs::msg::JointState joint_command;
    joint_command.name = joint_names_;
    joint_command.name.insert(joint_command.name.end(),
        gripper_joint_names_.begin(), gripper_joint_names_.end());

    std::vector<double> pos = pt.positions;
    std::vector<double> vel = pt.velocities;
    const size_t total_joints = joint_names_.size() + gripper_joint_names_.size();
    pos.resize(total_joints, gripper_state_);
    vel.resize(total_joints, 0.0);

    joint_command.position     = std::move(pos);
    joint_command.velocity     = std::move(vel);
    joint_command.header.stamp = this->get_clock()->now();
    pub_joint_state_->publish(joint_command);
}

void IsaacSimBridge::resetServo(){
    control_msgs::msg::JointJog jog;
    jog.header.stamp = this->get_clock()->now();
    jog.joint_names = joint_names_;
    jog.velocities.assign(joint_names_.size(), 0.0);
    pub_servo_reset_->publish(jog);
}

void IsaacSimBridge::publishExecActive(bool active){
    std_msgs::msg::Bool msg;
    msg.data = active;
    pub_exec_active_->publish(msg);
}

void IsaacSimBridge::setGripper(const std::shared_ptr<std_srvs::srv::SetBool::Request>  request,
                                       std::shared_ptr<std_srvs::srv::SetBool::Response> response){
    gripper_state_     = request->data ? gripper_close_ : gripper_open_;
    response->success  = true;

    sensor_msgs::msg::JointState joint_command = last_joint_state_;

    const size_t gripper_joint_start_index =
        joint_command.position.size() - gripper_joint_names_.size();
    for (size_t i = 0; i < gripper_joint_names_.size(); ++i) {
        joint_command.position[gripper_joint_start_index + i] = gripper_state_;
    }

    joint_command.header.stamp   = this->get_clock()->now();
    pub_joint_state_->publish(joint_command);

    RCLCPP_INFO(this->get_logger(), "Gripper: %s", request->data ? "close" : "open");
}

rclcpp_action::GoalResponse IsaacSimBridge::handle_goal(
    const rclcpp_action::GoalUUID&,
    std::shared_ptr<const FollowJT::Goal> goal)
{
    (void)goal;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse IsaacSimBridge::handle_cancel(
    const std::shared_ptr<GoalHandleFJT> /*goal_handle*/)
{
    return rclcpp_action::CancelResponse::ACCEPT;
}

void IsaacSimBridge::handle_accepted(const std::shared_ptr<GoalHandleFJT> goal_handle)
{
    std::thread(&IsaacSimBridge::execute, this, goal_handle).detach();
}

void IsaacSimBridge::execute(const std::shared_ptr<GoalHandleFJT> goal_handle)
{
    in_execution_ = true;   // reject servo while this plan executes
    publishExecActive(true);
    // Always re-enable servo on exit (completion, early return, or exception),
    // and re-anchor POSE mode to wherever this trajectory leaves the arm.
    ScopeExit on_exit{[this]() {
        in_execution_ = false;
        publishExecActive(false);
        resetServo();
    }};
    const auto& traj = goal_handle->get_goal()->trajectory;

    RCLCPP_INFO(this->get_logger(),
                "Received trajectory goal with %zu points", traj.points.size());

    // Log joint names
    std::ostringstream jn;
    for (size_t i = 0; i < traj.joint_names.size(); ++i) {
        if (i) {
            jn << ", ";
        }
        jn << traj.joint_names[i];
    }
    RCLCPP_INFO(this->get_logger(), "Joint names: [%s]", jn.str().c_str());

    // Log trajectory points
    for (size_t i = 0; i < traj.points.size(); ++i) {
        const auto& pt = traj.points[i];
        std::ostringstream pos, vel, acc;
        for (size_t k = 0; k < pt.positions.size();     ++k) {
            if (k) { pos << ", "; }
            pos << pt.positions[k];
        }
        for (size_t k = 0; k < pt.velocities.size();    ++k) {
            if (k) { vel << ", "; }
            vel << pt.velocities[k];
        }
        for (size_t k = 0; k < pt.accelerations.size(); ++k) {
            if (k) { acc << ", "; }
            acc << pt.accelerations[k];
        }
        RCLCPP_INFO(this->get_logger(),
                    "Point %zu: pos=[%s] vel=[%s] acc=[%s] t=%ds %uns",
                    i, pos.str().c_str(), vel.str().c_str(), acc.str().c_str(),
                    pt.time_from_start.sec, pt.time_from_start.nanosec);

        if (i != 0) {
            rclcpp::Duration dt = rclcpp::Duration(traj.points[i].time_from_start) -
                                  rclcpp::Duration(traj.points[i-1].time_from_start);
            RCLCPP_INFO(this->get_logger(), "Time interval: %.4f s", dt.seconds());
        }
    }

    // Execute trajectory
    for (size_t i = 0; i < traj.points.size(); ++i) {
        const auto& pt = traj.points[i];

        sensor_msgs::msg::JointState joint_command;
        joint_command.name = joint_names_;
        joint_command.name.insert(joint_command.name.end(),
            gripper_joint_names_.begin(), gripper_joint_names_.end());

        std::vector<double> pos = pt.positions;
        std::vector<double> vel = pt.velocities;

        const size_t total_joints = joint_names_.size() + gripper_joint_names_.size();
        pos.resize(total_joints, gripper_state_);
        vel.resize(total_joints, 0.0);

        joint_command.position     = std::move(pos);
        joint_command.velocity     = std::move(vel);
        joint_command.header.stamp = this->get_clock()->now();
        pub_joint_state_->publish(joint_command);

        if (i + 1 < traj.points.size()) {
            rclcpp::Duration dt = rclcpp::Duration(traj.points[i+1].time_from_start) -
                                  rclcpp::Duration(traj.points[i].time_from_start);
            std::this_thread::sleep_for(std::chrono::nanoseconds(dt.nanoseconds()));
        }
    }

    auto result       = std::make_shared<FollowJT::Result>();
    result->error_code = 0;
    goal_handle->succeed(result);
    // on_exit re-enables servo and signals execution end (see ScopeExit above).
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IsaacSimBridge>());
    rclcpp::shutdown();
    return 0;
}
