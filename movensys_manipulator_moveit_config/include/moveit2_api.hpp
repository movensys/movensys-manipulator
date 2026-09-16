// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#ifndef MOVEIT2_API_HPP_
#define MOVEIT2_API_HPP_

#include <atomic>
#include <map>
#include <memory>
#include <mutex>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <movensys_manipulator_moveit_config/srv/get_eef_pose.hpp>
#include <movensys_manipulator_moveit_config/srv/move_joints.hpp>
#include <movensys_manipulator_moveit_config/srv/move_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

// ServoCommandType only exists on Jazzy and newer. Without it the tool_pose
// bridge still streams, it just cannot switch Servo into POSE mode itself.
#if __has_include(<moveit_msgs/srv/servo_command_type.hpp>)
    #define HAVE_SERVO_COMMAND_TYPE
    #include <moveit_msgs/srv/servo_command_type.hpp>
#endif

#include "moveit2_client.hpp"

using MovePose   = movensys_manipulator_moveit_config::srv::MovePose;
using MoveJoints = movensys_manipulator_moveit_config::srv::MoveJoints;
using GetEefPose = movensys_manipulator_moveit_config::srv::GetEefPose;

class MoveIt2ApiNode{
public:
    MoveIt2ApiNode();
    rclcpp::Node::SharedPtr get_node();

private:
    static moveit2_client::PoseTarget toPoseTarget(const MovePose::Request::SharedPtr& req);

    void onAbsoluteBaseEefCartesian(const MovePose::Request::SharedPtr req,
                                    MovePose::Response::SharedPtr res);

    void onRelativeBaseEefCartesian(const MovePose::Request::SharedPtr req,
                                    MovePose::Response::SharedPtr res);

    void onRelativeToolEefCartesian(const MovePose::Request::SharedPtr req,
                                    MovePose::Response::SharedPtr res);

    void onAbsoluteBaseEefJointMovement(const MovePose::Request::SharedPtr req,
                                        MovePose::Response::SharedPtr res);

    void onJointMovement(const MoveJoints::Request::SharedPtr req,
                        MoveJoints::Response::SharedPtr res);

    void onRelativeJointMovement(const MoveJoints::Request::SharedPtr req,
                                 MoveJoints::Response::SharedPtr res);

    void onGetEefPose(const GetEefPose::Request::SharedPtr req,
                      GetEefPose::Response::SharedPtr res);

    void publishEefPose();

    void onToolPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void onExecActive(const std_msgs::msg::Bool::SharedPtr msg);
    void publishPoseTarget();
    bool seedPoseTarget();
    void switchServoCommandType(int8_t type);

    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<moveit2_client::MoveIt2Client> client_;
    rclcpp::CallbackGroup::SharedPtr cb_group_;
    rclcpp::CallbackGroup::SharedPtr pub_cb_group_;

    rclcpp::Service<MovePose>::SharedPtr                             abs_base_cart_srv_;
    rclcpp::Service<MovePose>::SharedPtr                             rel_base_cart_srv_;
    rclcpp::Service<MovePose>::SharedPtr                             rel_tool_cart_srv_;
    rclcpp::Service<MovePose>::SharedPtr                             abs_base_joint_srv_;
    rclcpp::Service<MoveJoints>::SharedPtr                           joint_mov_srv_;
    rclcpp::Service<MoveJoints>::SharedPtr                           rel_joint_mov_srv_;
    rclcpp::Service<GetEefPose>::SharedPtr                           get_eef_pose_srv_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr    eef_pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr eef_rpy_pub_;
    rclcpp::TimerBase::SharedPtr                                     eef_pose_timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr tool_pose_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr             exec_active_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr    pose_target_pub_;
#ifdef HAVE_SERVO_COMMAND_TYPE
    rclcpp::Client<moveit_msgs::srv::ServoCommandType>::SharedPtr    servo_switch_client_;
#endif
    rclcpp::TimerBase::SharedPtr                                     pose_target_timer_;
    rclcpp::CallbackGroup::SharedPtr                                 servo_cb_group_;

    std::mutex                              pose_target_mtx_;   // guards pose_target_
    geometry_msgs::msg::PoseStamped         pose_target_;
    std::atomic<bool>                       pose_target_valid_{false};
    std::atomic<bool>                       exec_active_{false};
};

#endif  // MOVEIT2_API_HPP_
