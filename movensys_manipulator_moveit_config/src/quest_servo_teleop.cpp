// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.
//
// quest_servo_teleop
//
// Teleoperate the arm with a Meta Quest RIGHT controller through MoveIt 2 Servo,
// using Servo's POSE command type with *relative clutch* mapping.
//
// Input  (from movensys-teleoperation / quest_pose_publisher):
//   <quest_pose_topic>  geometry_msgs/PoseStamped  controller pose (frame quest_world)
//   <enable_topic>      sensor_msgs/Joy            buttons/triggers; grip = clutch
//       axes    = [thumbstick_x, thumbstick_y, trigger_value, squeeze_value]
//       buttons = [primary_click, secondary_click, thumbstick_click, menu_click]
//
// Output (to moveit_servo::ServoNode):
//   /servo_node/pose_target_cmds       geometry_msgs/PoseStamped  target EEF pose
//   /servo_node/switch_command_type    moveit_msgs/srv/ServoCommandType (POSE=2)
//
// Clutch model (deadman): motion only while the grip is held. On the press edge
// we latch the controller pose (c0,qc0) and the current EEF pose (r0,qr0, from
// TF). While held, the EEF target follows the controller *delta* since the latch,
// rotated from the operator frame into the robot base frame and scaled. On
// release we stop streaming so Servo halts (incoming_command_timeout), and the
// next press re-anchors -- so the operator can recenter their hand without moving
// the robot (the "mouse-lift" clutch).

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/srv/servo_command_type.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/bool.hpp>

using namespace std::chrono_literals;

namespace {
// moveit_msgs/srv/ServoCommandType: 0=JOINT_JOG, 1=TWIST, 2=POSE.
constexpr int8_t CMD_POSE = 2;
}  // namespace

class QuestServoTeleop {
public:
    explicit QuestServoTeleop(const rclcpp::Node::SharedPtr& node) : node_(node) {
        // --- parameters ---------------------------------------------------
        quest_pose_topic_ = declare<std::string>("quest_pose_topic",
                                                  "/quest_pose_publisher/controller_pose_right");
        enable_topic_     = declare<std::string>("enable_topic",
                                                 "/quest_pose_publisher/joy_right");
        pose_target_topic_ = declare<std::string>("pose_target_topic",
                                                  "/servo_node/pose_target_cmds");
        switch_service_   = declare<std::string>("switch_service",
                                                 "/servo_node/switch_command_type");
        base_frame_       = declare<std::string>("base_frame", "world_manipulator");
        eef_frame_        = declare<std::string>("eef_frame", "Link6");

        // Clutch: use analog grip axis by default; set enable_button_index >= 0 to
        // use a digital button instead.
        enable_axis_index_   = declare<int>("enable_axis_index", 3);       // squeeze_value
        enable_axis_threshold_ = declare<double>("enable_axis_threshold", 0.6);
        enable_button_index_ = declare<int>("enable_button_index", -1);    // -1 = use axis

        position_scale_    = declare<double>("position_scale", 0.5);
        orientation_scale_ = declare<double>("orientation_scale", 1.0);    // 0..1 (1 = 1:1)
        align_yaw_deg_     = declare<double>("align_yaw_deg", 0.0);
        max_target_step_   = declare<double>("max_target_step", 0.05);     // m per publish
        double stream_hz   = declare<double>("stream_rate_hz", 50.0);

        // Operator->robot frame alignment (yaw about base +Z).
        r_align_.setRPY(0.0, 0.0, align_yaw_deg_ * M_PI / 180.0);
        r_align_.normalize();

        // --- ROS entities -------------------------------------------------
        target_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
            pose_target_topic_, 10);
        switch_client_ = node_->create_client<moveit_msgs::srv::ServoCommandType>(switch_service_);

        pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
            quest_pose_topic_, rclcpp::SensorDataQoS(),
            std::bind(&QuestServoTeleop::onQuestPose, this, std::placeholders::_1));
        joy_sub_ = node_->create_subscription<sensor_msgs::msg::Joy>(
            enable_topic_, rclcpp::SensorDataQoS(),
            std::bind(&QuestServoTeleop::onJoy, this, std::placeholders::_1));

        // sim_bridge publishes this while a move_group plan executes, on both the
        // Isaac Sim and Gazebo paths.
        exec_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
            "/moveit2_trajectory/execution_active", rclcpp::QoS(1).transient_local(),
            std::bind(&QuestServoTeleop::onExecActive, this, std::placeholders::_1));

        tf_buffer_   = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, stream_hz));
        stream_timer_ = node_->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&QuestServoTeleop::streamTarget, this));

        // Put Servo into POSE mode (retry until the service is up).
        switch_timer_ = node_->create_wall_timer(
            1s, std::bind(&QuestServoTeleop::ensurePoseMode, this));

        RCLCPP_INFO(node_->get_logger(),
                    "quest_servo_teleop: pose<-%s enable<-%s target->%s | base=%s eef=%s "
                    "pos_scale=%.2f align_yaw=%.1fdeg",
                    quest_pose_topic_.c_str(), enable_topic_.c_str(), pose_target_topic_.c_str(),
                    base_frame_.c_str(), eef_frame_.c_str(), position_scale_, align_yaw_deg_);
    }

private:
    template <typename T>
    T declare(const std::string& name, const T& def) {
        return node_->declare_parameter<T>(name, def);
    }

    // --- Servo command-type management -----------------------------------
    void ensurePoseMode() {
        if (pose_mode_set_.load()) {
            switch_timer_->cancel();
            return;
        }
        if (!switch_client_->service_is_ready()) {
            return;  // try again next tick
        }
        auto req = std::make_shared<moveit_msgs::srv::ServoCommandType::Request>();
        req->command_type = CMD_POSE;
        switch_client_->async_send_request(
            req, [this](rclcpp::Client<moveit_msgs::srv::ServoCommandType>::SharedFuture fut) {
                if (fut.get()->success) {
                    pose_mode_set_.store(true);
                    RCLCPP_INFO(node_->get_logger(), "Servo switched to POSE mode.");
                } else {
                    RCLCPP_WARN(node_->get_logger(), "Servo rejected POSE switch; retrying.");
                }
            });
    }

    // --- subscriptions ----------------------------------------------------
    void onQuestPose(geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mtx_);
        c_now_.setValue(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
        qc_now_.setValue(msg->pose.orientation.x, msg->pose.orientation.y,
                         msg->pose.orientation.z, msg->pose.orientation.w);
        have_pose_ = true;
    }

    void onJoy(sensor_msgs::msg::Joy::SharedPtr msg) {
        bool pressed = false;
        if (enable_button_index_ >= 0) {
            if (static_cast<size_t>(enable_button_index_) < msg->buttons.size()) {
                pressed = msg->buttons[enable_button_index_] != 0;
            }
        } else if (static_cast<size_t>(enable_axis_index_) < msg->axes.size()) {
            pressed = msg->axes[enable_axis_index_] >= enable_axis_threshold_;
        }
        if (pressed && !engaged_.load()) {
            engage();
        } else if (!pressed && engaged_.load()) {
            disengage();
        }
    }

    void onExecActive(std_msgs::msg::Bool::SharedPtr msg) {
        const bool was = exec_active_.exchange(msg->data);
        if (was && !msg->data && engaged_.load()) {
            // Trajectory finished while clutch still held: re-anchor to avoid a jump.
            reanchor();
        }
    }

    // --- clutch state machine --------------------------------------------
    void engage() {
        tf2::Vector3 r0;
        tf2::Quaternion qr0;
        if (!lookupEef(r0, qr0)) {
            RCLCPP_WARN(node_->get_logger(), "Clutch engage aborted: no %s->%s TF yet.",
                        base_frame_.c_str(), eef_frame_.c_str());
            return;
        }
        std::lock_guard<std::mutex> lock(mtx_);
        if (!have_pose_) {
            RCLCPP_WARN(node_->get_logger(), "Clutch engage aborted: no controller pose yet.");
            return;
        }
        c0_ = c_now_;
        qc0_ = qc_now_;
        r0_ = r0;
        qr0_ = qr0;
        target_p_ = r0;      // first command holds the current pose
        target_q_ = qr0;
        engaged_.store(true);
        RCLCPP_INFO(node_->get_logger(), "Clutch ENGAGED (anchor EEF [%.3f, %.3f, %.3f]).",
                    r0.x(), r0.y(), r0.z());
    }

    void disengage() {
        engaged_.store(false);
        RCLCPP_INFO(node_->get_logger(), "Clutch RELEASED (Servo will halt on timeout).");
    }

    // Re-latch anchors to the current controller + current EEF without dropping
    // the clutch (used after a move_group trajectory shifts the arm).
    void reanchor() {
        tf2::Vector3 r0;
        tf2::Quaternion qr0;
        if (!lookupEef(r0, qr0)) {
            return;
        }
        std::lock_guard<std::mutex> lock(mtx_);
        c0_ = c_now_;
        qc0_ = qc_now_;
        r0_ = r0;
        qr0_ = qr0;
        target_p_ = r0;
        target_q_ = qr0;
    }

    bool lookupEef(tf2::Vector3& p, tf2::Quaternion& q) {
        for (int i = 0; i < 20 && rclcpp::ok(); ++i) {
            try {
                auto tf = tf_buffer_->lookupTransform(base_frame_, eef_frame_, tf2::TimePointZero);
                p.setValue(tf.transform.translation.x, tf.transform.translation.y,
                           tf.transform.translation.z);
                q.setValue(tf.transform.rotation.x, tf.transform.rotation.y,
                           tf.transform.rotation.z, tf.transform.rotation.w);
                return true;
            } catch (const tf2::TransformException&) {
                std::this_thread::sleep_for(50ms);
            }
        }
        return false;
    }

    // --- streaming --------------------------------------------------------
    void streamTarget() {
        if (!engaged_.load() || exec_active_.load()) {
            return;
        }
        geometry_msgs::msg::PoseStamped out;
        {
            std::lock_guard<std::mutex> lock(mtx_);
            if (!have_pose_) {
                return;
            }
            // Position: scaled controller delta, rotated into the base frame.
            tf2::Vector3 dp_quest = c_now_ - c0_;
            tf2::Vector3 dp_robot = tf2::quatRotate(r_align_, dp_quest) * position_scale_;
            tf2::Vector3 desired = r0_ + dp_robot;

            // Clamp per-cycle target slew (rejects controller glitches/dropouts).
            tf2::Vector3 step = desired - target_p_;
            const double n = step.length();
            if (n > max_target_step_) {
                step *= (max_target_step_ / n);
            }
            target_p_ = target_p_ + step;

            // Orientation: controller rotation since anchor, expressed in the base
            // frame, optionally damped, then applied to the anchored EEF orientation.
            tf2::Quaternion dq_quest = qc_now_ * qc0_.inverse();
            tf2::Quaternion dq_robot = r_align_ * dq_quest * r_align_.inverse();
            dq_robot.normalize();
            if (orientation_scale_ < 0.999) {
                dq_robot = tf2::Quaternion::getIdentity().slerp(dq_robot, orientation_scale_);
                dq_robot.normalize();
            }
            target_q_ = (dq_robot * qr0_);
            target_q_.normalize();

            out.pose.position.x = target_p_.x();
            out.pose.position.y = target_p_.y();
            out.pose.position.z = target_p_.z();
            out.pose.orientation.x = target_q_.x();
            out.pose.orientation.y = target_q_.y();
            out.pose.orientation.z = target_q_.z();
            out.pose.orientation.w = target_q_.w();
        }
        out.header.stamp = node_->now();
        out.header.frame_id = base_frame_;
        target_pub_->publish(out);
    }

    // --- members ----------------------------------------------------------
    rclcpp::Node::SharedPtr node_;

    std::string quest_pose_topic_, enable_topic_, pose_target_topic_, switch_service_;
    std::string base_frame_, eef_frame_;
    int enable_axis_index_, enable_button_index_;
    double enable_axis_threshold_;
    double position_scale_, orientation_scale_, align_yaw_deg_, max_target_step_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pub_;
    rclcpp::Client<moveit_msgs::srv::ServoCommandType>::SharedPtr switch_client_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr exec_sub_;
    rclcpp::TimerBase::SharedPtr stream_timer_;
    rclcpp::TimerBase::SharedPtr switch_timer_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    tf2::Quaternion r_align_;

    std::atomic<bool> engaged_{false};
    std::atomic<bool> exec_active_{false};
    std::atomic<bool> pose_mode_set_{false};

    std::mutex mtx_;                     // guards the fields below
    bool have_pose_{false};
    tf2::Vector3 c_now_{0, 0, 0}, c0_{0, 0, 0}, r0_{0, 0, 0}, target_p_{0, 0, 0};
    tf2::Quaternion qc_now_{0, 0, 0, 1}, qc0_{0, 0, 0, 1}, qr0_{0, 0, 0, 1},
        target_q_{0, 0, 0, 1};
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("quest_servo_teleop");
    QuestServoTeleop teleop(node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
