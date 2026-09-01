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
//   ~/active_mode                      std_msgs/String            DOF mode (latched)
//
// Clutch model (deadman): motion only while the grip is held. On the press edge
// we latch the controller pose (c0,qc0) and the current EEF pose (r0,qr0, from
// TF). While held, the EEF target follows the controller *delta* since the latch,
// rotated from the operator frame into the robot base frame and scaled. On
// release we stop streaming so Servo halts (incoming_command_timeout), and the
// next press re-anchors -- so the operator can recenter their hand without moving
// the robot (the "mouse-lift" clutch).
//
// DOF modes: the clutch delta is multiplied by a per-axis gain vector
// [tx,ty,tz,rx,ry,rz] before it is applied to the anchor, so a gain of 0 pins
// that axis to its anchored value ("translation only", "yaw only", ...) and
// values in between damp it. Because the target is rebuilt from the anchor every
// cycle, a pinned axis needs no extra state -- zeroing its delta *is* holding it.
// Modes are named presets (see kModePresets), switchable at runtime through the
// `motion_mode` parameter or a controller button. Every switch re-anchors:
// without that, dropping an axis collapses its accumulated delta to zero in a
// single cycle and steps the target discontinuously.
//
// Note that masking the *target* does not guarantee the arm holds the pinned
// axis exactly -- Servo's IK, singularity damping and collision slowdown can all
// leave residual motion there. This is a teleop mapping, not a hard constraint.

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/srv/servo_command_type.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

namespace {
// moveit_msgs/srv/ServoCommandType: 0=JOINT_JOG, 1=TWIST, 2=POSE.
constexpr int8_t CMD_POSE = 2;

// Per-axis gains on the clutch delta: [tx, ty, tz, rx, ry, rz]. 0 pins the axis
// to its anchored value, 1 follows the hand 1:1. Rotation axis names below are
// those of the *mask frame* (mask_frame=base: rz=yaw, ry=pitch, rx=roll about
// the robot base; mask_frame=tool: the same about the anchored tool axes).
using DofGain = std::array<double, 6>;

struct ModePreset {
    const char* name;
    DofGain gain;
};

constexpr ModePreset kModePresets[] = {
    {"full",        {{1, 1, 1, 1, 1, 1}}},
    {"translation", {{1, 1, 1, 0, 0, 0}}},
    {"rotation",    {{0, 0, 0, 1, 1, 1}}},
    {"planar",      {{1, 1, 0, 0, 0, 1}}},  // tabletop: XY + yaw
    {"vertical",    {{0, 0, 1, 0, 0, 0}}},
    {"yaw",         {{0, 0, 0, 0, 0, 1}}},
    {"pitch",       {{0, 0, 0, 0, 1, 0}}},
    {"roll",        {{0, 0, 0, 1, 0, 0}}},
};

// Gains come from the custom_dof_gain parameter instead of the table.
constexpr const char* kCustomMode = "custom";

bool lookupPreset(const std::string& name, DofGain& out) {
    for (const auto& preset : kModePresets) {
        if (name == preset.name) {
            out = preset.gain;
            return true;
        }
    }
    return false;
}

// "full, translation, ..., custom" -- for parameter-rejection messages.
std::string modeNameList() {
    std::string s;
    for (const auto& preset : kModePresets) {
        s += preset.name;
        s += ", ";
    }
    return s + kCustomMode;
}
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

        // DOF masking.
        const std::string mode = declare<std::string>("motion_mode", "full");
        custom_dof_gain_ = declare<std::vector<double>>("custom_dof_gain",
                                                        {1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
        const std::string mask_frame = declare<std::string>("mask_frame", "base");
        mask_frame_tool_ = (mask_frame == "tool");
        if (!mask_frame_tool_ && mask_frame != "base") {
            RCLCPP_WARN(node_->get_logger(),
                        "mask_frame '%s' is not 'base' or 'tool'; using base.",
                        mask_frame.c_str());
        }
        mode_cycle_button_ = declare<int>("mode_cycle_button", -1);        // -1 = disabled
        mode_cycle_list_   = declare<std::vector<std::string>>(
            "mode_cycle_list", {"full", "translation", "planar", "yaw"});
        validateCycleList();

        // Operator->robot frame alignment (yaw about base +Z).
        r_align_.setRPY(0.0, 0.0, align_yaw_deg_ * M_PI / 180.0);
        r_align_.normalize();

        // --- ROS entities -------------------------------------------------
        target_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
            pose_target_topic_, 10);
        // Latched so a UI joining late still learns the active mode.
        mode_pub_ = node_->create_publisher<std_msgs::msg::String>(
            "~/active_mode", rclcpp::QoS(1).transient_local());
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

        // After the TF buffer exists: applyMode re-anchors through it whenever
        // the clutch is engaged.
        applyMode(mode);

        // The pose stream gets its own callback group so that a slow callback
        // elsewhere -- a TF miss, a parameter batch, a service reply -- can never
        // delay a command past Servo's incoming_command_timeout. Every other
        // callback stays in the node's default group and so remains serialised
        // against itself; only gain_ and the latch state cross the two groups,
        // and those are already under mtx_.
        stream_cb_group_ = node_->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, stream_hz));
        stream_timer_ = node_->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&QuestServoTeleop::streamTarget, this), stream_cb_group_);

        // Put Servo into POSE mode (retry until the service is up).
        switch_timer_ = node_->create_wall_timer(
            1s, std::bind(&QuestServoTeleop::ensurePoseMode, this));

        param_cb_ = node_->add_on_set_parameters_callback(
            std::bind(&QuestServoTeleop::onSetParams, this, std::placeholders::_1));

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

    // --- DOF mode management ---------------------------------------------
    DofGain customGain() const {
        DofGain g{{1.0, 1.0, 1.0, 1.0, 1.0, 1.0}};
        if (custom_dof_gain_.size() == 6) {
            std::copy(custom_dof_gain_.begin(), custom_dof_gain_.end(), g.begin());
        } else {
            RCLCPP_WARN(node_->get_logger(),
                        "custom_dof_gain has %zu entries (expected 6); using all-ones.",
                        custom_dof_gain_.size());
        }
        return g;
    }

    void validateCycleList() {
        DofGain unused;
        for (const auto& name : mode_cycle_list_) {
            if (name != kCustomMode && !lookupPreset(name, unused)) {
                RCLCPP_WARN(node_->get_logger(),
                            "mode_cycle_list entry '%s' is not a known mode; cycling to it will fail.",
                            name.c_str());
            }
        }
    }

    // Swap the active gain vector. Re-anchors in the same critical section, so
    // the operator's accumulated delta on a newly-pinned axis is discarded
    // against a fresh anchor instead of stepping the target.
    void applyMode(const std::string& name) {
        DofGain gain{};
        if (name == kCustomMode) {
            gain = customGain();
        } else if (!lookupPreset(name, gain)) {
            RCLCPP_WARN(node_->get_logger(), "Unknown motion_mode '%s'; keeping '%s'.",
                        name.c_str(), motion_mode_.c_str());
            return;
        }

        // The TF lookup can block, so keep it outside the lock.
        tf2::Vector3 r0;
        tf2::Quaternion qr0;
        bool relatch = false;
        if (engaged_.load()) {
            relatch = lookupEef(r0, qr0);
            if (!relatch) {
                // Changing the mask without re-anchoring would jump the target;
                // drop the clutch instead. The next joy message re-engages.
                engaged_.store(false);
                RCLCPP_WARN(node_->get_logger(),
                            "Mode change could not re-anchor (no %s->%s TF); clutch dropped.",
                            base_frame_.c_str(), eef_frame_.c_str());
            }
        }
        {
            std::lock_guard<std::mutex> lock(mtx_);
            gain_ = gain;
            if (relatch) {
                latchLocked(r0, qr0);
            }
        }

        motion_mode_ = name;
        std_msgs::msg::String msg;
        msg.data = name;
        mode_pub_->publish(msg);
        RCLCPP_INFO(node_->get_logger(),
                    "Motion mode '%s' [t %.2f %.2f %.2f | r %.2f %.2f %.2f] in %s frame.",
                    name.c_str(), gain[0], gain[1], gain[2], gain[3], gain[4], gain[5],
                    mask_frame_tool_ ? "tool" : "base");
    }

    void cycleMode() {
        const size_t n = mode_cycle_list_.size();
        if (n == 0) {
            return;
        }
        size_t next = 0;  // current mode not in the list -> start at the front
        for (size_t i = 0; i < n; ++i) {
            if (mode_cycle_list_[i] == motion_mode_) {
                next = (i + 1) % n;
                break;
            }
        }
        // Route through the parameter so `ros2 param get motion_mode` stays
        // truthful and external listeners see the change.
        const auto result = node_->set_parameter(rclcpp::Parameter("motion_mode",
                                                                   mode_cycle_list_[next]));
        if (!result.successful) {
            RCLCPP_WARN(node_->get_logger(), "Mode cycle to '%s' rejected: %s",
                        mode_cycle_list_[next].c_str(), result.reason.c_str());
        }
    }

    // rclcpp hands the whole `ros2 param set` batch to one callback, so validate
    // every parameter in it before touching any state: a batch that fails here is
    // rejected wholesale, and must not leave half its side effects behind.
    rcl_interfaces::msg::SetParametersResult onSetParams(
        const std::vector<rclcpp::Parameter>& params) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto& p : params) {
            if (p.get_name() == "motion_mode") {
                DofGain unused;
                const std::string v = p.as_string();
                if (v != kCustomMode && !lookupPreset(v, unused)) {
                    result.successful = false;
                    result.reason = "unknown motion_mode '" + v + "'; expected one of: "
                                    + modeNameList();
                }
            } else if (p.get_name() == "custom_dof_gain") {
                const auto v = p.as_double_array();
                if (v.size() != 6) {
                    result.successful = false;
                    result.reason = "custom_dof_gain needs 6 entries [tx,ty,tz,rx,ry,rz]";
                } else if (std::any_of(v.begin(), v.end(), [](double d) { return d < 0.0; })) {
                    result.successful = false;
                    result.reason = "custom_dof_gain entries must be >= 0";
                }
            }
        }
        if (!result.successful) {
            return result;
        }

        // Everything validated: commit the plain values first, so that a batch
        // setting both custom_dof_gain and motion_mode=custom picks up the new
        // gains when applyMode runs below.
        bool gains_changed = false;
        for (const auto& p : params) {
            if (p.get_name() == "custom_dof_gain") {
                custom_dof_gain_ = p.as_double_array();
                gains_changed = true;
            } else if (p.get_name() == "mode_cycle_list") {
                mode_cycle_list_ = p.as_string_array();
                validateCycleList();
            } else if (p.get_name() == "mode_cycle_button") {
                mode_cycle_button_ = static_cast<int>(p.as_int());
            }
        }

        // applyMode re-anchors and publishes, so run it at most once per batch.
        for (const auto& p : params) {
            if (p.get_name() == "motion_mode") {
                applyMode(p.as_string());
                return result;
            }
        }
        if (gains_changed && motion_mode_ == kCustomMode) {
            applyMode(kCustomMode);  // gains edited while already in custom mode
        }
        return result;
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
        } else if (enable_axis_index_ >= 0 &&
                   static_cast<size_t>(enable_axis_index_) < msg->axes.size()) {
            pressed = msg->axes[enable_axis_index_] >= enable_axis_threshold_;
        }
        if (pressed && !engaged_.load()) {
            engage();
        } else if (!pressed && engaged_.load()) {
            disengage();
        }

        // Mode cycling on the rising edge. Disabled by default so the face
        // buttons stay free for the gripper.
        if (mode_cycle_button_ >= 0 &&
            static_cast<size_t>(mode_cycle_button_) < msg->buttons.size()) {
            const bool now = msg->buttons[mode_cycle_button_] != 0;
            if (now && !cycle_prev_) {
                cycleMode();
            }
            cycle_prev_ = now;
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
    // Latch the clutch anchors to `r0`/`qr0` and the latest controller pose, and
    // reset the streamed target there. Caller holds mtx_.
    void latchLocked(const tf2::Vector3& r0, const tf2::Quaternion& qr0) {
        c0_ = c_now_;
        qc0_ = qc_now_;
        r0_ = r0;
        qr0_ = qr0;
        target_p_ = r0;  // first command holds the current pose
        target_q_ = qr0;
    }

    void engage() {
        tf2::Vector3 r0;
        tf2::Quaternion qr0;
        if (!lookupEef(r0, qr0)) {
            // Throttled: engaged_ stays false, so the next joy message retries.
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                                 "Clutch engage aborted: no %s->%s TF yet.",
                                 base_frame_.c_str(), eef_frame_.c_str());
            return;
        }
        std::lock_guard<std::mutex> lock(mtx_);
        if (!have_pose_) {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                                 "Clutch engage aborted: no controller pose yet.");
            return;
        }
        latchLocked(r0, qr0);
        engaged_.store(true);
        RCLCPP_INFO(node_->get_logger(),
                    "Clutch ENGAGED (anchor EEF [%.3f, %.3f, %.3f], mode '%s').",
                    r0.x(), r0.y(), r0.z(), motion_mode_.c_str());
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
            // Unlike engage(), nothing re-drives this edge, and streaming against
            // the pre-trajectory anchor would snap the arm back to where it was
            // before the plan ran. Drop the clutch; the next joy message re-engages.
            engaged_.store(false);
            RCLCPP_WARN(node_->get_logger(),
                        "Post-trajectory re-anchor failed; clutch dropped.");
            return;
        }
        std::lock_guard<std::mutex> lock(mtx_);
        latchLocked(r0, qr0);
    }

    // Latest base->EEF transform, and the only place this node reads robot state.
    // Deliberately non-blocking: every caller runs on an executor thread, and the
    // old 20x50ms retry loop slept there, stalling the pose stream well past
    // Servo's incoming_command_timeout (0.1 s) and tripping a halt/resume jerk.
    // A miss is safe at all three call sites -- engage() is re-driven by the next
    // joy message, reanchor() and applyMode() drop the clutch rather than stream
    // against a stale anchor.
    bool lookupEef(tf2::Vector3& p, tf2::Quaternion& q) {
        try {
            const auto tf = tf_buffer_->lookupTransform(base_frame_, eef_frame_,
                                                        tf2::TimePointZero);
            p.setValue(tf.transform.translation.x, tf.transform.translation.y,
                       tf.transform.translation.z);
            q.setValue(tf.transform.rotation.x, tf.transform.rotation.y,
                       tf.transform.rotation.z, tf.transform.rotation.w);
            return true;
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                                 "TF %s->%s unavailable: %s", base_frame_.c_str(),
                                 eef_frame_.c_str(), ex.what());
            return false;
        }
    }

    // --- DOF masking ------------------------------------------------------
    // Both helpers take a clutch delta already expressed in the robot base frame
    // and return it with the per-axis gains applied. `q_mask` rotates mask-frame
    // vectors into the base frame: identity for mask_frame=base, the *anchored*
    // EEF orientation for mask_frame=tool (a live tool orientation would rotate
    // the constraint surface out from under the operator). Caller holds mtx_.

    tf2::Vector3 maskTranslation(const tf2::Vector3& d, const tf2::Quaternion& q_mask) const {
        tf2::Vector3 v = tf2::quatRotate(q_mask.inverse(), d);
        v.setValue(v.x() * gain_[0], v.y() * gain_[1], v.z() * gain_[2]);
        return tf2::quatRotate(q_mask, v);
    }

    // Per-axis gains cannot be applied to a quaternion directly, so drop into the
    // rotation-vector (log map) representation, scale there, and exponentiate
    // back. This is continuous everywhere, unlike an Euler decomposition, which
    // would be order-dependent and gimbal-lock at pitch +-90deg. For a uniform
    // gain it is exactly slerp from identity, which is how orientation_scale_ is
    // folded in here.
    tf2::Quaternion maskRotation(tf2::Quaternion dq, const tf2::Quaternion& q_mask) const {
        if (dq.w() < 0.0) {
            // Shortest path: tf2's getAngle() spans [0, 2pi], so a negative-w
            // delta would otherwise be masked as the long way round.
            dq = tf2::Quaternion(-dq.x(), -dq.y(), -dq.z(), -dq.w());
        }
        const tf2::Quaternion dl = q_mask.inverse() * dq * q_mask;
        const double angle = dl.getAngle();
        if (angle < 1e-9) {
            return tf2::Quaternion::getIdentity();  // getAxis() is arbitrary here
        }
        tf2::Vector3 v = dl.getAxis() * angle;
        v.setValue(v.x() * gain_[3] * orientation_scale_,
                   v.y() * gain_[4] * orientation_scale_,
                   v.z() * gain_[5] * orientation_scale_);
        const double masked_angle = v.length();
        if (masked_angle < 1e-9) {
            return tf2::Quaternion::getIdentity();
        }
        const tf2::Quaternion masked(v / masked_angle, masked_angle);
        return q_mask * masked * q_mask.inverse();
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
            const tf2::Quaternion q_mask =
                mask_frame_tool_ ? qr0_ : tf2::Quaternion::getIdentity();

            // Position: scaled controller delta, rotated into the base frame,
            // then masked per axis. A gain of 0 leaves that component of
            // `desired` equal to the anchor, which is what pins it.
            const tf2::Vector3 dp_quest = c_now_ - c0_;
            const tf2::Vector3 dp_robot =
                maskTranslation(tf2::quatRotate(r_align_, dp_quest), q_mask) * position_scale_;
            const tf2::Vector3 desired = r0_ + dp_robot;

            // Clamp per-cycle target slew (rejects controller glitches/dropouts).
            tf2::Vector3 step = desired - target_p_;
            const double n = step.length();
            if (n > max_target_step_) {
                step *= (max_target_step_ / n);
            }
            target_p_ = target_p_ + step;

            // Orientation: controller rotation since anchor, expressed in the base
            // frame, masked and damped per axis, then applied to the anchored EEF
            // orientation.
            tf2::Quaternion dq_quest = qc_now_ * qc0_.inverse();
            tf2::Quaternion dq_robot = r_align_ * dq_quest * r_align_.inverse();
            dq_robot.normalize();
            target_q_ = maskRotation(dq_robot, q_mask) * qr0_;
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

    // DOF masking. Only touched from executor callbacks, except gain_.
    std::string motion_mode_{"full"};
    std::vector<double> custom_dof_gain_;
    std::vector<std::string> mode_cycle_list_;
    bool mask_frame_tool_{false};
    int mode_cycle_button_{-1};
    bool cycle_prev_{false};

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_;
    rclcpp::Client<moveit_msgs::srv::ServoCommandType>::SharedPtr switch_client_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr exec_sub_;
    rclcpp::CallbackGroup::SharedPtr stream_cb_group_;
    rclcpp::TimerBase::SharedPtr stream_timer_;
    rclcpp::TimerBase::SharedPtr switch_timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    tf2::Quaternion r_align_;

    std::atomic<bool> engaged_{false};
    std::atomic<bool> exec_active_{false};
    std::atomic<bool> pose_mode_set_{false};

    std::mutex mtx_;                     // guards the fields below
    bool have_pose_{false};
    DofGain gain_{{1.0, 1.0, 1.0, 1.0, 1.0, 1.0}};
    tf2::Vector3 c_now_{0, 0, 0}, c0_{0, 0, 0}, r0_{0, 0, 0}, target_p_{0, 0, 0};
    tf2::Quaternion qc_now_{0, 0, 0, 1}, qc0_{0, 0, 0, 1}, qr0_{0, 0, 0, 1},
        target_q_{0, 0, 0, 1};
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("quest_servo_teleop");
    QuestServoTeleop teleop(node);
    // Two threads: one runs the pose stream timer's callback group, the other
    // everything else. More would only contend on mtx_.
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
