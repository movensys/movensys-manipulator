// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.
//
// keyboard_teleop
//
// Keyboard tele-op for MoveIt 2 Servo using THIS robot's frames and joints
// (base "world_manipulator", eef "Link6", joints joint1..joint6) instead of the
// Panda names hardcoded in moveit_servo's stock servo_keyboard_input demo.
//
// Three modes matching moveit_msgs/srv/ServoCommandType:
//   j = JOINT_JOG : keys 1..6 jog each joint
//   t = TWIST     : arrows / '.' / ';' jog the EEF in the base or eef frame
//   p = POSE      : arrows / '.' / ';' nudge the EEF target pose; 'w'/'e' choose
//                   base- or eef-frame nudging.
//                   base frame: keys move a goal kept within pose_max_lead of the
//                   streamed target; the streamed target follows the goal with a
//                   pose_speed / pose_accel profile, so Servo never has to catch up.
//                   eef frame:  the raw delta goes to moveit2_api, which composes
//                   it onto the current EEF pose and streams the result.

#include <termios.h>
#include <unistd.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <control_msgs/msg/joint_jog.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <moveit_msgs/srv/servo_command_type.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

using namespace std::chrono_literals;

namespace {
// Terminal key codes.
constexpr int KEYCODE_RIGHT     = 0x43;
constexpr int KEYCODE_LEFT      = 0x44;
constexpr int KEYCODE_UP        = 0x41;
constexpr int KEYCODE_DOWN      = 0x42;
constexpr int KEYCODE_PERIOD    = 0x2E;
constexpr int KEYCODE_SEMICOLON = 0x3B;
constexpr int KEYCODE_1         = 0x31;
constexpr int KEYCODE_6         = 0x36;
constexpr int KEYCODE_P         = 0x70;
constexpr int KEYCODE_Q         = 0x71;
constexpr int KEYCODE_R         = 0x72;
constexpr int KEYCODE_J         = 0x6A;
constexpr int KEYCODE_T         = 0x74;
constexpr int KEYCODE_W         = 0x77;
constexpr int KEYCODE_E         = 0x65;

const char* TWIST_TOPIC    = "/servo_node/delta_twist_cmds";
const char* JOINT_TOPIC    = "/servo_node/delta_joint_cmds";
const char* POSE_TOPIC     = "/servo_node/pose_target_cmds";
const char* TOOL_POSE_TOPIC = "/wmx/servo_node/tool_pose";
const char* SWITCH_SERVICE = "/servo_node/switch_command_type";
const char* BASE_FRAME_ID  = "world_manipulator";
const char* EEF_FRAME_ID   = "Link6";

// moveit_msgs/srv/ServoCommandType: 0=JOINT_JOG, 1=TWIST, 2=POSE.
constexpr int8_t CMD_JOINT_JOG = 0;
constexpr int8_t CMD_TWIST     = 1;
constexpr int8_t CMD_POSE      = 2;
}  // namespace

enum class Mode { NONE, JOINT, TWIST, POSE };

// Puts the terminal into raw mode so keys are delivered without Enter/echo.
class KeyboardReader {
public:
    KeyboardReader() {
        tcgetattr(STDIN_FILENO, &initial_settings_);
        struct termios raw = initial_settings_;
        raw.c_lflag &= ~(ICANON | ECHO);
        raw.c_cc[VEOL] = 1;
        raw.c_cc[VEOF] = 2;
        tcsetattr(STDIN_FILENO, TCSANOW, &raw);
    }
    int readOne(char* c) {
        int rc = static_cast<int>(::read(STDIN_FILENO, c, 1));
        return rc < 0 ? rc : 0;
    }
    void restore() { tcsetattr(STDIN_FILENO, TCSANOW, &initial_settings_); }

private:
    struct termios initial_settings_;
};

KeyboardReader g_input;

void quit(int /*sig*/) {
    g_input.restore();
    rclcpp::shutdown();
    exit(0);
}

class KeyboardServo {
public:
    KeyboardServo() {
        node_ = rclcpp::Node::make_shared("keyboard_teleop");

        twist_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>(TWIST_TOPIC, 10);
        joint_pub_ = node_->create_publisher<control_msgs::msg::JointJog>(JOINT_TOPIC, 10);
        pose_pub_  = node_->create_publisher<geometry_msgs::msg::PoseStamped>(POSE_TOPIC, 10);
        tool_pose_pub_ =
            node_->create_publisher<geometry_msgs::msg::PoseStamped>(TOOL_POSE_TOPIC, 10);
        switch_client_ =
            node_->create_client<moveit_msgs::srv::ServoCommandType>(SWITCH_SERVICE);

        pose_step_     = node_->declare_parameter<double>("pose_step", 0.01);
        pose_max_lead_ = node_->declare_parameter<double>("pose_max_lead", 0.01);
        pose_speed_    = node_->declare_parameter<double>("pose_speed", 0.08);
        pose_accel_    = node_->declare_parameter<double>("pose_accel", 2.0);
        pose_rate_     = node_->declare_parameter<double>("pose_rate", 500.0);
        if (!(pose_rate_ > 0.0)) {
            pose_rate_ = 500.0;
        }

        tf_buffer_   = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Many target updates per Servo cycle, so Servo never sees a doubled step.
        pose_timer_ = node_->create_wall_timer(
            std::chrono::duration<double>(1.0 / pose_rate_),
            std::bind(&KeyboardServo::publishPose, this));

        // While move_group executes a trajectory, pause POSE streaming and
        // re-anchor the target when it finishes (transient_local = catch last state).
        exec_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
            "/moveit2_trajectory/execution_active", rclcpp::QoS(1).transient_local(),
            std::bind(&KeyboardServo::cbExecActive, this, std::placeholders::_1));
    }

    void spin() {
        rclcpp::executors::SingleThreadedExecutor exec;
        exec.add_node(node_);
        exec.spin();
    }

    int keyLoop();

private:
    void switchCommandType(int8_t type);
    bool seedTargetFromTF();
    void enterPoseMode();
    void nudgePose(double dx, double dy, double dz);
    void advanceTarget();
    void publishPose();
    void cbExecActive(std_msgs::msg::Bool::SharedPtr msg);

    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
    rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr tool_pose_pub_;
    rclcpp::Client<moveit_msgs::srv::ServoCommandType>::SharedPtr switch_client_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr exec_sub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr pose_timer_;

    std::atomic<bool> exec_active_{false};   // true while a move_group plan runs

    std::mutex mtx_;                       // guards mode_, target_pose_, goal_, target_vel_
    Mode mode_ = Mode::NONE;
    geometry_msgs::msg::PoseStamped target_pose_;   // streamed to Servo
    geometry_msgs::msg::Point goal_;                // where the keys want the EEF
    std::array<double, 3> target_vel_{};            // streamed target velocity [m/s]

    double pose_step_ = 0.0;
    double pose_max_lead_ = 0.0;
    double pose_speed_ = 0.0;
    double pose_accel_ = 0.0;
    double pose_rate_ = 0.0;

    std::string frame_ = BASE_FRAME_ID;    // twist command frame ('w'/'e')
    double sign_ = 1.0;                     // direction ('r')
};

void KeyboardServo::switchCommandType(int8_t type) {
    if (!switch_client_->wait_for_service(1s)) {
        RCLCPP_WARN(node_->get_logger(), "%s not available", SWITCH_SERVICE);
        return;
    }
    auto req = std::make_shared<moveit_msgs::srv::ServoCommandType::Request>();
    req->command_type = type;
    switch_client_->async_send_request(req);
}

// Seeds target_pose_ from the current EEF pose (base -> eef). TimePointZero =
// latest available; retry while the TF listener fills the buffer. Returns true on
// success. Must be called WITHOUT holding mtx_ (it takes the lock itself).
bool KeyboardServo::seedTargetFromTF() {
    geometry_msgs::msg::TransformStamped tf;
    bool ok = false;
    for (int i = 0; i < 20 && rclcpp::ok(); ++i) {
        try {
            tf = tf_buffer_->lookupTransform(BASE_FRAME_ID, EEF_FRAME_ID, tf2::TimePointZero);
            ok = true;
            break;
        } catch (const tf2::TransformException&) {
            std::this_thread::sleep_for(50ms);
        }
    }
    if (!ok) {
        RCLCPP_WARN(node_->get_logger(),
                    "POSE mode: could not look up %s -> %s",
                    BASE_FRAME_ID, EEF_FRAME_ID);
        return false;
    }

    std::lock_guard<std::mutex> lock(mtx_);
    target_pose_.header.frame_id = BASE_FRAME_ID;
    target_pose_.pose.position.x = tf.transform.translation.x;
    target_pose_.pose.position.y = tf.transform.translation.y;
    target_pose_.pose.position.z = tf.transform.translation.z;
    target_pose_.pose.orientation = tf.transform.rotation;
    goal_ = target_pose_.pose.position;
    target_vel_ = {0.0, 0.0, 0.0};
    RCLCPP_INFO(node_->get_logger(),
                "POSE target seeded at [%.3f, %.3f, %.3f]",
                target_pose_.pose.position.x, target_pose_.pose.position.y,
                target_pose_.pose.position.z);
    return true;
}

void KeyboardServo::enterPoseMode() {
    switchCommandType(CMD_POSE);
    if (!seedTargetFromTF()) {
        RCLCPP_WARN(node_->get_logger(), "POSE mode: staying in previous mode");
        return;
    }
    std::lock_guard<std::mutex> lock(mtx_);
    mode_ = Mode::POSE;
}

// move_group execution boundary: pause POSE streaming while it runs; on the
// true->false edge, re-anchor the target to where the trajectory left the arm
// so POSE mode resumes holding the new pose instead of snapping back.
void KeyboardServo::cbExecActive(std_msgs::msg::Bool::SharedPtr msg) {
    const bool was_active = exec_active_.exchange(msg->data);
    if (was_active && !msg->data) {
        Mode mode;
        { std::lock_guard<std::mutex> lock(mtx_); mode = mode_; }
        if (mode == Mode::POSE) {
            seedTargetFromTF();   // re-anchor to the arm's new pose
        }
    }
}

void KeyboardServo::nudgePose(double dx, double dy, double dz) {
    if (frame_ == EEF_FRAME_ID) {
        if (exec_active_.load()) {
            return;
        }
        geometry_msgs::msg::PoseStamped msg;
        msg.header.stamp       = node_->now();
        msg.header.frame_id    = EEF_FRAME_ID;
        msg.pose.position.x    = dx;
        msg.pose.position.y    = dy;
        msg.pose.position.z    = dz;
        msg.pose.orientation.w = 1.0;   // no rotation
        tool_pose_pub_->publish(msg);
        return;
    }

    // Base frame: keep only the part of the current lead that points along this
    // key's direction, so a reversal or turn takes effect at once, then move the
    // goal, never more than pose_max_lead_ ahead of the streamed target.
    std::lock_guard<std::mutex> lock(mtx_);
    const auto & p = target_pose_.pose.position;
    const double step = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (step <= 0.0) {
        return;
    }
    const double along = std::max(
        0.0, ((goal_.x - p.x) * dx + (goal_.y - p.y) * dy + (goal_.z - p.z) * dz) / step);
    double ex = (along + step) * dx / step;
    double ey = (along + step) * dy / step;
    double ez = (along + step) * dz / step;
    const double lead = std::sqrt(ex * ex + ey * ey + ez * ez);
    if (lead > pose_max_lead_) {
        const double k = pose_max_lead_ / lead;
        ex *= k;
        ey *= k;
        ez *= k;
    }
    goal_.x = p.x + ex;
    goal_.y = p.y + ey;
    goal_.z = p.z + ez;
}

// Moves the streamed target toward goal_. The wanted velocity points at the goal
// with a speed that can still stop on it; the actual velocity changes toward it by
// at most pose_accel_ * dt, so turns and reversals stay accel-limited.
void KeyboardServo::advanceTarget() {
    auto & p = target_pose_.pose.position;
    const double dt = 1.0 / pose_rate_;
    const std::array<double, 3> e{goal_.x - p.x, goal_.y - p.y, goal_.z - p.z};
    const double dist = std::sqrt(e[0] * e[0] + e[1] * e[1] + e[2] * e[2]);

    std::array<double, 3> want{0.0, 0.0, 0.0};
    if (dist > 1e-6) {
        const double speed = std::min(pose_speed_, std::sqrt(2.0 * pose_accel_ * dist));
        for (int i = 0; i < 3; ++i) {
            want[i] = e[i] / dist * speed;
        }
    }

    std::array<double, 3> dv{};
    double dvNorm = 0.0;
    for (int i = 0; i < 3; ++i) {
        dv[i] = want[i] - target_vel_[i];
        dvNorm += dv[i] * dv[i];
    }
    dvNorm = std::sqrt(dvNorm);
    const double dvMax = pose_accel_ * dt;
    const double k = (dvNorm > dvMax) ? dvMax / dvNorm : 1.0;
    for (int i = 0; i < 3; ++i) {
        target_vel_[i] += k * dv[i];
    }

    const double speed = std::sqrt(target_vel_[0] * target_vel_[0] +
                                   target_vel_[1] * target_vel_[1] +
                                   target_vel_[2] * target_vel_[2]);
    if (dist < 1e-4 && speed < dvMax) {
        p = goal_;
        target_vel_ = {0.0, 0.0, 0.0};
        return;
    }
    p.x += target_vel_[0] * dt;
    p.y += target_vel_[1] * dt;
    p.z += target_vel_[2] * dt;
}

void KeyboardServo::publishPose() {
    if (exec_active_.load()) {
        return;  // paused while a move_group trajectory executes
    }
    if (frame_ == EEF_FRAME_ID) {
        return;  // moveit2_api owns the stream in tool-frame mode
    }
    std::lock_guard<std::mutex> lock(mtx_);
    if (mode_ != Mode::POSE) {
        return;
    }
    advanceTarget();
    target_pose_.header.stamp = node_->now();
    pose_pub_->publish(target_pose_);
}

int KeyboardServo::keyLoop() {
    puts("\nMovensys keyboard tele-op for MoveIt Servo");
    puts("------------------------------------------");
    puts("  j : JOINT mode   -> keys 1..6 jog joint 1..6");
    puts("  t : TWIST mode   -> arrows = X/Y, '.'/';' = Z");
    puts("  p : POSE mode    -> arrows = X/Y, '.'/';' = Z nudge the EEF target pose");
    puts("  w / e : frame for TWIST jog and POSE nudge = base / eef");
    puts("  r : reverse direction (twist/joint)");
    puts("  q : quit\n");

    char c;
    for (;;) {
        if (g_input.readOne(&c) < 0) {
            perror("read()");
            return -1;
        }

        auto twist = std::make_unique<geometry_msgs::msg::TwistStamped>();
        auto joint = std::make_unique<control_msgs::msg::JointJog>();
        bool publish_twist = false;
        bool publish_joint = false;

        Mode mode;
        { std::lock_guard<std::mutex> lock(mtx_); mode = mode_; }

        switch (c) {
            // --- mode selection ---
            case KEYCODE_J: switchCommandType(CMD_JOINT_JOG);
                { std::lock_guard<std::mutex> lock(mtx_); mode_ = Mode::JOINT; }
                RCLCPP_INFO(node_->get_logger(), "JOINT mode"); break;
            case KEYCODE_T: switchCommandType(CMD_TWIST);
                { std::lock_guard<std::mutex> lock(mtx_); mode_ = Mode::TWIST; }
                RCLCPP_INFO(node_->get_logger(), "TWIST mode"); break;
            case KEYCODE_P: enterPoseMode(); break;

            // --- X / Y ---
            case KEYCODE_UP:
                if (mode == Mode::POSE) {
                    nudgePose(pose_step_, 0, 0);
                } else {
                    twist->twist.linear.x =  sign_; publish_twist = true;
                }
                break;
            case KEYCODE_DOWN:
                if (mode == Mode::POSE) {
                    nudgePose(-pose_step_, 0, 0);
                } else {
                    twist->twist.linear.x = -sign_; publish_twist = true;
                }
                break;
            case KEYCODE_RIGHT:
                if (mode == Mode::POSE) {
                    nudgePose(0, pose_step_, 0);
                } else {
                    twist->twist.linear.y =  sign_; publish_twist = true;
                }
                break;
            case KEYCODE_LEFT:
                if (mode == Mode::POSE) {
                    nudgePose(0, -pose_step_, 0);
                } else {
                    twist->twist.linear.y = -sign_; publish_twist = true;
                }
                break;

            // --- Z ---
            case KEYCODE_SEMICOLON:
                if (mode == Mode::POSE) {
                    nudgePose(0, 0, pose_step_);
                } else {
                    twist->twist.linear.z =  sign_; publish_twist = true;
                }
                break;
            case KEYCODE_PERIOD:
                if (mode == Mode::POSE) {
                    nudgePose(0, 0, -pose_step_);
                } else {
                    twist->twist.linear.z = -sign_; publish_twist = true;
                }
                break;

            // --- twist command frame / direction ---
            case KEYCODE_W: {
                frame_ = BASE_FRAME_ID;
                RCLCPP_INFO(node_->get_logger(), "Frame: %s", frame_.c_str());
                Mode mode;
                { std::lock_guard<std::mutex> lock(mtx_); mode = mode_; }
                if (mode == Mode::POSE) {
                    seedTargetFromTF();
                }
                break;
            }
            case KEYCODE_E: frame_ = EEF_FRAME_ID;
                RCLCPP_INFO(node_->get_logger(), "Frame: %s", frame_.c_str()); break;
            case KEYCODE_R: sign_ = -sign_;
                RCLCPP_INFO(node_->get_logger(), "Direction reversed"); break;

            case KEYCODE_Q: quit(0); break;

            default:
                if (mode == Mode::JOINT && c >= KEYCODE_1 && c <= KEYCODE_6) {
                    joint->joint_names.push_back("joint" + std::to_string(c - KEYCODE_1 + 1));
                    joint->velocities.push_back(sign_);
                    publish_joint = true;
                }
                break;
        }

        if (publish_twist) {
            twist->header.stamp = node_->now();
            twist->header.frame_id = frame_;
            twist_pub_->publish(std::move(twist));
        } else if (publish_joint) {
            joint->header.stamp = node_->now();
            joint->header.frame_id = BASE_FRAME_ID;
            joint_pub_->publish(std::move(joint));
        }
    }
    return 0;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    signal(SIGINT, quit);

    KeyboardServo keyboard;
    std::thread spin_thread([&keyboard]() { keyboard.spin(); });
    spin_thread.detach();

    int rc = keyboard.keyLoop();

    g_input.restore();
    rclcpp::shutdown();
    return rc;
}
