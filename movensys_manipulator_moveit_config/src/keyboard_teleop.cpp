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
//   p = POSE      : arrows / '.' / ';' nudge an absolute EEF target pose, seeded
//                   from the current pose (via TF) and streamed so Servo tracks it;
//                   'w'/'e' choose base- or eef-frame nudging

#include <termios.h>
#include <unistd.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <atomic>
#include <chrono>
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
const char* SWITCH_SERVICE = "/servo_node/switch_command_type";
const char* BASE_FRAME_ID  = "world_manipulator";
const char* EEF_FRAME_ID   = "Link6";

// moveit_msgs/srv/ServoCommandType: 0=JOINT_JOG, 1=TWIST, 2=POSE.
constexpr int8_t CMD_JOINT_JOG = 0;
constexpr int8_t CMD_TWIST     = 1;
constexpr int8_t CMD_POSE      = 2;

constexpr double POSE_STEP = 0.01;  // metres per keypress in POSE mode
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
        // Force sim time so TF and stamps line up with the Isaac-driven clock.
        auto opts = rclcpp::NodeOptions().parameter_overrides(
            {rclcpp::Parameter("use_sim_time", true)});
        node_ = rclcpp::Node::make_shared("keyboard_teleop", opts);

        twist_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>(TWIST_TOPIC, 10);
        joint_pub_ = node_->create_publisher<control_msgs::msg::JointJog>(JOINT_TOPIC, 10);
        pose_pub_  = node_->create_publisher<geometry_msgs::msg::PoseStamped>(POSE_TOPIC, 10);
        switch_client_ =
            node_->create_client<moveit_msgs::srv::ServoCommandType>(SWITCH_SERVICE);

        tf_buffer_   = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Streams the POSE target so Servo keeps tracking it (~50 Hz).
        pose_timer_ = node_->create_wall_timer(
            20ms, std::bind(&KeyboardServo::publishPose, this));

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
    void publishPose();
    void cbExecActive(std_msgs::msg::Bool::SharedPtr msg);

    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
    rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Client<moveit_msgs::srv::ServoCommandType>::SharedPtr switch_client_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr exec_sub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr pose_timer_;

    std::atomic<bool> exec_active_{false};   // true while a move_group plan runs

    std::mutex mtx_;                       // guards mode_ and target_pose_
    Mode mode_ = Mode::NONE;
    geometry_msgs::msg::PoseStamped target_pose_;

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
    std::lock_guard<std::mutex> lock(mtx_);
    tf2::Vector3 d(dx, dy, dz);
    if (frame_ == EEF_FRAME_ID) {
        tf2::Quaternion q(
            target_pose_.pose.orientation.x, target_pose_.pose.orientation.y,
            target_pose_.pose.orientation.z, target_pose_.pose.orientation.w);
        d = tf2::quatRotate(q, d);
    }
    target_pose_.pose.position.x += d.x();
    target_pose_.pose.position.y += d.y();
    target_pose_.pose.position.z += d.z();
}

void KeyboardServo::publishPose() {
    if (exec_active_.load()) {
        return;  // paused while a move_group trajectory executes
    }
    std::lock_guard<std::mutex> lock(mtx_);
    if (mode_ != Mode::POSE) {
        return;
    }
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
                    nudgePose(POSE_STEP, 0, 0);
                } else {
                    twist->twist.linear.x =  sign_; publish_twist = true;
                }
                break;
            case KEYCODE_DOWN:
                if (mode == Mode::POSE) {
                    nudgePose(-POSE_STEP, 0, 0);
                } else {
                    twist->twist.linear.x = -sign_; publish_twist = true;
                }
                break;
            case KEYCODE_RIGHT:
                if (mode == Mode::POSE) {
                    nudgePose(0, POSE_STEP, 0);
                } else {
                    twist->twist.linear.y =  sign_; publish_twist = true;
                }
                break;
            case KEYCODE_LEFT:
                if (mode == Mode::POSE) {
                    nudgePose(0, -POSE_STEP, 0);
                } else {
                    twist->twist.linear.y = -sign_; publish_twist = true;
                }
                break;

            // --- Z ---
            case KEYCODE_SEMICOLON:
                if (mode == Mode::POSE) {
                    nudgePose(0, 0, POSE_STEP);
                } else {
                    twist->twist.linear.z =  sign_; publish_twist = true;
                }
                break;
            case KEYCODE_PERIOD:
                if (mode == Mode::POSE) {
                    nudgePose(0, 0, -POSE_STEP);
                } else {
                    twist->twist.linear.z = -sign_; publish_twist = true;
                }
                break;

            // --- twist command frame / direction ---
            case KEYCODE_W: frame_ = BASE_FRAME_ID;
                RCLCPP_INFO(node_->get_logger(), "Frame: %s", frame_.c_str()); break;
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
