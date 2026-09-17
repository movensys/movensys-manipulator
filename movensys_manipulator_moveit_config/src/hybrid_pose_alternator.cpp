#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <moveit_msgs/action/hybrid_planner.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace
{
using HybridPlanner = moveit_msgs::action::HybridPlanner;
using GoalHandleHybridPlanner = rclcpp_action::ClientGoalHandle<HybridPlanner>;

constexpr std::size_t kJointCount = 6;

std::array<double, kJointCount> toArray(
  const std::vector<double> & values,
  const std::array<double, kJointCount> & fallback)
{
  if (values.size() != kJointCount) {
    return fallback;
  }

  std::array<double, kJointCount> result{};
  std::copy(values.begin(), values.end(), result.begin());
  return result;
}
}  // namespace

class HybridPoseAlternator : public rclcpp::Node
{
public:
  HybridPoseAlternator()
  : Node("hybrid_pose_alternator")
  {
    action_name_ = declare_parameter<std::string>("action_name", "/run_hybrid_planning");
    planning_group_ = declare_parameter<std::string>("planning_group", "movensys_manipulator_arm");
    pipeline_id_ = declare_parameter<std::string>("pipeline_id", "isaac_ros_cumotion");
    allowed_planning_time_ = declare_parameter<double>("allowed_planning_time", 5.0);
    velocity_scaling_ = declare_parameter<double>("max_velocity_scaling_factor", 0.4);
    acceleration_scaling_ = declare_parameter<double>("max_acceleration_scaling_factor", 0.4);
    tolerance_ = declare_parameter<double>("joint_tolerance", 0.01);
    reach_tolerance_ = declare_parameter<double>("goal_reached_tolerance", tolerance_);
    wait_after_reach_seconds_ = declare_parameter<double>("wait_after_reach_seconds", 1.0);
    retry_delay_seconds_ = declare_parameter<double>("retry_delay_seconds", 5.0);
    goal_response_timeout_seconds_ = declare_parameter<double>("goal_response_timeout_seconds", 2.0);
    result_timeout_seconds_ = declare_parameter<double>("result_timeout_seconds", 60.0);
    reach_timeout_seconds_ = declare_parameter<double>("reach_timeout_seconds", 3.0);
    monitor_period_seconds_ = declare_parameter<double>("monitor_period_seconds", 0.1);
    joint_state_topic_ = declare_parameter<std::string>("joint_state_topic", "/joint_states");

    const std::array<double, kJointCount> default_pose_a = {
      -0.6173796057701111,
      -0.44901999831199646,
      -0.3376253545284271,
      -0.7845771908760071,
      1.5694209337234497,
      -0.6163911819458008,
    };
    const std::array<double, kJointCount> default_pose_b = {
      1.246348261833191,
      -0.5102301239967346,
      -0.23093771934509277,
      -0.830804169178009,
      1.57161545753479,
      1.2473478317260742,
    };

    joint_names_ = declare_parameter<std::vector<std::string>>(
      "joint_names", {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"});
    pose_a_ = toArray(declare_parameter<std::vector<double>>(
      "pose_a", std::vector<double>(default_pose_a.begin(), default_pose_a.end())), default_pose_a);
    pose_b_ = toArray(declare_parameter<std::vector<double>>(
      "pose_b", std::vector<double>(default_pose_b.begin(), default_pose_b.end())), default_pose_b);

    if (joint_names_.size() != kJointCount) {
      RCLCPP_WARN(
        get_logger(),
        "joint_names must contain 6 names; falling back to joint1..joint6");
      joint_names_ = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
    }

    action_client_ = rclcpp_action::create_client<HybridPlanner>(this, action_name_);
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_state_topic_, rclcpp::QoS(10),
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        updateJointState(*msg);
      });
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(monitor_period_seconds_)),
      [this]() { monitorGoalProgress(); });
  }

  void start()
  {
    sendNextGoal();
  }

private:
  void sendNextGoal()
  {
    sendGoal(next_pose_is_a_);
  }

  void sendGoal(bool pose_is_a)
  {
    if (goal_in_flight_) {
      RCLCPP_WARN(get_logger(), "Previous HybridPlanner goal is still active; skipping this send");
      return;
    }

    if (!action_client_->wait_for_action_server(std::chrono::seconds(1))) {
      RCLCPP_WARN(
        get_logger(),
        "HybridPlanner action server '%s' is not available yet",
        action_name_.c_str());
      scheduleRetry(pose_is_a);
      return;
    }

    retry_pending_ = false;
    const auto & target = pose_is_a ? pose_a_ : pose_b_;
    const std::string target_name = pose_is_a ? "pose_a" : "pose_b";
    const std::size_t sequence = ++active_goal_sequence_;
    auto goal = makeGoal(target);

    rclcpp_action::Client<HybridPlanner>::SendGoalOptions options;
    options.goal_response_callback =
      [this, target_name, pose_is_a, sequence](const GoalHandleHybridPlanner::SharedPtr & goal_handle) {
        if (sequence != active_goal_sequence_) {
          RCLCPP_DEBUG(
            get_logger(), "Ignoring stale goal response for %s", target_name.c_str());
          return;
        }

        goal_response_received_ = true;
        if (!goal_handle) {
          goal_in_flight_ = false;
          waiting_for_reach_ = false;
          active_goal_handle_.reset();
          RCLCPP_ERROR(get_logger(), "HybridPlanner rejected %s", target_name.c_str());
          scheduleRetry(pose_is_a);
          return;
        }
        active_goal_handle_ = goal_handle;
        RCLCPP_INFO(get_logger(), "HybridPlanner accepted %s", target_name.c_str());
      };
    options.result_callback =
      [this, target_name, pose_is_a, sequence](const GoalHandleHybridPlanner::WrappedResult & result) {
        if (sequence != active_goal_sequence_) {
          RCLCPP_DEBUG(get_logger(), "Ignoring stale result for %s", target_name.c_str());
          return;
        }

        goal_in_flight_ = false;
        active_goal_handle_.reset();
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED &&
            result.result->error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
          planning_succeeded_ = true;
          reach_wait_start_time_ = get_clock()->now();
          RCLCPP_INFO(get_logger(), "HybridPlanner succeeded for %s", target_name.c_str());
        } else {
          waiting_for_reach_ = false;
          planning_succeeded_ = false;
          RCLCPP_ERROR(
            get_logger(),
            "HybridPlanner failed for %s: action_code=%d moveit_code=%d message='%s'",
            target_name.c_str(),
            static_cast<int>(result.code),
            result.result ? result.result->error_code.val : 0,
            result.result ? result.result->error_message.c_str() : "");
          scheduleRetry(pose_is_a);
        }
      };

    goal_in_flight_ = true;
    active_pose_is_a_ = pose_is_a;
    active_target_ = target;
    active_target_name_ = target_name;
    waiting_for_reach_ = true;
    goal_response_received_ = false;
    planning_succeeded_ = false;
    goal_sent_time_ = get_clock()->now();
    reach_wait_start_time_ = goal_sent_time_;
    reached_since_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    RCLCPP_INFO(get_logger(), "Sending HybridPlanner goal for %s", target_name.c_str());
    action_client_->async_send_goal(goal, options);
    next_pose_is_a_ = !pose_is_a;
  }

  void updateJointState(const sensor_msgs::msg::JointState & msg)
  {
    const std::size_t count = std::min(msg.name.size(), msg.position.size());
    for (std::size_t i = 0; i < count; ++i) {
      current_joint_positions_[msg.name[i]] = msg.position[i];
    }
    have_joint_state_ = true;
  }

  void monitorGoalProgress()
  {
    if (goal_in_flight_) {
      const auto now = get_clock()->now();
      const double elapsed = (now - goal_sent_time_).seconds();
      if (!goal_response_received_ && elapsed >= goal_response_timeout_seconds_) {
        RCLCPP_ERROR(
          get_logger(),
          "HybridPlanner did not respond to %s within %.3f sec; retrying",
          active_target_name_.c_str(), goal_response_timeout_seconds_);
        goal_in_flight_ = false;
        active_goal_handle_.reset();
        ++active_goal_sequence_;
        scheduleRetry(active_pose_is_a_);
        return;
      }

      if (goal_response_received_ && elapsed >= result_timeout_seconds_) {
        RCLCPP_ERROR(
          get_logger(),
          "HybridPlanner did not finish %s within %.3f sec; retrying",
          active_target_name_.c_str(), result_timeout_seconds_);
        goal_in_flight_ = false;
        if (active_goal_handle_) {
          action_client_->async_cancel_goal(active_goal_handle_);
          active_goal_handle_.reset();
        }
        ++active_goal_sequence_;
        scheduleRetry(active_pose_is_a_);
        return;
      }
    }

    if (retry_pending_ && !goal_in_flight_) {
      const auto now = get_clock()->now();
      if (now >= retry_time_) {
        const bool retry_pose_is_a = retry_pose_is_a_;
        retry_pending_ = false;
        sendGoal(retry_pose_is_a);
      }
      return;
    }

    if (!waiting_for_reach_) {
      return;
    }

    if (!have_joint_state_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for joint states on '%s' before checking goal reach",
        joint_state_topic_.c_str());
      return;
    }

    double max_error = 0.0;
    std::string worst_joint;
    if (!isTargetReached(max_error, worst_joint)) {
      if (reached_since_.nanoseconds() != 0) {
        RCLCPP_INFO(
          get_logger(),
          "%s moved outside goal tolerance; max_error=%.6f at %s, tolerance=%.6f",
          active_target_name_.c_str(), max_error, worst_joint.c_str(), reach_tolerance_);
      }
      reached_since_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());

      const auto now = get_clock()->now();
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for %s to reach goal: max_error=%.6f at %s, tolerance=%.6f",
        active_target_name_.c_str(), max_error, worst_joint.c_str(), reach_tolerance_);

      if (planning_succeeded_ && !goal_in_flight_ && reach_timeout_seconds_ > 0.0 &&
          (now - reach_wait_start_time_).seconds() >= reach_timeout_seconds_) {
        RCLCPP_ERROR(
          get_logger(),
          "%s did not reach goal position within %.3f sec after HybridPlanner success; retrying",
          active_target_name_.c_str(), reach_timeout_seconds_);
        scheduleRetry(active_pose_is_a_);
      }
      return;
    }

    const auto now = get_clock()->now();
    if (reached_since_.nanoseconds() == 0) {
      reached_since_ = now;
      RCLCPP_INFO(
        get_logger(),
        "%s reached goal position; waiting %.3f sec before next goal",
        active_target_name_.c_str(), wait_after_reach_seconds_);
      return;
    }

    if ((now - reached_since_).seconds() < wait_after_reach_seconds_) {
      return;
    }

    if (goal_in_flight_) {
      RCLCPP_DEBUG(
        get_logger(),
        "%s reached goal, but HybridPlanner action is still active",
        active_target_name_.c_str());
      return;
    }

    waiting_for_reach_ = false;
    reached_since_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    sendNextGoal();
  }

  void scheduleRetry(bool pose_is_a)
  {
    retry_pose_is_a_ = pose_is_a;
    retry_pending_ = true;
    waiting_for_reach_ = false;
    planning_succeeded_ = false;
    reached_since_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    retry_time_ = get_clock()->now() + rclcpp::Duration::from_seconds(retry_delay_seconds_);
    next_pose_is_a_ = pose_is_a;
    goal_response_received_ = false;
    active_goal_handle_.reset();
    RCLCPP_WARN(
      get_logger(),
      "Retrying %s after %.3f sec",
      pose_is_a ? "pose_a" : "pose_b", retry_delay_seconds_);
  }

  bool isTargetReached(double & max_error, std::string & worst_joint) const
  {
    max_error = 0.0;
    worst_joint.clear();
    bool reached = true;
    for (std::size_t i = 0; i < kJointCount; ++i) {
      const auto it = current_joint_positions_.find(joint_names_[i]);
      if (it == current_joint_positions_.end()) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Joint state does not contain '%s'", joint_names_[i].c_str());
        return false;
      }

      const double error = std::abs(it->second - active_target_[i]);
      if (error > max_error) {
        max_error = error;
        worst_joint = joint_names_[i];
      }
      if (error > reach_tolerance_) {
        reached = false;
      }
    }
    return reached;
  }

  HybridPlanner::Goal makeGoal(const std::array<double, kJointCount> & positions) const
  {
    HybridPlanner::Goal goal;
    goal.planning_group = planning_group_;

    moveit_msgs::msg::MotionSequenceItem item;
    item.req.pipeline_id = pipeline_id_;
    item.req.group_name = planning_group_;
    item.req.allowed_planning_time = allowed_planning_time_;
    item.req.max_velocity_scaling_factor = velocity_scaling_;
    item.req.max_acceleration_scaling_factor = acceleration_scaling_;

    moveit_msgs::msg::Constraints constraints;
    constraints.joint_constraints.reserve(kJointCount);
    for (std::size_t i = 0; i < kJointCount; ++i) {
      moveit_msgs::msg::JointConstraint joint_constraint;
      joint_constraint.joint_name = joint_names_[i];
      joint_constraint.position = positions[i];
      joint_constraint.tolerance_above = tolerance_;
      joint_constraint.tolerance_below = tolerance_;
      joint_constraint.weight = 1.0;
      constraints.joint_constraints.push_back(joint_constraint);
    }

    item.req.goal_constraints.push_back(constraints);
    item.blend_radius = 0.0;
    goal.motion_sequence.items.push_back(item);
    return goal;
  }

  rclcpp_action::Client<HybridPlanner>::SharedPtr action_client_;
  GoalHandleHybridPlanner::SharedPtr active_goal_handle_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string action_name_;
  std::string planning_group_;
  std::string pipeline_id_;
  std::string joint_state_topic_;
  std::vector<std::string> joint_names_;
  std::array<double, kJointCount> pose_a_{};
  std::array<double, kJointCount> pose_b_{};
  std::array<double, kJointCount> active_target_{};
  std::string active_target_name_;
  std::unordered_map<std::string, double> current_joint_positions_;
  double allowed_planning_time_ = 5.0;
  double velocity_scaling_ = 0.4;
  double acceleration_scaling_ = 0.4;
  double tolerance_ = 0.01;
  double reach_tolerance_ = 0.01;
  double wait_after_reach_seconds_ = 5.0;
  double retry_delay_seconds_ = 10.0;
  double goal_response_timeout_seconds_ = 2.0;
  double result_timeout_seconds_ = 60.0;
  double reach_timeout_seconds_ = 5.0;
  double monitor_period_seconds_ = 0.1;
  rclcpp::Time reached_since_;
  rclcpp::Time goal_sent_time_;
  rclcpp::Time reach_wait_start_time_;
  rclcpp::Time retry_time_;
  std::size_t active_goal_sequence_ = 0;
  bool next_pose_is_a_ = true;
  bool active_pose_is_a_ = true;
  bool goal_in_flight_ = false;
  bool waiting_for_reach_ = false;
  bool have_joint_state_ = false;
  bool retry_pending_ = false;
  bool retry_pose_is_a_ = true;
  bool goal_response_received_ = false;
  bool planning_succeeded_ = false;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HybridPoseAlternator>();
  node->start();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
