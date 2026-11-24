#ifndef MOVENSYS_MANIPULATOR_ISAAC_ROS_CONFIG__CUROBO_PANEL_HPP_
#define MOVENSYS_MANIPULATOR_ISAAC_ROS_CONFIG__CUROBO_PANEL_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <QCheckBox>
#include <QComboBox>
#include <QLabel>
#include <QPushButton>
#include <QTimer>

#include "movensys_manipulator_isaac_ros_config/msg/planner_status.hpp"
#include "movensys_manipulator_isaac_ros_config/srv/planning_command.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rviz_common/panel.hpp"

namespace movensys_manipulator_isaac_ros_config
{

class CuroboPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit CuroboPanel(QWidget * parent = nullptr);
  void onInitialize() override;

private Q_SLOTS:
  void refresh();
  void onStartActivated(int index);
  void onGoalActivated(int index);
  void onFollowClicked(bool checked);

private:
  using Status = msg::PlannerStatus;
  using Command = srv::PlanningCommand;

  void send(const std::string & command, const std::string & name = "");
  static void fillCombo(
    QComboBox * combo, const std::vector<std::string> & items, const std::string & current);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<Command>::SharedPtr client_;
  rclcpp::Subscription<Status>::SharedPtr sub_;

  std::mutex mutex_;
  Status::SharedPtr status_;
  std::string reply_;
  bool replyError_ = false;
  rclcpp::Time statusTime_{0, 0, RCL_STEADY_TIME};

  QComboBox * start_;
  QComboBox * goal_;
  QPushButton * plan_;
  QPushButton * execute_;
  QPushButton * planExecute_;
  QPushButton * stop_;
  QCheckBox * follow_;
  QLabel * status_label_;
  QLabel * info_;
  QTimer * timer_;
};

}  // namespace movensys_manipulator_isaac_ros_config

#endif  // MOVENSYS_MANIPULATOR_ISAAC_ROS_CONFIG__CUROBO_PANEL_HPP_
