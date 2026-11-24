#include "movensys_manipulator_isaac_ros_config/curobo_panel.hpp"

#include <algorithm>

#include <QAbstractItemView>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QSignalBlocker>
#include <QVBoxLayout>

#include "pluginlib/class_list_macros.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"

namespace movensys_manipulator_isaac_ros_config
{

namespace
{
constexpr char kCommandService[] = "/curobo/command";
constexpr char kStatusTopic[] = "/curobo/planner_status";
constexpr double kStaleSeconds = 2.0;
}  // namespace

CuroboPanel::CuroboPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  auto * query = new QGroupBox("Query");
  auto * form = new QFormLayout(query);
  start_ = new QComboBox;
  goal_ = new QComboBox;
  start_->setToolTip("State the plan starts from. Execute needs <current>.");
  goal_->setToolTip("<marker>: drag the goal marker. <current>: the robot's state.");
  form->addRow("Start State:", start_);
  form->addRow("Goal State:", goal_);

  auto * commands = new QGroupBox("Commands");
  auto * grid = new QGridLayout(commands);
  plan_ = new QPushButton("Plan");
  execute_ = new QPushButton("Execute");
  planExecute_ = new QPushButton("Plan && Execute");
  stop_ = new QPushButton("Stop");
  follow_ = new QCheckBox("Live follow (track the marker with MPC)");
  grid->addWidget(plan_, 0, 0);
  grid->addWidget(execute_, 0, 1);
  grid->addWidget(planExecute_, 1, 0);
  grid->addWidget(stop_, 1, 1);
  grid->addWidget(follow_, 2, 0, 1, 2);

  auto * statusBox = new QGroupBox("Status");
  auto * statusLayout = new QVBoxLayout(statusBox);
  status_label_ = new QLabel("Waiting for curobo_node...");
  status_label_->setWordWrap(true);
  info_ = new QLabel;
  info_->setWordWrap(true);
  statusLayout->addWidget(status_label_);
  statusLayout->addWidget(info_);

  auto * layout = new QVBoxLayout(this);
  layout->addWidget(query);
  layout->addWidget(commands);
  layout->addWidget(statusBox);
  layout->addStretch(1);

  connect(plan_, &QPushButton::clicked, this, [this]() {send(Command::Request::PLAN);});
  connect(execute_, &QPushButton::clicked, this, [this]() {send(Command::Request::EXECUTE);});
  connect(
    planExecute_, &QPushButton::clicked, this,
    [this]() {send(Command::Request::PLAN_EXECUTE);});
  connect(stop_, &QPushButton::clicked, this, [this]() {send(Command::Request::STOP);});
  connect(follow_, &QCheckBox::clicked, this, &CuroboPanel::onFollowClicked);
  connect(
    start_, QOverload<int>::of(&QComboBox::activated), this, &CuroboPanel::onStartActivated);
  connect(
    goal_, QOverload<int>::of(&QComboBox::activated), this, &CuroboPanel::onGoalActivated);

  timer_ = new QTimer(this);
  connect(timer_, &QTimer::timeout, this, &CuroboPanel::refresh);
}

void CuroboPanel::onInitialize()
{
  node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  client_ = node_->create_client<Command>(kCommandService);
  sub_ = node_->create_subscription<Status>(
    kStatusTopic, rclcpp::QoS(1).transient_local().reliable(),
    [this](Status::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      status_ = msg;
      statusTime_ = rclcpp::Clock(RCL_STEADY_TIME).now();
    });
  timer_->start(100);
  refresh();
}

void CuroboPanel::send(const std::string & command, const std::string & name)
{
  if (!client_ || !client_->service_is_ready()) {
    std::lock_guard<std::mutex> lock(mutex_);
    reply_ = "curobo_node is not running (" + std::string(kCommandService) + ")";
    replyError_ = true;
    return;
  }
  auto request = std::make_shared<Command::Request>();
  request->command = command;
  request->name = name;
  client_->async_send_request(
    request, [this](rclcpp::Client<Command>::SharedFuture future) {
      const auto response = future.get();
      std::lock_guard<std::mutex> lock(mutex_);
      replyError_ = !response->success;
      reply_ = response->success ? "" : response->message;
    });
}

void CuroboPanel::fillCombo(
  QComboBox * combo, const std::vector<std::string> & items, const std::string & current)
{
  const QSignalBlocker blocker(combo);
  bool same = combo->count() == static_cast<int>(items.size());
  for (int i = 0; same && i < combo->count(); ++i) {
    same = combo->itemText(i).toStdString() == items[i];
  }
  if (!same) {
    combo->clear();
    for (const auto & item : items) {
      combo->addItem(QString::fromStdString(item));
    }
  }
  const int index = combo->findText(QString::fromStdString(current));
  if (index >= 0 && !combo->view()->isVisible()) {
    combo->setCurrentIndex(index);
  }
}

void CuroboPanel::refresh()
{
  Status::SharedPtr status;
  std::string reply;
  bool replyError = false;
  bool stale = true;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    status = status_;
    reply = reply_;
    replyError = replyError_;
    if (status) {
      stale = (rclcpp::Clock(RCL_STEADY_TIME).now() - statusTime_).seconds() > kStaleSeconds;
    }
  }

  const bool online = status && !stale && client_ && client_->service_is_ready();
  if (!online) {
    status_label_->setText("Waiting for curobo_node...");
    status_label_->setStyleSheet("color: gray;");
    info_->clear();
    for (QWidget * w : {static_cast<QWidget *>(start_), static_cast<QWidget *>(goal_),
        static_cast<QWidget *>(plan_), static_cast<QWidget *>(execute_),
        static_cast<QWidget *>(planExecute_), static_cast<QWidget *>(stop_),
        static_cast<QWidget *>(follow_)})
    {
      w->setEnabled(false);
    }
    return;
  }

  fillCombo(start_, status->start_states, status->start_state);
  fillCombo(goal_, status->goal_states, status->goal_state);

  const bool idle = !status->moving && !status->planning;
  start_->setEnabled(idle && !status->live_follow);
  goal_->setEnabled(!status->planning);
  plan_->setEnabled(idle && status->goal_ok && !status->live_follow);
  execute_->setEnabled(idle && status->can_execute && !status->live_follow);
  planExecute_->setEnabled(
    idle && status->goal_ok && !status->live_follow && status->start_state == "<current>");
  stop_->setEnabled(status->moving || status->planning || status->live_follow);
  follow_->setEnabled(!status->planning);
  {
    const QSignalBlocker blocker(follow_);
    follow_->setChecked(status->live_follow);
  }

  static const char * colors[] = {"", "color: #2e9e2e;", "color: #c98a00;", "color: #d03030;"};
  status_label_->setText(QString::fromStdString(status->text));
  status_label_->setStyleSheet(colors[std::min<int>(status->level, 3)]);

  QString info;
  if (status->has_plan) {
    info = QString("Plan: %1 s").arg(status->plan_duration, 0, 'f', 2);
    if (!status->can_execute) {
      info += " (preview only: start is not the current state)";
    }
  }
  if (replyError && !reply.empty()) {
    info += (info.isEmpty() ? "" : "\n") + QString::fromStdString(reply);
  }
  info_->setText(info);
}

void CuroboPanel::onStartActivated(int index)
{
  send(Command::Request::SET_START, start_->itemText(index).toStdString());
}

void CuroboPanel::onGoalActivated(int index)
{
  send(Command::Request::SET_GOAL, goal_->itemText(index).toStdString());
}

void CuroboPanel::onFollowClicked(bool checked)
{
  send(Command::Request::LIVE_FOLLOW, checked ? "on" : "off");
}

}  // namespace movensys_manipulator_isaac_ros_config

PLUGINLIB_EXPORT_CLASS(movensys_manipulator_isaac_ros_config::CuroboPanel, rviz_common::Panel)
