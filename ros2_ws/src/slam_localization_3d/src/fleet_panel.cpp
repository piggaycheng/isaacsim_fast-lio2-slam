#include <QComboBox>
#include <QLabel>
#include <QMap>
#include <QMetaObject>
#include <QPointer>
#include <QPushButton>
#include <QSignalBlocker>
#include <QTimer>
#include <QVBoxLayout>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/config.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/display_group.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/properties/property.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <rviz_common/tool.hpp>
#include <rviz_common/tool_manager.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <std_msgs/msg/string.hpp>

namespace slam_localization_3d
{

class FleetPanel : public rviz_common::Panel
{
public:
  explicit FleetPanel(QWidget * parent = nullptr)
  : Panel(parent)
  {
    auto * layout = new QVBoxLayout(this);
    layout->addWidget(new QLabel("Display floor (monitoring only):", this));
    floors_ = new QComboBox(this);
    floors_->setObjectName("fleet_floor");
    layout->addWidget(floors_);
    auto * reference_hint = new QLabel(
      "Yellow SIM REF ~ arrows: approximate simulator reference, not localization.", this);
    reference_hint->setWordWrap(true);
    layout->addWidget(reference_hint);
    layout->addWidget(new QLabel("Selected robot:", this));
    robots_ = new QComboBox(this);
    robots_->setObjectName("fleet_robot");
    layout->addWidget(robots_);
    status_ = new QLabel(this);
    status_->setWordWrap(true);
    layout->addWidget(status_);
    goal_ = new QPushButton("Set navigation goal", this);
    pose_ = new QPushButton("Set initial pose", this);
    goal_->setObjectName("fleet_goal");
    pose_->setObjectName("fleet_initial_pose");
    layout->addWidget(goal_);
    layout->addWidget(pose_);
    layout->addWidget(new QLabel("Click and drag on the map to set position and heading.", this));
    layout->addStretch();
    connect(floors_, &QComboBox::currentTextChanged, this, [this]() {
      cancelTool();
      refreshDisplays();
      updateTopics();
      Q_EMIT configChanged();
    });
    connect(robots_, &QComboBox::currentTextChanged, this, [this]() {
      if (manager_) {
        // Abort an unfinished click-and-drag before changing its destination robot.
        manager_->setCurrentTool(manager_->getDefaultTool());
      }
      updateTopics();
      Q_EMIT configChanged();
    });
    connect(goal_, &QPushButton::clicked, this, [this]() { activate("SetGoal"); });
    connect(pose_, &QPushButton::clicked, this, [this]() { activate("SetInitialPose"); });
    updateTopics();
  }

  void onInitialize() override
  {
    manager_ = getDisplayContext()->getToolManager();
    subscribeFloors();
    connect(manager_, &rviz_common::ToolManager::toolChanged, this,
      [this](rviz_common::Tool *) { updateTopics(); });
    // Tools are loaded after panels when opening an RViz config.
    auto * timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, [this]() {
      refreshDisplays();
      updateTopics();
    });
    timer->start(100);
  }

  void load(const rviz_common::Config & config) override
  {
    Panel::load(config);
    if (manager_) {
      manager_->setCurrentTool(manager_->getDefaultTool());
    }
    QString selected;
    config.mapGetString("Selected Robot", &selected);
    {
      const QSignalBlocker blocker(robots_);
      robots_->clear();
      const auto names = config.mapGetChild("Robots");
      for (int i = 0; i < names.listLength(); ++i) {
        const QString name = names.listChildAt(i).getValue().toString();
        if (!name.isEmpty() && robots_->findText(name) < 0) {
          robots_->addItem(name);
        }
      }
      const int index = robots_->findText(selected);
      if (index >= 0) {
        robots_->setCurrentIndex(index);
      }
    }
    {
      const QSignalBlocker blocker(floors_);
      floors_->clear();
      const auto names = config.mapGetChild("Floors");
      for (int i = 0; i < names.listLength(); ++i) {
        const auto floor = names.listChildAt(i).getValue().toString();
        if (!floor.isEmpty() && floors_->findText(floor) < 0) {
          floors_->addItem(floor);
        }
      }
      config.mapGetString("Selected Floor", &selected);
      const int index = floors_->findText(selected);
      if (index >= 0) {
        floors_->setCurrentIndex(index);
      }
    }
    restoreGroups();
    pending_enabled_.clear();
    const auto enabled = config.mapGetChild("Robot Visibility");
    for (int i = 0; i < robots_->count(); ++i) {
      bool value;
      const auto robot = robots_->itemText(i);
      if (enabled.mapGetBool(robot, &value)) {
        pending_enabled_.insert(robot, value);
      }
    }
    subscribeFloors();
    refreshDisplays();
    updateTopics();
    QTimer::singleShot(0, this, [this]() { refreshDisplays(); updateTopics(); });
  }

  void save(rviz_common::Config config) const override
  {
    Panel::save(config);
    auto names = config.mapMakeChild("Robots");
    for (int i = 0; i < robots_->count(); ++i) {
      names.listAppendNew().setValue(robots_->itemText(i));
    }
    config.mapSetValue("Selected Robot", robots_->currentText());
    auto floors = config.mapMakeChild("Floors");
    for (int i = 0; i < floors_->count(); ++i) {
      floors.listAppendNew().setValue(floors_->itemText(i));
    }
    config.mapSetValue("Selected Floor", floors_->currentText());
    auto enabled = config.mapMakeChild("Robot Visibility");
    if (manager_) {
      auto * root = getDisplayContext()->getRootDisplayGroup();
      for (int i = 0; i < root->numDisplays(); ++i) {
        auto * display = root->getDisplayAt(i);
        const auto robot = display->getName();
        if (robots_->findText(robot) >= 0) {
          const auto hidden = hidden_groups_.constFind(robot);
          enabled.mapSetValue(robot, hidden == hidden_groups_.constEnd() ?
            display->isEnabled() : hidden->enabled);
        }
      }
    }
  }

private:
  struct RobotFloor
  {
    QString floor;
    QString transition;
    QString initial_pose_floor;
  };

  struct HiddenGroup
  {
    QPointer<rviz_common::Display> display;
    bool enabled;
  };

  void cancelTool()
  {
    if (manager_) {
      manager_->setCurrentTool(manager_->getDefaultTool());
    }
  }

  void subscribeFloors()
  {
    subscriptions_.clear();
    robot_floors_.clear();
    if (!manager_) {
      return;
    }
    auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
    for (int i = 0; i < robots_->count(); ++i) {
      const auto robot = robots_->itemText(i);
      robot_floors_.insert(robot, RobotFloor{});
      for (const std::string field :
        {"current_floor", "floor_transition_state", "initial_pose_floor"})
      {
        const auto topic = "/" + robot.toStdString() + "/navigation/" + field;
        subscriptions_.push_back(node->create_subscription<std_msgs::msg::String>(
          topic, rclcpp::QoS(1).transient_local(),
          [this, robot, field](std_msgs::msg::String::ConstSharedPtr msg) {
            const auto value = QString::fromStdString(msg->data);
            QMetaObject::invokeMethod(this, [this, robot, field, value]() {
              auto it = robot_floors_.find(robot);
              if (it == robot_floors_.end()) {
                return;
              }
              if (field == "floor_transition_state") {
                it->transition = value;
              } else if (field == "initial_pose_floor") {
                it->initial_pose_floor = value;
              } else {
                it->floor = value;
              }
              refreshDisplays();
              updateTopics();
            }, Qt::QueuedConnection);
          }));
      }
    }
  }

  bool onDisplayFloor(const QString & robot) const
  {
    const auto it = robot_floors_.constFind(robot);
    return !floors_->currentText().isEmpty() && it != robot_floors_.constEnd() &&
           it->floor == floors_->currentText() && it->transition == "idle";
  }

  void resetDisplay(rviz_common::Display * display)
  {
    display->reset();
    if (auto * group = dynamic_cast<rviz_common::DisplayGroup *>(display)) {
      for (int i = 0; i < group->numDisplays(); ++i) {
        resetDisplay(group->getDisplayAt(i));
      }
    }
  }

  void restoreGroups()
  {
    for (const auto & saved : hidden_groups_) {
      if (saved.display) {
        resetDisplay(saved.display);
        saved.display->setEnabled(saved.enabled);
      }
    }
    hidden_groups_.clear();
  }

  void refreshDisplays()
  {
    if (!manager_) {
      return;
    }
    auto * root = getDisplayContext()->getRootDisplayGroup();
    for (int i = 0; i < root->numDisplays(); ++i) {
      auto * display = root->getDisplayAt(i);
      const auto name = display->getName();
      if (name == "Map" && display->getClassId() == "rviz_default_plugins/Map" &&
        floors_->currentIndex() >= 0)
      {
        auto * topic = display->subProp("Topic");
        const auto value = QString("/fleet/maps/floor_%1/map").arg(floors_->currentIndex());
        if (topic->getValue().toString() != value) {
          display->reset();
          topic->setValue(value);
        }
      }
      if (name == "Simulation reference (approximate)" &&
        display->getClassId() == "rviz_default_plugins/MarkerArray" && floors_->currentIndex() >= 0)
      {
        auto * topic = display->subProp("Topic");
        const auto value =
          QString("/fleet/maps/floor_%1/simulation_reference").arg(floors_->currentIndex());
        if (topic->getValue().toString() != value) {
          display->reset();
          topic->setValue(value);
        }
      }
      if (name == "Costmap zone editor" &&
        display->getClassId() == "rviz_default_plugins/InteractiveMarkers" &&
        !floors_->currentText().isEmpty())
      {
        // Matches floor_maps.fleet_zone_namespace(); one editor serves each floor.
        auto * property = display->subProp("Interactive Markers Namespace");
        const auto value = zoneNamespace() + "/costmap_filter_editor";
        if (property->getValue().toString() != value) {
          display->reset();
          property->setValue(value);
        }
      }
      if (robots_->findText(name) < 0 ||
        !dynamic_cast<rviz_common::DisplayGroup *>(display))
      {
        continue;
      }
      if (pending_enabled_.contains(name)) {
        display->setEnabled(pending_enabled_.take(name));
      }
      auto hidden = hidden_groups_.find(name);
      if (!onDisplayFloor(name)) {
        if (hidden == hidden_groups_.end() || hidden->display != display) {
          hidden_groups_.insert(name, HiddenGroup{display, display->isEnabled()});
          display->setEnabled(false);
          resetDisplay(display);
        } else {
          display->setEnabled(false);
        }
      } else if (hidden != hidden_groups_.end()) {
        const bool enabled = hidden->enabled;
        hidden_groups_.erase(hidden);
        resetDisplay(display);
        display->setEnabled(enabled);
      }
    }
  }

  rviz_common::Tool * findTool(const QString & name) const
  {
    if (manager_) {
      for (int i = 0; i < manager_->numTools(); ++i) {
        auto * tool = manager_->getTool(i);
        if (tool->getClassId() == "rviz_default_plugins/" + name) {
          return tool;
        }
      }
    }
    return nullptr;
  }

  QString zoneNamespace() const
  {
    return "/fleet/zones/floor_" + floors_->currentText();
  }

  void updateTopics()
  {
    if (auto * point = findTool("PublishPoint"); point && !floors_->currentText().isEmpty()) {
      const auto value = zoneNamespace() + "/clicked_point";
      auto * topic = point->getPropertyContainer()->subProp("Topic");
      if (topic->getValue().toString() != value) {
        topic->setValue(value);
      }
    }
    const auto robot = robots_->currentText();
    auto * goal = findTool("SetGoal");
    auto * pose = findTool("SetInitialPose");
    const auto floor = robot_floors_.value(robot);
    const bool tools_ready = !robot.isEmpty() && goal && pose;
    const bool ready = tools_ready && onDisplayFloor(robot);
    const bool manual_pose = tools_ready && !floors_->currentText().isEmpty() &&
      floor.transition == "switching" && floor.initial_pose_floor == floors_->currentText();
    const bool pose_ready = ready || manual_pose;
    goal_->setEnabled(ready);
    pose_->setEnabled(pose_ready);
    if (tools_ready) {
      goal->getPropertyContainer()->subProp("Topic")->setValue("/" + robot + "/goal_pose");
      pose->getPropertyContainer()->subProp("Topic")->setValue("/" + robot + "/initialpose");
    }
    if (manager_ && manager_->getCurrentTool() &&
      ((!ready && manager_->getCurrentTool() == goal) ||
      (!pose_ready && manager_->getCurrentTool() == pose)))
    {
      cancelTool();
    }
    if (!ready) {
      if (manual_pose) {
        status_->setText("Waiting for " + robot + " initial pose on " +
          floors_->currentText() + ". Navigation stays disabled.");
      } else if (tools_ready && floors_->count() > 0) {
        status_->setText(
          "Controls disabled: " + robot + " is on " +
          (floor.floor.isEmpty() ? "an unconfirmed floor" : floor.floor) +
          " (" + (floor.transition.isEmpty() ? "waiting for floor manager" : floor.transition) +
          "). Select its floor and wait for idle.");
      } else {
        status_->setText("Fleet controls unavailable: load robots, floors and pose tools.");
      }
      return;
    }
    status_->setText("Goals: /" + robot + "/goal_pose\nInitial pose: /" + robot + "/initialpose");
  }

  void activate(const QString & name)
  {
    updateTopics();
    if ((name == "SetGoal" && goal_->isEnabled()) ||
      (name == "SetInitialPose" && pose_->isEnabled()))
    {
      if (auto * tool = findTool(name)) {
        manager_->setCurrentTool(tool);
      }
    }
  }

  QComboBox * robots_;
  QComboBox * floors_;
  QLabel * status_;
  QPushButton * goal_;
  QPushButton * pose_;
  rviz_common::ToolManager * manager_ = nullptr;
  QMap<QString, RobotFloor> robot_floors_;
  QMap<QString, HiddenGroup> hidden_groups_;
  QMap<QString, bool> pending_enabled_;
  std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> subscriptions_;
};

}  // namespace slam_localization_3d

PLUGINLIB_EXPORT_CLASS(slam_localization_3d::FleetPanel, rviz_common::Panel)
