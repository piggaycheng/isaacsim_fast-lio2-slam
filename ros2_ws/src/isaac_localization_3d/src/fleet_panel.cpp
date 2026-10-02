#include <QComboBox>
#include <QLabel>
#include <QPushButton>
#include <QSignalBlocker>
#include <QTimer>
#include <QVBoxLayout>

#include <rviz_common/config.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/properties/property.hpp>
#include <rviz_common/tool.hpp>
#include <rviz_common/tool_manager.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace isaac_localization_3d
{

class FleetPanel : public rviz_common::Panel
{
public:
  explicit FleetPanel(QWidget * parent = nullptr)
  : Panel(parent)
  {
    auto * layout = new QVBoxLayout(this);
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
    connect(manager_, &rviz_common::ToolManager::toolChanged, this,
      [this](rviz_common::Tool *) { updateTopics(); });
    // Tools are loaded after panels when opening an RViz config.
    QTimer::singleShot(0, this, [this]() { updateTopics(); });
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
    updateTopics();
    QTimer::singleShot(0, this, [this]() { updateTopics(); });
  }

  void save(rviz_common::Config config) const override
  {
    Panel::save(config);
    auto names = config.mapMakeChild("Robots");
    for (int i = 0; i < robots_->count(); ++i) {
      names.listAppendNew().setValue(robots_->itemText(i));
    }
    config.mapSetValue("Selected Robot", robots_->currentText());
  }

private:
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

  void updateTopics()
  {
    const auto robot = robots_->currentText();
    auto * goal = findTool("SetGoal");
    auto * pose = findTool("SetInitialPose");
    const bool ready = !robot.isEmpty() && goal && pose;
    goal_->setEnabled(ready);
    pose_->setEnabled(ready);
    if (!ready) {
      status_->setText("Fleet controls unavailable: load a fleet config with robots and pose tools.");
      return;
    }
    goal->getPropertyContainer()->subProp("Topic")->setValue("/" + robot + "/goal_pose");
    pose->getPropertyContainer()->subProp("Topic")->setValue("/" + robot + "/initialpose");
    status_->setText("Goals: /" + robot + "/goal_pose\nInitial pose: /" + robot + "/initialpose");
  }

  void activate(const QString & name)
  {
    updateTopics();
    if (!robots_->currentText().isEmpty()) {
      if (auto * tool = findTool(name)) {
        manager_->setCurrentTool(tool);
      }
    }
  }

  QComboBox * robots_;
  QLabel * status_;
  QPushButton * goal_;
  QPushButton * pose_;
  rviz_common::ToolManager * manager_ = nullptr;
};

}  // namespace isaac_localization_3d

PLUGINLIB_EXPORT_CLASS(isaac_localization_3d::FleetPanel, rviz_common::Panel)
