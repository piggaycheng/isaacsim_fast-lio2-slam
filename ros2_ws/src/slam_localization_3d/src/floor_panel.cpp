#include <QComboBox>
#include <QLabel>
#include <QLineEdit>
#include <QMetaObject>
#include <QPointer>
#include <QPushButton>
#include <QRegularExpression>
#include <QRegularExpressionValidator>
#include <QVBoxLayout>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/config.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/display_group.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <std_msgs/msg/string.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace slam_localization_3d
{

// Sends change_floor through floor_command_bridge, which publishes it to the MQTT broker
// exactly like an upper system; the map is then relocalized with 2D Pose Estimate.
class FloorPanel : public rviz_common::Panel
{
public:
  explicit FloorPanel(QWidget * parent = nullptr)
  : Panel(parent)
  {
    auto * layout = new QVBoxLayout(this);
    current_ = new QLabel("Current floor: unknown", this);
    current_->setObjectName("floor_current");
    layout->addWidget(current_);
    floors_ = new QComboBox(this);
    floors_->setObjectName("floor_name");
    floors_->setEditable(true);
    floors_->setInsertPolicy(QComboBox::NoInsert);
    floors_->lineEdit()->setValidator(new QRegularExpressionValidator(
        QRegularExpression("[A-Za-z0-9_-]+"), floors_));
    floors_->addItems({"1F", "B1"});
    layout->addWidget(floors_);
    auto * prepare = new QPushButton("Prepare elevator travel (MQTT)", this);
    prepare->setObjectName("floor_prepare");
    layout->addWidget(prepare);
    connect(prepare, &QPushButton::clicked, this, [this]() { sendRequest(true); });
    send_ = new QPushButton("Switch floor (MQTT)", this);
    send_->setObjectName("floor_switch");
    layout->addWidget(send_);
    status_ = new QLabel(
      "After the new map appears, set the pose with 2D Pose Estimate.", this);
    status_->setObjectName("floor_status");
    status_->setWordWrap(true);
    layout->addWidget(status_);
    transition_ = new QLabel("Transition: waiting for floor manager", this);
    transition_->setObjectName("floor_transition");
    layout->addWidget(transition_);
    layout->addStretch();
    connect(send_, &QPushButton::clicked, this, [this]() { sendRequest(); });
  }

  void onInitialize() override
  {
    auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
    request_ = node->create_publisher<std_msgs::msg::String>("navigation/floor_request", 1);
    prepare_request_ = node->create_publisher<std_msgs::msg::String>(
      "navigation/floor_prepare_request", 1);
    const auto latched = rclcpp::QoS(1).transient_local();
    current_sub_ = node->create_subscription<std_msgs::msg::String>(
      "navigation/current_floor", latched, [this](std_msgs::msg::String::ConstSharedPtr msg) {
        const QString floor = QString::fromStdString(msg->data);
        QMetaObject::invokeMethod(this, [this, floor]() {
          current_->setText("Current floor: " + (floor.isEmpty() ? "switching" : floor));
        }, Qt::QueuedConnection);
      });
    state_sub_ = node->create_subscription<std_msgs::msg::String>(
      "navigation/floor_transition_state", latched,
      [this](std_msgs::msg::String::ConstSharedPtr msg) {
        const QString state = QString::fromStdString(msg->data);
        QMetaObject::invokeMethod(this, [this, state]() {
          transition_->setText("Transition: " + state);
          updateNavigationDisplays(state != "idle");
        }, Qt::QueuedConnection);
      });
    status_sub_ = node->create_subscription<std_msgs::msg::String>(
      "navigation/floor_command_status", latched,
      [this](std_msgs::msg::String::ConstSharedPtr msg) {
        const QString text = QString::fromStdString(msg->data);
        QMetaObject::invokeMethod(
          this, [this, text]() { status_->setText(text); }, Qt::QueuedConnection);
      });
  }

  void load(const rviz_common::Config & config) override
  {
    Panel::load(config);
    const auto names = config.mapGetChild("Floors");
    if (names.listLength() > 0) {
      floors_->clear();
      for (int i = 0; i < names.listLength(); ++i) {
        const QString name = names.listChildAt(i).getValue().toString();
        if (!name.isEmpty() && floors_->findText(name) < 0) {
          floors_->addItem(name);
        }
      }
    }
  }

  void save(rviz_common::Config config) const override
  {
    Panel::save(config);
    auto names = config.mapMakeChild("Floors");
    for (int i = 0; i < floors_->count(); ++i) {
      names.listAppendNew().setValue(floors_->itemText(i));
    }
  }

private:
  void hideNavigationDisplays(rviz_common::DisplayGroup * group)
  {
    for (int i = 0; i < group->numDisplays(); ++i) {
      auto * display = group->getDisplayAt(i);
      if (auto * child = dynamic_cast<rviz_common::DisplayGroup *>(display)) {
        hideNavigationDisplays(child);
        continue;
      }
      rviz_common::Config config;
      display->save(config);
      QString scheme;
      config.mapGetString("Color Scheme", &scheme);
      const auto type = display->getClassId();
      if ((type == "rviz_default_plugins/Map" && scheme == "costmap") ||
        type == "rviz_default_plugins/Path" ||
        type == "rviz_default_plugins/Polygon" ||
        type == "rviz_default_plugins/LaserScan" ||
        type == "rviz_default_plugins/PoseWithCovariance")
      {
        hidden_displays_.push_back({display, display->isEnabled()});
        display->setEnabled(false);
        display->reset();
      }
    }
  }

  void updateNavigationDisplays(bool switching)
  {
    if (switching == switching_) {
      return;
    }
    switching_ = switching;
    if (switching) {
      hideNavigationDisplays(getDisplayContext()->getRootDisplayGroup());
    } else {
      for (const auto & saved : hidden_displays_) {
        if (saved.display) {
          saved.display->reset();
          saved.display->setEnabled(saved.enabled);
        }
      }
      hidden_displays_.clear();
    }
  }

  void sendRequest(bool prepare = false)
  {
    const QString floor = floors_->currentText().trimmed();
    auto publisher = prepare ? prepare_request_ : request_;
    if ((!prepare && floor.isEmpty()) || !publisher) {
      return;
    }
    if (publisher->get_subscription_count() == 0) {
      status_->setText(
        "floor_command_bridge is not running: start run_nav.sh with --mqtt-host.");
      return;
    }
    std_msgs::msg::String message;
    message.data = floor.toStdString();
    publisher->publish(message);
    status_->setText(prepare ? "Preparing elevator travel ..." : "Requesting " + floor + " ...");
  }

  QLabel * current_;
  QComboBox * floors_;
  QPushButton * send_;
  QLabel * status_;
  QLabel * transition_;
  struct HiddenDisplay
  {
    QPointer<rviz_common::Display> display;
    bool enabled;
  };
  std::vector<HiddenDisplay> hidden_displays_;
  bool switching_ = false;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr request_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr prepare_request_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr current_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub_;
};

}  // namespace slam_localization_3d

PLUGINLIB_EXPORT_CLASS(slam_localization_3d::FloorPanel, rviz_common::Panel)
