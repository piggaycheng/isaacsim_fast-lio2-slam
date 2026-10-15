#include <QComboBox>
#include <QLabel>
#include <QLineEdit>
#include <QMetaObject>
#include <QPushButton>
#include <QRegularExpression>
#include <QRegularExpressionValidator>
#include <QVBoxLayout>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/config.hpp>
#include <rviz_common/display_context.hpp>
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
    send_ = new QPushButton("Switch floor (MQTT)", this);
    send_->setObjectName("floor_switch");
    layout->addWidget(send_);
    status_ = new QLabel(
      "After the new map appears, set the pose with 2D Pose Estimate.", this);
    status_->setObjectName("floor_status");
    status_->setWordWrap(true);
    layout->addWidget(status_);
    layout->addStretch();
    connect(send_, &QPushButton::clicked, this, [this]() { sendRequest(); });
  }

  void onInitialize() override
  {
    auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
    request_ = node->create_publisher<std_msgs::msg::String>("navigation/floor_request", 1);
    const auto latched = rclcpp::QoS(1).transient_local();
    current_sub_ = node->create_subscription<std_msgs::msg::String>(
      "navigation/current_floor", latched, [this](std_msgs::msg::String::ConstSharedPtr msg) {
        const QString floor = QString::fromStdString(msg->data);
        QMetaObject::invokeMethod(this, [this, floor]() {
          current_->setText("Current floor: " + (floor.isEmpty() ? "switching" : floor));
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
  void sendRequest()
  {
    const QString floor = floors_->currentText().trimmed();
    if (floor.isEmpty() || !request_) {
      return;
    }
    if (request_->get_subscription_count() == 0) {
      status_->setText(
        "floor_command_bridge is not running: start run_nav.sh with --mqtt-host.");
      return;
    }
    std_msgs::msg::String message;
    message.data = floor.toStdString();
    request_->publish(message);
    status_->setText("Requesting " + floor + " ...");
  }

  QLabel * current_;
  QComboBox * floors_;
  QPushButton * send_;
  QLabel * status_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr request_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr current_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
};

}  // namespace slam_localization_3d

PLUGINLIB_EXPORT_CLASS(slam_localization_3d::FloorPanel, rviz_common::Panel)
