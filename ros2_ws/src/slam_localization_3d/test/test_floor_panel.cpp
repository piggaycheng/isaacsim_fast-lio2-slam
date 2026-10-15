#include <chrono>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <string>

#include <QApplication>
#include <QComboBox>
#include <QLabel>
#include <QPushButton>
#include <QTest>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display_group.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction.hpp>
#include <rviz_common/visualization_frame.hpp>
#include <rviz_common/visualization_manager.hpp>
#include <std_msgs/msg/string.hpp>

void require(bool condition, const char * message)
{
  if (!condition) {
    throw std::runtime_error(message);
  }
}

int main(int argc, char ** argv)
{
  if (argc != 2) {
    std::cerr << "Usage: test_floor_panel global_fusion.rviz\n";
    return 2;
  }
  rclcpp::init(argc, argv);
  QApplication app(argc, argv);
  auto rviz_node = std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>(
    "floor_panel_test_rviz");
  auto probe = std::make_shared<rclcpp::Node>("floor_panel_test_probe");
  int result = 0;
  try {
    rviz_common::VisualizationFrame frame(rviz_node, nullptr);
    frame.initialize(rviz_node, argv[1]);
    frame.show();
    require(QTest::qWaitForWindowExposed(&frame, 10000), "RViz window was not exposed");
    auto wait = [&](const std::function<bool()> & condition) {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
        do {
          app.processEvents();
          rclcpp::spin_some(probe);
          if (condition()) {
            return;
          }
          QTest::qWait(20);
        } while (std::chrono::steady_clock::now() < deadline);
        throw std::runtime_error("Timed out waiting for the floor panel");
      };
    auto * floors = frame.findChild<QComboBox *>("floor_name");
    auto * send = frame.findChild<QPushButton *>("floor_switch");
    auto * prepare = frame.findChild<QPushButton *>("floor_prepare");
    auto * transition = frame.findChild<QLabel *>("floor_transition");
    auto * status = frame.findChild<QLabel *>("floor_status");
    auto * current = frame.findChild<QLabel *>("floor_current");
    require(floors && send && prepare && status && current && transition,
      "Floor panel plugin did not load");
    require(floors->count() == 2 && floors->itemText(1) == "B1", "Floor list not loaded");

    send->click();
    require(status->text().contains("not running"), "Missing bridge was not reported");

    std::string requested;
    auto request_sub = probe->create_subscription<std_msgs::msg::String>(
      "/navigation/floor_request", 1,
      [&](std_msgs::msg::String::ConstSharedPtr msg) { requested = msg->data; });
    const auto latched = rclcpp::QoS(1).transient_local();
    auto status_pub = probe->create_publisher<std_msgs::msg::String>(
      "/navigation/floor_command_status", latched);
    auto floor_pub = probe->create_publisher<std_msgs::msg::String>(
      "/navigation/current_floor", latched);
    auto state_pub = probe->create_publisher<std_msgs::msg::String>(
      "/navigation/floor_transition_state", latched);
    bool prepare_received = false;
    auto prepare_sub = probe->create_subscription<std_msgs::msg::String>(
      "/navigation/floor_prepare_request", 1,
      [&](std_msgs::msg::String::ConstSharedPtr) { prepare_received = true; });
    wait([&]() { return prepare_sub->get_publisher_count() > 0; });
    wait([&]() {
        prepare->click();
        return !status->text().contains("not running");
      });
    wait([&]() { return prepare_received; });
    wait([&]() { return request_sub->get_publisher_count() > 0; });
    wait([&]() {
        send->click();
        return send->isEnabled() && !status->text().contains("not running");
      });
    floors->setCurrentIndex(1);
    send->click();
    wait([&]() { return requested == "B1"; });

    std_msgs::msg::String message;
    message.data = "cmd 1 completed: Floor B1 maps, ICP and navigation ready";
    status_pub->publish(message);
    message.data = "B1";
    floor_pub->publish(message);
    wait([&]() {
        return status->text().contains("completed") && current->text() == "Current floor: B1";
      });
    auto * root = frame.getManager()->getRootDisplayGroup();
    auto find = [&](const QString & name) {
        for (int i = 0; i < root->numDisplays(); ++i) {
          if (root->getDisplayAt(i)->getName() == name) {
            return root->getDisplayAt(i);
          }
        }
        throw std::runtime_error("Missing test display");
      };
    auto * map = find("Office PGM");
    auto * global = find("Global costmap (optional)");
    auto * local = find("Local costmap (optional)");
    auto * path = find("Nav2 planned path (navigate mode)");
    auto * footprint = find("Global body footprint");
    auto * scan = find("Height-filtered obstacles");
    require(scan->isEnabled(), "Height-filtered obstacles should initially be enabled");
    local->setEnabled(false);
    for (int i = 0; i < 2; ++i) {
      const bool scan_enabled = i == 0;
      scan->setEnabled(scan_enabled);
      message.data = "preparing";
      state_pub->publish(message);
      wait([&]() { return transition->text() == "Transition: preparing"; });
      require(!scan->isEnabled(), "Preparation did not hide height-filtered obstacles");
      message.data = "prepared";
      state_pub->publish(message);
      wait([&]() { return transition->text() == "Transition: prepared"; });
      require(!global->isEnabled(), "Preparation did not hide navigation overlays");
      message.data = "";
      floor_pub->publish(message);
      wait([&]() { return current->text() == "Current floor: switching"; });
      require(map->isEnabled(), "Floor map was hidden");
      require(!global->isEnabled() && !local->isEnabled() &&
        !path->isEnabled() && !footprint->isEnabled() && !scan->isEnabled(),
        "Stale navigation overlays remain enabled");
      // A failed switch must not restore overlays while the floor is still unknown.
      message.data = "cmd 2 failed: localization timeout";
      status_pub->publish(message);
      wait([&]() { return status->text().contains("failed"); });
      message.data = "failed";
      state_pub->publish(message);
      wait([&]() { return transition->text() == "Transition: failed"; });
      require(!global->isEnabled(), "Failure restored stale costmap");
      require(!scan->isEnabled(), "Failure restored height-filtered obstacles");
      message.data = "";
      floor_pub->publish(message);
      QTest::qWait(100);
      app.processEvents();
      message.data = "1F";
      floor_pub->publish(message);
      message.data = "idle";
      state_pub->publish(message);
      wait([&]() { return current->text() == "Current floor: 1F"; });
      wait([&]() { return transition->text() == "Transition: idle"; });
      require(global->isEnabled() && path->isEnabled() && footprint->isEnabled(),
        "Navigation overlays were not restored");
      require(!local->isEnabled(), "User-disabled costmap was re-enabled");
      require(scan->isEnabled() == scan_enabled,
        "Height-filtered obstacle display preference was not restored");
    }
    std::cout << "FLOOR_PANEL PASSED: routing, status, overlay hiding and restoration\n";
    frame.close();
  } catch (const std::exception & error) {
    std::cerr << "FLOOR_PANEL FAILED: " << error.what() << "\n";
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
