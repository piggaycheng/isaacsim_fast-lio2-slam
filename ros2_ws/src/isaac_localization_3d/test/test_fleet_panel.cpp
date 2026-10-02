#include <array>
#include <chrono>
#include <functional>
#include <iostream>
#include <stdexcept>

#include <QApplication>
#include <QComboBox>
#include <QMouseEvent>
#include <QPushButton>
#include <QTest>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/config.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/properties/property.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction.hpp>
#include <rviz_common/tool.hpp>
#include <rviz_common/tool_manager.hpp>
#include <rviz_common/visualization_frame.hpp>
#include <rviz_common/visualization_manager.hpp>

void require(bool condition, const char * message)
{
  if (!condition) {
    throw std::runtime_error(message);
  }
}

int main(int argc, char ** argv)
{
  if (argc != 2) {
    std::cerr << "Usage: test_fleet_panel CONFIG.rviz (robots: carter1, carter2)\n";
    return 2;
  }
  rclcpp::init(argc, argv);
  QApplication app(argc, argv);
  auto rviz_node = std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>(
    "fleet_panel_test_rviz");
  auto probe = std::make_shared<rclcpp::Node>("fleet_panel_test_probe");
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
        throw std::runtime_error("Timed out waiting for RViz or a pose message");
      };
    auto * robots = frame.findChild<QComboBox *>("fleet_robot");
    auto * goal = frame.findChild<QPushButton *>("fleet_goal");
    auto * pose = frame.findChild<QPushButton *>("fleet_initial_pose");
    require(robots && goal && pose, "Fleet panel plugin did not load");
    std::cout << "Waiting for enabled fleet controls\n" << std::flush;
    wait([&]() { return goal->isEnabled() && pose->isEnabled(); });
    require(robots->count() == 2, "Robot list not loaded");
    auto * manager = frame.getManager()->getToolManager();
    auto * render = frame.getManager()->getRenderPanel();
    std::array<int, 2> goals{0, 0}, poses{0, 0};
    std::array<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr, 2> goal_subs;
    std::array<rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr, 2>
      pose_subs;
    for (int i = 0; i < 2; ++i) {
      const std::string ns = "/carter" + std::to_string(i + 1);
      goal_subs[i] = probe->create_subscription<geometry_msgs::msg::PoseStamped>(
        ns + "/goal_pose", 10, [&, i](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
          require(msg->header.frame_id == "map", "Wrong goal frame");
          ++goals[i];
        });
      pose_subs[i] =
        probe->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        ns + "/initialpose", 10,
        [&, i](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg) {
          require(msg->header.frame_id == "map", "Wrong initial-pose frame");
          ++poses[i];
        });
    }
    auto drag = [&]() {
        const QPoint start(render->width() / 2, render->height() / 2);
        const QPoint end = start + QPoint(40, 0);
        QTest::mousePress(render, Qt::LeftButton, Qt::NoModifier, start);
        QMouseEvent move(
          QEvent::MouseMove, end, Qt::NoButton, Qt::LeftButton, Qt::NoModifier);
        QApplication::sendEvent(render, &move);
        QTest::mouseRelease(render, Qt::LeftButton, Qt::NoModifier, end);
      };
    for (const int i : {0, 1, 0}) {
      robots->setCurrentIndex(i);
      goal->click();
      std::cout << "Waiting for goal publisher " << i << "\n" << std::flush;
      require(manager->getCurrentTool()->getPropertyContainer()->subProp("Topic")
        ->getValue().toString() == robots->currentText().prepend("/").append("/goal_pose"),
        "Goal topic not switched");
      wait([&]() { return goal_subs[i]->get_publisher_count() > 0; });
      const auto before = goals;
      drag();
      std::cout << "Waiting for goal message " << i << "\n" << std::flush;
      wait([&]() { return goals[i] == before[i] + 1; });
      require(goals[1 - i] == before[1 - i], "Goal sent to another robot");
      pose->click();
      std::cout << "Waiting for initial-pose publisher " << i << "\n" << std::flush;
      wait([&]() { return pose_subs[i]->get_publisher_count() > 0; });
      const auto pose_before = poses;
      drag();
      std::cout << "Waiting for initial-pose message " << i << "\n" << std::flush;
      wait([&]() { return poses[i] == pose_before[i] + 1; });
      require(poses[1 - i] == pose_before[1 - i], "Initial pose sent to another robot");
    }
    goal->click();
    QTest::mousePress(render, Qt::LeftButton, Qt::NoModifier,
      QPoint(render->width() / 2, render->height() / 2));
    const auto before = goals;
    robots->setCurrentIndex(1);
    require(manager->getCurrentTool() == manager->getDefaultTool(),
      "Switching robot did not abort the unfinished goal");
    QTest::mouseRelease(render, Qt::LeftButton);
    QTest::qWait(250);
    rclcpp::spin_some(probe);
    require(goals == before, "Switching robot emitted an unfinished goal");
    auto * panel = qobject_cast<rviz_common::Panel *>(robots->parentWidget());
    require(panel, "Fleet panel widget not found");
    rviz_common::Config saved;
    panel->save(saved);
    robots->setCurrentIndex(0);
    panel->load(saved);
    require(robots->currentText() == "carter2", "Selected robot was not restored");
    goal->click();
    require(manager->getCurrentTool()->getPropertyContainer()->subProp("Topic")
      ->getValue().toString() == "/carter2/goal_pose", "Restored robot has wrong topic");
    rviz_common::Config empty;
    panel->load(empty);
    require(!goal->isEnabled() && !pose->isEnabled(), "Missing config silently enabled controls");
    std::cout << "FLEET_PANEL PASSED: routing, switching mid-drag, save/load, missing config\n";
    frame.close();
  } catch (const std::exception & error) {
    std::cerr << "FLEET_PANEL FAILED: " << error.what() << "\n";
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
