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
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/config.hpp>
#include <rviz_common/display_group.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/properties/property.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction.hpp>
#include <rviz_common/tool.hpp>
#include <rviz_common/tool_manager.hpp>
#include <rviz_common/visualization_frame.hpp>
#include <rviz_common/visualization_manager.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

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
  std::array<rclcpp::Publisher<std_msgs::msg::String>::SharedPtr, 2> floor_pubs, state_pubs;
  std::array<rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr, 2> map_pubs;
  std::array<rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr, 2> reference_pubs;
  auto manual_pose_pub = probe->create_publisher<std_msgs::msg::String>(
    "/carter2/navigation/initial_pose_floor", rclcpp::QoS(1).transient_local());
  auto publish = [](const auto & publisher, const std::string & value) {
      std_msgs::msg::String message;
      message.data = value;
      publisher->publish(message);
    };
  for (int i = 0; i < 2; ++i) {
    const auto ns = "/carter" + std::to_string(i + 1) + "/navigation/";
    floor_pubs[i] = probe->create_publisher<std_msgs::msg::String>(
      ns + "current_floor", rclcpp::QoS(1).transient_local());
    state_pubs[i] = probe->create_publisher<std_msgs::msg::String>(
      ns + "floor_transition_state", rclcpp::QoS(1).transient_local());
    publish(floor_pubs[i], "1F");
    publish(state_pubs[i], "idle");
    map_pubs[i] = probe->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/fleet/maps/floor_" + std::to_string(i) + "/map", rclcpp::QoS(1).transient_local());
    nav_msgs::msg::OccupancyGrid map;
    map.header.frame_id = "map";
    map.header.stamp = probe->now();
    map.info.resolution = 0.1;
    map.info.width = i + 2;
    map.info.height = 1;
    map.info.origin.orientation.w = 1.0;
    map.data.assign(map.info.width, i == 0 ? 0 : 100);
    map_pubs[i]->publish(map);
    reference_pubs[i] = probe->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/fleet/maps/floor_" + std::to_string(i) + "/simulation_reference",
      rclcpp::QoS(1).transient_local());
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.ns = "simulation_reference/carter" + std::to_string(i + 1);
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.9;
    marker.scale.y = marker.scale.z = 0.14;
    marker.color.r = marker.color.a = 1.0;
    marker.color.g = 0.9;
    visualization_msgs::msg::MarkerArray reference;
    reference.markers.push_back(marker);
    reference_pubs[i]->publish(reference);
  }
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
    auto * floors = frame.findChild<QComboBox *>("fleet_floor");
    auto * goal = frame.findChild<QPushButton *>("fleet_goal");
    auto * pose = frame.findChild<QPushButton *>("fleet_initial_pose");
    require(robots && floors && goal && pose, "Fleet panel plugin did not load");
    require(floors->count() == 2 && floors->currentText() == "1F", "Floor list not loaded");
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
    auto * root = frame.getManager()->getRootDisplayGroup();
    auto display = [&](const QString & name) -> rviz_common::Display * {
        for (int i = 0; i < root->numDisplays(); ++i) {
          if (root->getDisplayAt(i)->getName() == name) {
            return root->getDisplayAt(i);
          }
        }
        throw std::runtime_error("Missing fleet display");
      };
    auto * first = display("carter1");
    auto * second = display("carter2");
    auto * map = display("Map");
    auto * reference = display("Simulation reference (approximate)");
    auto has_reference = [&](const QString & name) {
        auto * namespaces = reference->subProp("Namespaces");
        for (int i = 0; i < namespaces->numChildren(); ++i) {
          if (namespaces->childAt(i)->getName() == name) {
            return true;
          }
        }
        return false;
      };
    wait([&]() { return has_reference("simulation_reference/carter1"); });
    wait([&]() { return map->subProp("Width")->getValue().toInt() == 2; });
    require(first->isEnabled() && second->isEnabled(), "Same-floor robots were hidden");
    publish(floor_pubs[1], "B1");
    wait([&]() { return !second->isEnabled() && !goal->isEnabled(); });
    for (int i = 0; i < manager->numTools(); ++i) {
      if (manager->getTool(i)->getClassId() == "rviz_default_plugins/SetGoal") {
        manager->setCurrentTool(manager->getTool(i));
        break;
      }
    }
    require(manager->getCurrentTool() == manager->getDefaultTool(),
      "Toolbar allowed a goal on another floor");
    floors->setCurrentText("B1");
    wait([&]() { return !first->isEnabled() && second->isEnabled() && goal->isEnabled(); });
    require(map->subProp("Topic")->getValue().toString() == "/fleet/maps/floor_1/map",
      "Monitoring map topic did not switch");
    wait([&]() { return map->subProp("Width")->getValue().toInt() == 3; });
    require(reference->isEnabled(), "Reference hidden with another floor's robot group");
    require(reference->subProp("Topic")->getValue().toString() ==
      "/fleet/maps/floor_1/simulation_reference", "Reference topic did not follow selected floor");
    wait([&]() { return has_reference("simulation_reference/carter2"); });
    require(robots->currentText() == "carter2", "Floor switch changed selected robot");
    goal->click();
    QTest::mousePress(render, Qt::LeftButton, Qt::NoModifier,
      QPoint(render->width() / 2, render->height() / 2));
    const auto floor_before = goals;
    floors->setCurrentText("1F");
    require(manager->getCurrentTool() == manager->getDefaultTool(),
      "Floor switch did not abort unfinished goal");
    QTest::mouseRelease(render, Qt::LeftButton);
    QTest::qWait(250);
    rclcpp::spin_some(probe);
    require(goals == floor_before, "Floor switch emitted an unfinished goal");
    require(first->isEnabled() && !second->isEnabled(), "Returning to 1F did not restore groups");
    wait([&]() { return map->subProp("Width")->getValue().toInt() == 2; });
    first->setEnabled(false);
    floors->setCurrentText("B1");
    floors->setCurrentText("1F");
    require(!first->isEnabled(), "Floor switch lost manually disabled group preference");
    first->setEnabled(true);
    floors->setCurrentText("B1");
    publish(state_pubs[1], "prepared");
    publish(floor_pubs[1], "");
    wait([&]() { return !second->isEnabled() && !goal->isEnabled() && !pose->isEnabled(); });
    require(reference->isEnabled(), "Reference hidden before localization is available");
    publish(floor_pubs[1], "B1");
    publish(state_pubs[1], "failed");
    QTest::qWait(200);
    app.processEvents();
    require(!second->isEnabled() && !goal->isEnabled(), "Failed transition enabled controls");
    publish(state_pubs[1], "idle");
    wait([&]() { return second->isEnabled() && goal->isEnabled(); });
    publish(state_pubs[1], "switching");
    publish(floor_pubs[1], "");
    publish(manual_pose_pub, "B1");
    wait([&]() { return !goal->isEnabled() && pose->isEnabled() && !second->isEnabled(); });
    pose->click();
    require(manager->getCurrentTool()->getClassId() == "rviz_default_plugins/SetInitialPose",
      "Manual floor initialization tool was blocked");
    floors->setCurrentText("1F");
    require(!pose->isEnabled(), "Manual initial pose was allowed on the wrong floor");
    floors->setCurrentText("B1");
    require(pose->isEnabled(), "Manual initial pose was not restored on its target floor");
    publish(manual_pose_pub, "");
    publish(floor_pubs[1], "B1");
    publish(state_pubs[1], "idle");
    wait([&]() { return second->isEnabled() && goal->isEnabled(); });
    app.processEvents();
    require(!first->isEnabled(), "Other-floor robot became visible");
    require(probe->count_publishers("/carter1/navigation/floor_request") == 0 &&
      probe->count_publishers("/carter2/navigation/floor_request") == 0,
      "Monitoring panel published robot floor requests");
    auto * panel = qobject_cast<rviz_common::Panel *>(robots->parentWidget());
    require(panel, "Fleet panel widget not found");
    rviz_common::Config saved;
    panel->save(saved);
    bool saved_visibility = false;
    require(saved.mapGetChild("Robot Visibility").mapGetBool("carter1", &saved_visibility) &&
      saved_visibility, "Saved hidden group lost its original enabled preference");
    robots->setCurrentIndex(0);
    floors->setCurrentText("1F");
    panel->load(saved);
    require(robots->currentText() == "carter2", "Selected robot was not restored");
    require(floors->currentText() == "B1", "Selected floor was not restored");
    wait([&]() { return goal->isEnabled(); });
    goal->click();
    require(manager->getCurrentTool()->getPropertyContainer()->subProp("Topic")
      ->getValue().toString() == "/carter2/goal_pose", "Restored robot has wrong topic");
    rviz_common::Config empty;
    panel->load(empty);
    require(!goal->isEnabled() && !pose->isEnabled(), "Missing config silently enabled controls");
    std::cout << "FLEET_PANEL PASSED: simulation reference rendering, routing, floor filtering, transition guards, "
      "switching mid-drag, save/load, missing config\n";
    // Destruction releases RViz without the interactive save-on-close dialog.
    frame.hide();
  } catch (const std::exception & error) {
    std::cerr << "FLEET_PANEL FAILED: " << error.what() << "\n";
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
