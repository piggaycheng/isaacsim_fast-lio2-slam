#!/usr/bin/env python3

import copy
import hashlib
import json
import math
import os
import tempfile
from pathlib import Path

import rclpy
from geometry_msgs.msg import Point, PointStamped
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from nav2_msgs.msg import CostmapFilterInfo
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback, Marker

from costmap_zone_geometry import rasterize_zones, validate_polygon


class CostmapFilterEditor(Node):
    def __init__(self):
        super().__init__("costmap_filter_editor")
        self.state_file = Path(self.declare_parameter("state_file", "").value)
        if not self.state_file.name:
            raise ValueError("Editor requires a persistent state_file")
        self.map = None
        self.signature = None
        self.zones = []
        self.draft = []
        self.editing = None
        self.server = InteractiveMarkerServer(self, "costmap_filter_editor")
        self.status = self.create_publisher(String, "/costmap_filters/editor_status", 10)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.mask_publishers = {
            kind: self.create_publisher(OccupancyGrid, f"/costmap_filters/{kind}_mask", qos)
            for kind in ("keepout", "speed")
        }
        self.info_publishers = {
            kind: self.create_publisher(CostmapFilterInfo, f"/costmap_filters/{kind}_info", qos)
            for kind in ("keepout", "speed")
        }
        self.create_subscription(OccupancyGrid, "/map", self.on_map, qos)
        self.create_subscription(PointStamped, "/clicked_point", self.on_point, 10)
        self.menu = MenuHandler()
        self.menu.insert("Apply draft: Keepout", callback=lambda feedback: self.apply("keepout"))
        speed_menu = self.menu.insert("Apply draft: Speed")
        for percent in (5, 10, 25, 50, 75, 100):
            self.menu.insert(
                f"{percent}%", parent=speed_menu,
                callback=lambda feedback, percent=percent: self.apply("speed", percent),
            )
        self.menu.insert("Undo draft vertex", callback=lambda feedback: self.undo())
        self.menu.insert("Discard draft", callback=lambda feedback: self.discard())
        self.menu.insert("Edit this zone's vertices", callback=self.start_edit)
        self.menu.insert("Delete this zone", callback=self.delete)
        self.menu.insert("Finish vertex editing", callback=lambda feedback: self.finish_edit())
        clear = self.menu.insert("Delete all zones")
        self.menu.insert("Confirm delete all", parent=clear,
                         callback=lambda feedback: self.commit([]))

    def notify(self, text, error=False):
        self.status.publish(String(data=text))
        logger = self.get_logger()
        if error:
            logger.error(text)
        else:
            logger.info(text)
        marker = self.server.get("controls")
        if marker is not None:
            marker.description = f"Costmap zones: {text}"
            self.server.insert(marker)
            self.menu.apply(self.server, "controls")
            self.server.applyChanges()

    def on_map(self, message):
        if message.header.frame_id != "map" or not message.info.width or not message.info.height:
            raise ValueError("Editor needs a nonempty /map in map frame")
        orientation = message.info.origin.orientation
        if (abs(orientation.x) > 1e-9 or abs(orientation.y) > 1e-9
                or abs(orientation.z) > 1e-9 or abs(abs(orientation.w) - 1.0) > 1e-9):
            raise ValueError("Editor needs an axis-aligned map origin")
        info = message.info
        metadata = [info.width, info.height, info.resolution,
                    info.origin.position.x, info.origin.position.y]
        signature = hashlib.sha256(
            json.dumps(metadata).encode() + bytes((value + 1) & 255 for value in message.data)
        ).hexdigest()
        if self.signature is not None:
            if signature != self.signature:
                raise ValueError("Map changed while editor is active; restart with a matching state file")
            return
        self.map, self.signature = message, signature
        zones = []
        if self.state_file.exists():
            state = json.loads(self.state_file.read_text())
            if state["version"] != 1 or state["map_signature"] != signature:
                raise ValueError("Editor state belongs to another map; choose a different state_file")
            zones = state["zones"]
        # Invalid persisted zones or unwritable storage must not produce empty success masks.
        self.publish_candidate(zones)
        self.zones = zones
        self.refresh_markers()
        self.notify("Ready: use Publish Point for vertices, then Interact/right-click to apply a zone")

    def masks(self, zones):
        info = self.map.info
        return rasterize_zones(
            zones, info.width, info.height, info.resolution,
            (info.origin.position.x, info.origin.position.y),
        )

    def save(self, zones):
        self.state_file.parent.mkdir(parents=True, exist_ok=True)
        filename = None
        try:
            with tempfile.NamedTemporaryFile(
                mode="w", dir=self.state_file.parent, prefix=".costmap_zones_", delete=False,
            ) as stream:
                filename = stream.name
                json.dump({"version": 1, "map_signature": self.signature, "zones": zones}, stream)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(filename, self.state_file)
        finally:
            if filename is not None and os.path.exists(filename):
                os.unlink(filename)

    def publish_candidate(self, zones):
        masks = self.masks(zones)
        self.save(zones)
        stamp = self.get_clock().now().to_msg()
        for kind, data in zip(("keepout", "speed"), masks):
            message = OccupancyGrid()
            message.header.frame_id = "map"
            message.header.stamp = stamp
            message.info = copy.deepcopy(self.map.info)
            message.data = data.ravel().tolist()
            self.mask_publishers[kind].publish(message)
            info = CostmapFilterInfo()
            info.header = message.header
            info.type = 0 if kind == "keepout" else 1
            info.filter_mask_topic = f"/costmap_filters/{kind}_mask"
            info.base, info.multiplier = 0.0, 1.0
            self.info_publishers[kind].publish(info)

    def commit(self, zones):
        if self.map is None:
            self.notify("Wait for /map before editing zones", error=True)
            return False
        try:
            self.publish_candidate(zones)
        except (OSError, ValueError, TypeError, KeyError) as error:
            self.notify(f"Zone update rejected: {error}", error=True)
            return False
        self.zones = zones
        self.editing = None
        self.refresh_markers()
        self.notify(f"Saved and published {len(zones)} zones")
        return True

    def on_point(self, message):
        if self.map is None or message.header.frame_id != "map":
            self.notify("Wait for /map and set RViz Fixed Frame to map", error=True)
            return
        point = message.point
        if not math.isfinite(point.x) or not math.isfinite(point.y):
            self.notify("Clicked point must be finite", error=True)
            return
        info = self.map.info
        ox, oy = info.origin.position.x, info.origin.position.y
        if not (ox <= point.x < ox + info.width * info.resolution
                and oy <= point.y < oy + info.height * info.resolution):
            self.notify("Clicked point is outside /map", error=True)
            return
        self.draft.append([point.x, point.y])
        self.refresh_markers()
        self.notify(f"Draft: {len(self.draft)} vertices")

    def apply(self, kind, percent=100):
        try:
            points = validate_polygon(self.draft)
        except ValueError as error:
            self.notify(str(error), error=True)
            return
        identifier = max((zone["id"] for zone in self.zones), default=0) + 1
        zone = {"id": identifier, "kind": kind, "percent": percent, "points": points}
        if self.commit([*self.zones, zone]):
            self.draft = []
            self.refresh_markers()

    def undo(self):
        if self.draft:
            self.draft.pop()
        else:
            self.notify("Draft has no vertices to undo", error=True)
        self.refresh_markers()

    def discard(self):
        self.draft = []
        self.refresh_markers()

    def zone_id(self, feedback):
        if not feedback.marker_name.startswith("zone_"):
            self.notify("Select an existing zone for this action", error=True)
            return None
        identifier = int(feedback.marker_name.removeprefix("zone_"))
        if not any(zone["id"] == identifier for zone in self.zones):
            self.notify("This zone no longer exists", error=True)
            return None
        return identifier

    def delete(self, feedback):
        identifier = self.zone_id(feedback)
        if identifier is not None:
            self.commit([zone for zone in self.zones if zone["id"] != identifier])

    def start_edit(self, feedback):
        identifier = self.zone_id(feedback)
        if identifier is not None:
            self.editing = identifier
            self.refresh_markers()

    def finish_edit(self):
        self.editing = None
        self.refresh_markers()

    def move_vertex(self, feedback):
        if feedback.event_type != InteractiveMarkerFeedback.MOUSE_UP:
            return
        if feedback.header.frame_id != "map":
            self.notify("Vertex feedback must use map frame", error=True)
            self.refresh_markers()
            return
        _, identifier, index = feedback.marker_name.split("_")
        zones = copy.deepcopy(self.zones)
        for zone in zones:
            if zone["id"] == int(identifier):
                zone["points"][int(index)] = [feedback.pose.position.x, feedback.pose.position.y]
                break
        editing = self.editing
        self.commit(zones)
        self.editing = editing
        self.refresh_markers()

    def marker(self, name, label, points, color):
        interactive = InteractiveMarker()
        interactive.header.frame_id = "map"
        interactive.name, interactive.description = name, label
        interactive.scale = 0.5
        if points:
            interactive.pose.position.x = sum(point[0] for point in points) / len(points)
            interactive.pose.position.y = sum(point[1] for point in points) / len(points)
        interactive.pose.orientation.w = 1.0
        control = InteractiveMarkerControl()
        control.interaction_mode = InteractiveMarkerControl.MENU
        control.always_visible = True
        line = Marker(type=Marker.LINE_STRIP)
        line.pose.orientation.w = 1.0
        line.scale.x = 0.04
        line.color.r, line.color.g, line.color.b, line.color.a = (*color, 1.0)
        for x, y in [*points, *points[:1]]:
            line.points.append(Point(
                x=x - interactive.pose.position.x, y=y - interactive.pose.position.y, z=0.05,
            ))
        if points:
            control.markers.append(line)
        button = Marker(type=Marker.CUBE)
        button.pose.orientation.w = 1.0
        button.pose.position.z = 0.2
        button.scale.x = button.scale.y = button.scale.z = 0.25
        button.color = line.color
        control.markers.append(button)
        interactive.controls.append(control)
        self.server.insert(interactive)
        self.menu.apply(self.server, name)

    def refresh_markers(self):
        self.server.clear()
        self.marker("controls", "Costmap zones: right-click for actions", [], (0.2, 0.8, 1.0))
        if self.draft:
            self.marker("draft", "Draft polygon: right-click to apply", self.draft, (1.0, 1.0, 0.0))
        for zone in self.zones:
            label = f"Zone {zone['id']}: {zone['kind']}"
            if zone["kind"] == "speed":
                label += f" {zone['percent']}%"
            color = (1.0, 0.2, 0.2) if zone["kind"] == "keepout" else (0.2, 0.8, 1.0)
            self.marker(f"zone_{zone['id']}", label, zone["points"], color)
            if self.editing == zone["id"]:
                for index, (x, y) in enumerate(zone["points"]):
                    marker = InteractiveMarker()
                    marker.header.frame_id = "map"
                    marker.name = f"vertex_{zone['id']}_{index}"
                    marker.description = f"Vertex {index + 1}"
                    marker.scale = 0.4
                    marker.pose.position.x, marker.pose.position.y = x, y
                    marker.pose.orientation.w = 1.0
                    control = InteractiveMarkerControl()
                    control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
                    control.orientation.w = control.orientation.y = 2 ** -0.5
                    control.always_visible = True
                    sphere = Marker(type=Marker.SPHERE)
                    sphere.pose.orientation.w = 1.0
                    sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.15
                    sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = (*color, 1.0)
                    control.markers.append(sphere)
                    marker.controls.append(control)
                    self.server.insert(marker, feedback_callback=self.move_vertex)
        self.server.applyChanges()


def main():
    rclpy.init()
    node = None
    try:
        node = CostmapFilterEditor()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
