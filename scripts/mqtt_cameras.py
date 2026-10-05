"""Publish the camera list to an MQTT broker as one retained message.

New subscribers get the latest list immediately; the list is republished only when
a camera is enabled or disabled. A last-will message marks it offline if the
simulation dies without a clean shutdown.
"""

import json
import threading

import paho.mqtt.client as mqtt


class CameraCatalog:
    def __init__(self, host: str, port: int, topic: str, rtsp_port: int,
                 username: str | None = None, password: str | None = None):
        self.topic = topic
        self.rtsp_port = rtsp_port
        self.cameras: list[dict] = []
        self.lock = threading.Lock()
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="isaacsim-cameras")
        if username:
            self.client.username_pw_set(username, password)
        self.client.will_set(topic, self._payload(False, []), qos=1, retain=True)
        self.client.on_connect = self._on_connect
        self.client.reconnect_delay_set(min_delay=1, max_delay=10)
        # connect_async never blocks the simulation if the broker is down.
        self.client.connect_async(host, port, keepalive=30)
        self.client.loop_start()

    def _payload(self, online: bool, cameras: list[dict]) -> str:
        return json.dumps({"online": online, "rtsp_port": self.rtsp_port, "cameras": cameras})

    def _on_connect(self, client, userdata, flags, reason_code, properties) -> None:
        self._publish()

    def _publish(self) -> None:
        with self.lock:
            payload = self._payload(True, self.cameras)
        self.client.publish(self.topic, payload, qos=1, retain=True)

    def set_cameras(self, cameras: list[dict]) -> None:
        with self.lock:
            self.cameras = cameras
        self._publish()

    def update_state(self, states: dict[str, bool]) -> None:
        """Republish only when a camera's `enabled` flag changed. Keys are camera names."""
        changed = False
        with self.lock:
            for camera in self.cameras:
                enabled = states.get(camera["name"])
                if enabled is not None and camera["enabled"] != enabled:
                    camera["enabled"] = enabled
                    changed = True
        if changed:
            self._publish()

    def close(self) -> None:
        info = self.client.publish(self.topic, self._payload(False, []), qos=1, retain=True)
        info.wait_for_publish(timeout=2.0)
        self.client.loop_stop()
        self.client.disconnect()
