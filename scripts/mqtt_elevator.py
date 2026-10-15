"""Drive the simulated Office lift from MQTT, like a building lift controller.

The fields mirror Open-RMF's rmf_lift_msgs so a lift adapter only translates them.

Request, JSON on `<topic>/<lift>/request`:
    {"session_id": "carter1", "request_type": "agv" | "human" | "end_session",
     "destination_floor": "B1", "door_state": "open" | "closed"}
State, retained JSON on `<topic>/<lift>/state` (2 Hz and on every change):
    {"lift_name", "available_floors", "current_floor", "destination_floor",
     "door_state": "closed" | "moving" | "open", "motion_state": "stopped" | "up" | "down",
     "current_mode": "human" | "agv" | "offline", "session_id"}

One AGV session holds the lift: other sessions' requests are ignored until it sends
end_session. Doors only open once the cabin has stopped at the destination landing.
"""

import json
import os
import threading
import time

REQUEST_TYPES = ("agv", "human", "end_session")
DOOR_STATES = ("open", "closed")


def parse_floors(text):
    """'1F:0,B1:-3.3' -> {'1F': 0.0, 'B1': -3.3} (cabin height offset per landing)."""
    floors = {}
    for item in text.split(","):
        name, separator, height = item.strip().partition(":")
        if not name or not separator or name in floors:
            raise ValueError(f"Invalid lift floor {item!r}; expected NAME:HEIGHT[,NAME:HEIGHT]")
        floors[name] = float(height)
    if len(floors) < 2:
        raise ValueError("A lift needs at least two floors")
    return floors


class LiftController:
    """Session and landing logic on top of ElevatorDoors' move_to/set_doors."""

    def __init__(self, doors, name, floors):
        self.doors = doors
        self.name = name
        self.floors = dict(floors)
        self.session_id = ""
        self.destination = None
        self.door_request = "closed"
        self.pending = []
        self.lock = threading.Lock()

    def submit(self, request):
        """Validate a request from any thread; update() applies it in the sim loop."""
        if not isinstance(request, dict):
            raise ValueError("Lift request must be a JSON object")
        kind = request.get("request_type")
        session = request.get("session_id", "")
        if kind not in REQUEST_TYPES:
            raise ValueError(f"request_type must be one of {REQUEST_TYPES}")
        if not isinstance(session, str):
            raise ValueError("session_id must be a string")
        if kind != "end_session":
            if request.get("destination_floor") not in self.floors:
                raise ValueError(f"destination_floor must be one of {list(self.floors)}")
            if request.get("door_state", "open") not in DOOR_STATES:
                raise ValueError(f"door_state must be one of {DOOR_STATES}")
        with self.lock:
            self.pending.append(request)

    def _apply(self, request):
        session = request.get("session_id", "")
        if request["request_type"] == "end_session":
            if session == self.session_id:
                self.session_id = ""
            return
        if self.session_id and session != self.session_id:
            print(f"Lift {self.name}: busy with {self.session_id}; ignored {session}", flush=True)
            return
        if request["request_type"] == "agv":
            self.session_id = session
        self.destination = request["destination_floor"]
        self.door_request = request.get("door_state", "open")

    def update(self):
        with self.lock:
            requests, self.pending = self.pending, []
        for request in requests:
            self._apply(request)
        if self.destination is None:
            return
        height = self.floors[self.destination]
        if self.doors.lift_target != height or self.doors.moving:
            if not self.doors.moving:
                self.doors.move_to(height)
            return
        if self.doors.target_open != (self.door_request == "open"):
            self.doors.set_doors(self.door_request == "open")

    def floor_at(self, height):
        return min(self.floors, key=lambda name: abs(self.floors[name] - height))

    def state(self):
        doors = self.doors
        travel = doors.lift_target - doors.lift_height
        fraction = doors.fraction
        return {
            "lift_name": self.name,
            "available_floors": list(self.floors),
            "current_floor": self.floor_at(doors.lift_height),
            "destination_floor": self.floor_at(doors.lift_target),
            "door_state": "closed" if fraction <= 0.0 else "open" if fraction >= 1.0 else "moving",
            "motion_state": "stopped" if not travel else "up" if travel > 0 else "down",
            "current_mode": "agv" if self.session_id else "human",
            "session_id": self.session_id,
        }


class MqttLift:
    """MQTT transport for LiftController; never blocks the simulation loop."""

    def __init__(self, controller, host, port, topic="rmf/lift", username=None, password=None,
                 period=0.5, clock=time.monotonic):
        import paho.mqtt.client as mqtt

        self.controller = controller
        base = f"{topic.strip('/')}/{controller.name}"
        self.request_topic, self.state_topic = f"{base}/request", f"{base}/state"
        self.period, self.clock = period, clock
        self.last_state, self.last_publish = None, None
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2,
                                  client_id=f"isaacsim-lift-{controller.name}-{os.getpid()}")
        if username:
            self.client.username_pw_set(username, password)
        offline = dict(controller.state(), current_mode="offline")
        self.client.will_set(self.state_topic, json.dumps(offline), qos=1, retain=True)
        self.client.on_connect = self._on_connect
        self.client.on_message = self._on_message
        self.client.reconnect_delay_set(min_delay=1, max_delay=10)
        self.client.connect_async(host, port, keepalive=30)
        self.client.loop_start()
        print(f"Lift {controller.name} on mqtt://{host}:{port}/{base}/(request|state); "
              f"floors {controller.floors}", flush=True)

    def _on_connect(self, client, userdata, flags, reason_code, properties):
        client.subscribe(self.request_topic, qos=1)
        self.last_publish = None

    def _on_message(self, client, userdata, message):
        try:
            self.controller.submit(json.loads(message.payload))
        except (ValueError, UnicodeDecodeError) as error:
            print(f"Lift {self.controller.name}: rejected request: {error}", flush=True)

    def update(self):
        self.controller.update()
        state = self.controller.state()
        now = self.clock()
        if (state != self.last_state or self.last_publish is None
                or now - self.last_publish >= self.period):
            self.client.publish(self.state_topic, json.dumps(state), qos=0, retain=True)
            self.last_state, self.last_publish = state, now

    def close(self):
        offline = dict(self.controller.state(), current_mode="offline")
        try:
            self.client.publish(self.state_topic, json.dumps(offline), qos=1,
                                retain=True).wait_for_publish(2)
        except (RuntimeError, ValueError):
            pass  # Broker unreachable: the last will reports offline instead.
        self.client.loop_stop()
        self.client.disconnect()
