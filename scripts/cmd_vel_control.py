"""Fail-closed velocity receiver for Isaac Sim's native ROS 2 subscription."""

import math
import time


class CmdVelReceiver:
    def __init__(self, timeout=0.5, max_linear=1.0, max_angular=0.75):
        self.timeout = timeout
        self.max_linear = max_linear
        self.max_angular = max_angular
        self.last_command = (0.0, 0.0)
        self.last_received = None

    def accept_twist(self, linear, angular):
        values = (*linear, *angular)
        if (
            len(linear) != 3 or len(angular) != 3
            or not all(math.isfinite(value) for value in values)
            or any(value != 0 for value in (*linear[1:], *angular[:2]))
            or abs(linear[0]) > self.max_linear
            or abs(angular[2]) > self.max_angular
        ):
            self.last_command = (0.0, 0.0)
            self.last_received = None
            raise ValueError("Invalid or excessive ROS 2 cmd_vel")
        self.last_command = (linear[0], angular[2])
        self.last_received = time.monotonic()

    def command(self):
        if self.last_received is None or time.monotonic() - self.last_received > self.timeout:
            return (0.0, 0.0)
        return self.last_command


receiver = CmdVelReceiver()
_receivers = {"": receiver}


def receiver_for(name=""):
    """Per-robot receiver; the root namespace keeps the module-level receiver."""
    if name not in _receivers:
        _receivers[name] = CmdVelReceiver()
    return _receivers[name]
