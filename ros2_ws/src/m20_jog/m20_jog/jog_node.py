"""Qt jog panel for the M20: publishes geometry_msgs/Twist on /cmd_vel at a fixed rate.

Runs outside Isaac Sim, so a laggy simulator cannot delay the commands. Hold a button or key to move,
release to stop (a zero command is published). Keys: W/S (or Up/Down) forward/back, A/D (or Left/Right) strafe,
Q/E (or N/M) turn left/right, Space stop.
"""

import sys

import rclpy
from geometry_msgs.msg import Twist
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import QApplication, QDoubleSpinBox, QGridLayout, QLabel, QPushButton, QWidget

# key -> (axis index [vx, vy, wz], sign)
KEYS = {
    Qt.Key_W: (0, 1), Qt.Key_Up: (0, 1), Qt.Key_S: (0, -1), Qt.Key_Down: (0, -1),
    Qt.Key_A: (1, 1), Qt.Key_Left: (1, 1), Qt.Key_D: (1, -1), Qt.Key_Right: (1, -1),
    Qt.Key_Q: (2, 1), Qt.Key_N: (2, 1), Qt.Key_E: (2, -1), Qt.Key_M: (2, -1),
}  # fmt: skip


class JogWindow(QWidget):
    def __init__(self, node, topic: str, rate_hz: float, limits) -> None:
        super().__init__()
        self._node = node
        self._pub = node.create_publisher(Twist, topic, 1)
        self._held = {}  # source (key or button id) -> (axis, sign)
        self.setWindowTitle(f"M20 jog -> {topic}")
        self.setFocusPolicy(Qt.StrongFocus)

        layout = QGridLayout(self)
        self._spins = []
        for col, (label, default, maximum) in enumerate(zip(("vx (m/s)", "vy (m/s)", "wz (rad/s)"), (0.5, 0.2, 0.6), limits)):
            spin = QDoubleSpinBox()
            spin.setRange(0.0, maximum)
            spin.setSingleStep(0.05)
            spin.setValue(min(default, maximum))
            layout.addWidget(QLabel(label), 0, col, Qt.AlignCenter)
            layout.addWidget(spin, 1, col)
            self._spins.append(spin)

        # (text, row, col, axis, sign)
        for text, row, col, axis, sign in (
            ("Forward", 2, 1, 0, 1), ("Left", 3, 0, 1, 1), ("Right", 3, 2, 1, -1), ("Back", 4, 1, 0, -1),
            ("Turn left", 2, 0, 2, 1), ("Turn right", 2, 2, 2, -1),
        ):  # fmt: skip
            button = QPushButton(text)
            button.setMinimumSize(90, 60)
            button.setFocusPolicy(Qt.NoFocus)
            button.pressed.connect(lambda b=button, a=axis, s=sign: self._held.__setitem__(id(b), (a, s)))
            button.released.connect(lambda b=button: self._held.pop(id(b), None))
            layout.addWidget(button, row, col)
        stop = QPushButton("STOP")
        stop.setMinimumSize(90, 60)
        stop.setFocusPolicy(Qt.NoFocus)
        stop.clicked.connect(self._held.clear)
        layout.addWidget(stop, 3, 1)
        self._status = QLabel("vx=0.00 vy=0.00 wz=0.00")
        layout.addWidget(self._status, 5, 0, 1, 3, Qt.AlignCenter)

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._tick)
        self._timer.start(int(1000.0 / rate_hz))

    def keyPressEvent(self, event) -> None:
        if event.key() == Qt.Key_Space:
            self._held.clear()
        elif event.key() in KEYS and not event.isAutoRepeat():
            self._held[event.key()] = KEYS[event.key()]

    def keyReleaseEvent(self, event) -> None:
        if event.key() in KEYS and not event.isAutoRepeat():
            self._held.pop(event.key(), None)

    def focusOutEvent(self, event) -> None:
        self._held.clear()  # never keep moving after the window loses focus
        super().focusOutEvent(event)

    def closeEvent(self, event) -> None:
        self._held.clear()
        self._publish([0.0, 0.0, 0.0])
        super().closeEvent(event)

    def _publish(self, cmd) -> None:
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = cmd
        self._pub.publish(msg)

    def _tick(self) -> None:
        cmd = [0.0, 0.0, 0.0]
        for axis, sign in self._held.values():
            cmd[axis] += sign * self._spins[axis].value()
        # opposing inputs cancel; clamp to a single spin-box magnitude per axis
        cmd = [max(-s.value(), min(s.value(), c)) for c, s in zip(cmd, self._spins)]
        self._publish(cmd)
        self._status.setText("vx={:.2f} vy={:.2f} wz={:.2f}".format(*cmd))
        rclpy.spin_once(self._node, timeout_sec=0.0)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = rclpy.create_node("m20_jog")
    node.declare_parameter("cmd_vel_topic", "/cmd_vel")
    node.declare_parameter("rate_hz", 10.0)
    node.declare_parameter("max_vx", 1.0)
    node.declare_parameter("max_vy", 1.0)
    node.declare_parameter("max_wz", 1.0)
    p = node.get_parameter
    app = QApplication(sys.argv[:1])
    window = JogWindow(
        node, p("cmd_vel_topic").value, float(p("rate_hz").value),
        (float(p("max_vx").value), float(p("max_vy").value), float(p("max_wz").value)),
    )  # fmt: skip
    window.show()
    code = app.exec_()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(code)


if __name__ == "__main__":
    main()
