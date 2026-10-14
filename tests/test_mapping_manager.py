from contextlib import redirect_stderr
from collections import deque
import io
import json
from pathlib import Path
import socket
import struct
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import Mock
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from mapping_manager import MappingManager, MappingProcess, RosBackend, receive, validate_map


def write_map(directory):
    (directory / "map.pcd").write_bytes(
        b"FIELDS x y z\nSIZE 4 4 4\nCOUNT 1 1 1\nPOINTS 1\nDATA binary\n"
        + struct.pack("<fff", 1, 2, 3),
    )
    (directory / "poses.txt").write_text("0.pcd 0 0 0 1 0 0 0\n")
    (directory / "patches").mkdir(exist_ok=True)
    (directory / "patches/0.pcd").write_bytes((directory / "map.pcd").read_bytes())


class MappingManagerTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.backend = Mock()
        self.backend.wait_ready.return_value = {"floor_pose": [0, 0, 0]}
        self.backend.save.side_effect = write_map
        self.process = Mock()
        self.process.running.return_value = True
        self.manager = MappingManager(self.temp.name, self.backend, self.process)

    def test_independent_sessions_and_successful_save_then_stop(self):
        result = self.manager.start("floor_ground")
        self.assertEqual(result["state"], "mapping")
        self.manager.save_stop()
        directory = Path(self.temp.name) / "floor_ground"
        saved = json.loads((directory / "session.json").read_text())
        self.assertEqual(saved["state"], "saved_stopped")
        self.assertEqual(saved["point_count"], 1)
        self.process.stop.assert_called_once()
        self.assertEqual(self.manager.status()["state"], "idle")
        self.manager.start("floor_lower")
        self.backend.begin.assert_called_with()
        self.assertEqual(self.backend.begin.call_count, 2)
        self.assertEqual(self.process.start.call_count, 2)

    def test_existing_directory_and_path_traversal_are_rejected(self):
        directory = Path(self.temp.name) / "floor_ground"
        directory.mkdir()
        (directory / "map.pcd").write_bytes(b"do not overwrite")
        with self.assertRaises(FileExistsError):
            self.manager.start("floor_ground")
        self.assertEqual((directory / "map.pcd").read_bytes(), b"do not overwrite")
        for floor in ("../escape", "/tmp/escape", "", "x/y", "a" * 65, None):
            with self.subTest(floor=floor), self.assertRaises(ValueError):
                self.manager.start(floor)
        self.process.start.assert_not_called()

    def test_save_failure_keeps_mapping_alive_and_can_retry(self):
        self.manager.start("floor_ground")
        self.backend.save.side_effect = RuntimeError("NO POSES")
        with self.assertRaisesRegex(RuntimeError, "NO POSES"):
            self.manager.save_stop()
        self.process.stop.assert_not_called()
        self.assertEqual(self.manager.status()["state"], "mapping")
        self.backend.save.side_effect = write_map
        self.manager.save_stop()
        self.process.stop.assert_called_once()

    def test_success_response_with_invalid_artifacts_does_not_stop(self):
        self.manager.start("floor_ground")
        self.backend.save.side_effect = None
        with self.assertRaises(OSError):
            self.manager.save_stop()
        self.process.stop.assert_not_called()
        self.assertEqual(self.manager.status()["state"], "mapping")

    def test_start_failure_cleans_process_and_preserves_diagnostics(self):
        self.backend.wait_ready.side_effect = TimeoutError("No sensors")
        with self.assertRaisesRegex(TimeoutError, "No sensors"):
            self.manager.start("floor_ground")
        self.process.stop.assert_called_once()
        self.assertIsNone(self.manager.session)
        self.assertEqual(self.manager.last_session["state"], "start_failed")
        self.assertEqual(self.manager.last_session["error"], "No sensors")
        self.backend.wait_ready.side_effect = None
        self.manager.start("floor_lower")

    def test_unexpected_exit_is_not_reported_as_mapping(self):
        self.manager.start("floor_ground")
        self.process.running.return_value = False
        status = self.manager.status()
        self.assertEqual(status["state"], "error")
        self.assertEqual(status["last_session"]["state"], "error")
        self.assertIn("exited unexpectedly", status["last_error"])
        self.process.stop.assert_called_once()

    def test_saved_session_can_retry_shutdown_without_saving_again(self):
        self.manager.start("floor_ground")
        self.process.stop.side_effect = RuntimeError("shutdown incomplete")
        with self.assertRaises(RuntimeError):
            self.manager.save_stop()
        self.process.stop.side_effect = None
        self.manager.save_stop()
        self.backend.save.assert_called_once()
        self.assertEqual(self.manager.state, "idle")

    def test_start_metadata_failure_does_not_leave_a_phantom_session(self):
        self.manager.persist = Mock(side_effect=OSError("disk full"))
        with self.assertRaisesRegex(OSError, "disk full"):
            self.manager.start("floor_ground")
        self.process.start.assert_not_called()
        self.assertIsNone(self.manager.session)
        self.assertEqual(self.manager.state, "idle")
        self.assertEqual(self.manager.last_session["error"], "disk full")

    def test_stopped_metadata_failure_can_retry_without_resaving(self):
        self.manager.start("floor_ground")
        persist = self.manager.persist
        calls = 0

        def fail_final_write():
            nonlocal calls
            calls += 1
            if calls == 2:
                raise OSError("disk full")
            persist()

        self.manager.persist = fail_final_write
        with self.assertRaisesRegex(OSError, "disk full"):
            self.manager.save_stop()
        self.process.running.return_value = False
        self.manager.save_stop()
        self.backend.save.assert_called_once()
        self.assertEqual(self.manager.state, "idle")

    def test_only_one_session_and_no_save_when_idle(self):
        with self.assertRaises(RuntimeError):
            self.manager.save_stop()
        self.manager.start("floor_ground")
        with self.assertRaises(RuntimeError):
            self.manager.start("floor_lower")
        self.assertFalse((Path(self.temp.name) / "floor_lower").exists())

    def test_truncated_binary_pcd_is_rejected(self):
        directory = Path(self.temp.name)
        write_map(directory)
        self.assertEqual(validate_map(directory), 1)
        data = (directory / "map.pcd").read_bytes()
        (directory / "map.pcd").write_bytes(data[:-1])
        with self.assertRaisesRegex(RuntimeError, "truncated"):
            validate_map(directory)

    def test_socket_protocol_requires_complete_json_command(self):
        sender, receiver = socket.socketpair()
        with sender, receiver:
            sender.sendall(b'{"command":"status"}\n')
            self.assertEqual(receive(receiver), {"command": "status"})
            sender.shutdown(socket.SHUT_WR)
            with self.assertRaisesRegex(ValueError, "Incomplete"):
                receive(receiver)

    def test_real_owned_process_group_is_stopped(self):
        process = MappingProcess()
        process.process = subprocess.Popen(
            [sys.executable, "-c", "import time; time.sleep(60)"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True,
        )
        self.addCleanup(process.stop)
        process.stop()
        self.assertIsNone(process.process)

    def test_close_records_interruption_without_saving(self):
        self.manager.start("floor_ground")
        with redirect_stderr(io.StringIO()) as output:
            self.manager.close()
        self.assertIn("not automatically saved", output.getvalue())
        self.backend.save.assert_not_called()
        self.process.stop.assert_called_once()
        metadata = json.loads((Path(self.temp.name) / "floor_ground/session.json").read_text())
        self.assertEqual(metadata["state"], "interrupted")
        self.assertIn("stopped_at", metadata)

    def test_reference_rejects_nonfinite_pose_and_zero_quaternion(self):
        message = SimpleNamespace(pose=SimpleNamespace(pose=SimpleNamespace(
            position=SimpleNamespace(x=0.0, y=0.0, z=0.0),
            orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=0.0),
        )))
        with self.assertRaisesRegex(ValueError, "invalid quaternion"):
            RosBackend.pose(message)
        message.pose.pose.orientation.w = 1.0
        message.pose.pose.position.x = float("nan")
        with self.assertRaisesRegex(ValueError, "non-finite"):
            RosBackend.pose(message)

    def test_initial_reference_survives_ground_truth_buffer_eviction(self):
        backend = RosBackend.__new__(RosBackend)
        backend.initial_reference = None
        backend.first_odom = SimpleNamespace(header=SimpleNamespace(
            stamp=SimpleNamespace(sec=1, nanosec=0),
        ))
        backend.ground_truth = [backend.first_odom]
        backend.sensor_frames = {}
        backend.pose = Mock(return_value={"position": [0, 0, 0]})
        reference = backend.reference()
        backend.ground_truth = [SimpleNamespace(header=SimpleNamespace(
            stamp=SimpleNamespace(sec=10, nanosec=0),
        ))]
        self.assertIs(backend.reference(), reference)

    def reference_backend(self):
        backend = RosBackend.__new__(RosBackend)
        backend.initial_reference = None
        backend.first_odom = None
        backend.ground_truth = deque()
        backend.collecting = True
        backend.collection_deadline = time.monotonic() + backend.STARTUP_TIMEOUT
        backend.sensor_frames = {}
        backend.pose = Mock(return_value={"position": [0, 0, 0]})
        return backend

    @staticmethod
    def stamped_message(tick):
        return SimpleNamespace(header=SimpleNamespace(
            stamp=SimpleNamespace(sec=tick // 120, nanosec=(tick % 120) * 1_000_000_000 // 120),
        ))

    def test_delayed_first_odom_matches_truth_older_than_200_messages(self):
        backend = self.reference_backend()
        for tick in range(600):
            backend.on_truth(self.stamped_message(tick))
        backend.on_odom(self.stamped_message(10))
        self.assertIsNotNone(backend.initial_reference)
        self.assertEqual(backend.initial_reference["stamp_difference_seconds"], 0)
        backend.on_truth(self.stamped_message(601))
        self.assertEqual(len(backend.ground_truth), 600)

    def test_reference_is_latched_when_matching_truth_arrives_after_odom(self):
        backend = self.reference_backend()
        backend.on_odom(self.stamped_message(120))
        self.assertIsNone(backend.initial_reference)
        backend.on_truth(self.stamped_message(120))
        self.assertIsNotNone(backend.initial_reference)

    def test_truth_is_not_collected_while_idle_or_after_startup_deadline(self):
        backend = self.reference_backend()
        backend.collecting = False
        backend.on_truth(self.stamped_message(120))
        backend.collecting = True
        backend.collection_deadline = 0.0
        backend.on_truth(self.stamped_message(120))
        self.assertFalse(backend.ground_truth)

    def test_readiness_timeout_stops_truth_collection(self):
        backend = self.reference_backend()
        backend.wait = Mock(side_effect=TimeoutError("not ready"))
        with self.assertRaises(TimeoutError):
            backend.wait_ready(Mock())
        self.assertFalse(backend.collecting)


if __name__ == "__main__":
    unittest.main()
