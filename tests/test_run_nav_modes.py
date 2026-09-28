import subprocess
import unittest
from pathlib import Path


RUN_NAV = Path(__file__).resolve().parents[1] / "run_nav.sh"
RUN_2D = RUN_NAV.with_name("run_2d_localization.sh")


def launch(*args):
    return subprocess.run(
        [str(RUN_NAV), *args], capture_output=True, text=True, timeout=10,
        check=False,
    )


class TestRunNavModes(unittest.TestCase):
    def test_mode_is_required(self):
        result = launch()
        self.assertEqual(result.returncode, 2)
        self.assertIn("Specify --mode 2d or --mode 3d", result.stderr)

    def test_invalid_or_repeated_mode_is_rejected(self):
        for args in (("--mode", "other"), ("--mode",), ("--mode", "2d", "--mode", "3d")):
            with self.subTest(args=args):
                self.assertEqual(launch(*args).returncode, 2)

    def test_2d_rejects_3d_only_options(self):
        for option in ("--pcd", "--manual-initial-pose"):
            with self.subTest(option=option):
                args = (option, "map.pcd") if option == "--pcd" else (option,)
                result = launch("--mode", "2d", *args)
                self.assertEqual(result.returncode, 2)
                self.assertIn("require --mode 3d", result.stderr)

    def test_2d_keeps_its_map_validation(self):
        result = launch("--mode", "2d", "--map", "missing.yaml")
        self.assertEqual(result.returncode, 1)
        self.assertIn("Map YAML does not exist", result.stderr)

    def test_2d_script_can_be_run_directly(self):
        result = subprocess.run(
            [str(RUN_2D), "--map", "missing.yaml"],
            capture_output=True, text=True, timeout=10, check=False,
        )
        self.assertEqual(result.returncode, 1)
        self.assertIn("Map YAML does not exist", result.stderr)

    def test_3d_delegates_to_3d_map_validation(self):
        result = launch("--mode", "3d", "--map", "missing.yaml")
        self.assertEqual(result.returncode, 1)
        self.assertIn("PGM map YAML does not exist", result.stderr)

    def test_3d_forwards_pcd_map(self):
        result = launch("--mode", "3d", "--pcd", "missing.pcd")
        self.assertEqual(result.returncode, 1)
        self.assertIn("PCD map does not exist", result.stderr)


if __name__ == "__main__":
    unittest.main()
