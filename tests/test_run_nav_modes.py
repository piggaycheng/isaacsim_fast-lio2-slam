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
        for option in ("--pcd", "--manual-initial-pose", "--navigate",
                       "--filter-editor", "--filter-state"):
            with self.subTest(option=option):
                args = ((option, "missing.yaml") if option in
                        ("--pcd", "--filter-state") else (option,))
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

    def test_navigation_cannot_compete_with_auto_jog(self):
        result = launch("--mode", "3d", "--navigate", "--auto-jog")
        self.assertEqual(result.returncode, 2)
        self.assertIn("--auto-jog cannot be combined with --navigate", result.stderr)

    def test_missing_filter_values_are_rejected(self):
        for option in ("--filter-state",):
            self.assertEqual(launch("--mode", "3d", option).returncode, 2)
            self.assertEqual(launch("--mode", "3d", option, "").returncode, 2)

    def test_removed_mask_options_are_rejected(self):
        for script in (RUN_NAV, RUN_NAV.with_name("run_3d_localization.sh")):
            for option in ("--keepout-mask", "--speed-mask"):
                result = subprocess.run(
                    [str(script), option, "missing.yaml"],
                    capture_output=True, text=True, timeout=10, check=False,
                )
                self.assertEqual(result.returncode, 2)
                self.assertIn(f"Unknown option: {option}", result.stderr)
            result = subprocess.run(
                [str(script), "--help"], capture_output=True, text=True, timeout=10,
                check=False,
            )
            self.assertNotIn("--keepout-mask", result.stdout)
            self.assertNotIn("--speed-mask", result.stdout)

    def test_direct_3d_filters_require_costmaps(self):
        result = subprocess.run(
            [str(RUN_NAV.with_name("run_3d_localization.sh")),
             "--global-fusion", "--filter-editor"],
            capture_output=True, text=True, timeout=10, check=False,
        )
        self.assertEqual(result.returncode, 2)
        self.assertIn("Costmap filters require", result.stderr)

    def test_state_requires_editor(self):
        result = launch("--mode", "3d", "--filter-state", "zones.json")
        self.assertEqual(result.returncode, 2)
        self.assertIn("--filter-state requires --filter-editor", result.stderr)

    def test_editor_state_help_uses_map_directory(self):
        for script in (RUN_NAV, RUN_NAV.with_name("run_3d_localization.sh")):
            with self.subTest(script=script.name):
                result = subprocess.run(
                    [str(script), "--help"], capture_output=True, text=True, timeout=10,
                    check=False,
                )
                self.assertEqual(result.returncode, 0)
                self.assertIn("maps/costmap_filters/editor.json", result.stdout)
                self.assertNotIn("ros2_ws/log/costmap_filters/editor.json", result.stdout)


if __name__ == "__main__":
    unittest.main()
