"""Launch substitutions applying robot_fleet namespacing and robot-type overrides."""

import os
import sys
import tempfile

import yaml
from launch.substitution import Substitution
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from robot_fleet import (  # noqa: E402
    load_robot_profile, merge_overrides, namespace_config, namespace_rviz,
    namespaced_topic, normalize_namespace, with_local_ekf_inputs,
)

_directory = None


def _write(prefix, config):
    global _directory
    if _directory is None:
        _directory = tempfile.TemporaryDirectory(prefix="isaac_robot_namespace_")
    descriptor, filename = tempfile.mkstemp(prefix=prefix, suffix=".yaml", dir=_directory.name)
    with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
        yaml.safe_dump(config, stream, sort_keys=False)
    return filename


def robot_overrides(robot_type):
    return load_robot_profile(robot_type)["parameter_overrides"]


def local_odometry_file(path, templates_path, namespace, robot_type, inputs):
    """local_odometry.yaml for this robot with local_ekf fed by the selected inputs."""
    with open(templates_path, encoding="utf-8") as stream:
        templates = yaml.safe_load(stream)
    return robot_parameter_file(
        path, namespace, robot_type,
        transform=lambda config: with_local_ekf_inputs(config, templates, inputs),
    )


def load_parameters(path, robot_type):
    """Read a parameter file with the robot type's overrides applied."""
    with open(path, encoding="utf-8") as stream:
        return merge_overrides(yaml.safe_load(stream), robot_overrides(robot_type))


def robot_parameter_file(path, namespace, robot_type, transform=None):
    """Return a parameter file for this robot; the original path if nothing changes.

    transform(config) may rewrite the merged config before namespacing.
    """
    namespace = normalize_namespace(namespace)
    with open(path, encoding="utf-8") as stream:
        original = yaml.safe_load(stream)
    config = merge_overrides(original, robot_overrides(robot_type))
    if transform is not None:
        config = transform(config)
    if not namespace and config == original:
        return path
    return _write(f"{namespace or 'root'}_", namespace_config(config, namespace))


def namespaced_rviz_file(path, namespace):
    namespace = normalize_namespace(namespace)
    if not namespace:
        return path
    with open(path, encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    return _write(f"{namespace}_rviz_", namespace_rviz(config, namespace))


class _RobotSubstitution(Substitution):
    def __init__(self, *values):
        super().__init__()
        self._values = [normalize_to_list_of_substitutions(value) for value in values]

    def _perform(self, context):
        return [perform_substitutions(context, value) for value in self._values]


class NamespacedTopic(_RobotSubstitution):
    """NamespacedTopic(name, namespace)."""

    def perform(self, context):
        name, namespace = self._perform(context)
        return namespaced_topic(namespace, name)


class RobotParameterFile(_RobotSubstitution):
    """RobotParameterFile(path, namespace, robot_type)."""

    def perform(self, context):
        return robot_parameter_file(*self._perform(context))


class NamespacedRvizFile(_RobotSubstitution):
    """NamespacedRvizFile(path, namespace)."""

    def perform(self, context):
        return namespaced_rviz_file(*self._perform(context))
