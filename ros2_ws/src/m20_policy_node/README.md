# m20_policy_node

External ROS 2 inference node for the M20 policy trained in `locomotion_training`
(`MyLocomotionTraining-Velocity-Flat-M20` or `...-Rough-M20`). It loads the TorchScript `policy.pt` exported by `isaaclab play`
(`logs/rsl_rl/m20_{flat,rough}/<run>/exported/policy.pt`) and is independent of Isaac Sim, so the same node can drive a real robot.

| Direction | Topic | Type | Content |
|-----------|-------|------|---------|
| in | `/m20/joint_states` | `sensor_msgs/JointState` | position/velocity of the 16 joints by name; the latest message is used by the 50 Hz inference timer (`control_rate_hz:=0`: each message triggers one inference) |
| in | `/m20/imu` | `sensor_msgs/Imu` | orientation (wxyz base→world) and angular velocity in the base frame |
| in | `/m20/base_velocity` | `geometry_msgs/TwistStamped` | linear velocity in the base frame |
| in | `/cmd_vel` | `geometry_msgs/Twist` | vx, vy, yaw rate (treated as zero after `command_timeout_s`) |
| out | `/m20/joint_commands` | `sensor_msgs/JointState` | leg `position` targets (rad) and wheel `velocity` targets (rad/s), NaN where unused; stamp copied from the triggering joint state |

The launch file defaults to `use_sim_time:=true` (follows `/clock`, which `m20_standalone.py --external` publishes); pass `use_sim_time:=false` on the real robot.
**Rough policy (243-dim obs):** the 17x11 height scan (1.6 x 1.0 m, 0.1 m, yaw-aligned, `base_z - ground_z - 0.5` clipped to +-1) is
sampled from the elevation_mapping_cupy GridMap (`elevation_map_topic`, layer `height_layer`, map -> `base_frame` via TF).
`drift_free_height` (default true) takes the base height as the standing height above the median scan terrain, so LiDAR-odometry z drift does not corrupt the scan. `use_height_scan:=false` gives the 56-dim flat obs. Until the map and TF are available the node publishes the default pose.

Controls: the `m20_jog` panel (see below); in the sim itself only `--command VX VY WZ` or `--demo` (scripted loop), otherwise a zero command. RViz starts with the script (`--no-rviz` to skip). The IMU/TF graph starts after 3 s of sim time so FAST-LIO does not initialise on the spawn jolt (it diverged otherwise). The LiDAR sits on the body at z=0.15 (`--lidar-offset`), its mesh is hidden and returns closer than `--lidar-near-range` (default 1.0 m, `nearRangeM`) are dropped so the robot body does not show up in the cloud.

Scene options of `scripts/m20_standalone.py`: `--environment office|grid` (default office); `--office-ground mesh|plane` (default `mesh`: floors and stairs get triangle-mesh colliders so you can descend below z=0; `plane` keeps a collision plane at z=0 plus stair colliders). Debug: `--flat-scan` (local mode) and `FLAT_HEIGHT_SCAN=true scripts/run_m20_policy.sh` (node param `flat_height_scan`) fill the height scan with flat ground to isolate policy vs. map problems.
The launch file also starts FAST-LIO2 + elevation mapping (`height_map:=false` to skip). The sim side needs the LiDAR/IMU
mounted by `m20_standalone.py --external` (`--no-lidar` to disable). LiDAR odometry degrades on the featureless default
grid ground; use a scene with structure for long runs.

Inference is timer-driven (`control_rate_hz`, default 50 Hz, node clock) on the latest joint states / IMU / base velocity; if any is older than `state_timeout_s` (0.1 s) that tick is skipped. `m20_standalone.py --external` publishes the state at every physics step (200 Hz) like real sensor streams, applies the newest command once per control period (50 Hz), waiting up to `--external-timeout` for one if none arrived. `control_rate_hz:=0` (inference per `joint_states` message) would run the policy at 200 Hz there, so keep the timer.

QoS: `joint_states`, `imu`, `base_velocity` (subscribers) and `joint_commands` (publisher) use depth 1 and BEST_EFFORT by default (`best_effort_qos`), compatible with sensor-data publishers; `m20_standalone.py --external` uses the same QoS. Set `best_effort_qos:=false` for reliable endpoints (a reliable subscriber cannot receive from a best-effort publisher, so both sides must agree). `/cmd_vel` and the elevation map keep the default reliable QoS.

```bash
docker compose run --rm ros build
ros2 launch m20_policy_node m20_policy.launch.py policy_path:=/workspace/path/to/policy.pt  # use_sim_time:=false for hardware
# Isaac Sim side (host), using the ROS 2 DDS settings of run_height_map.sh:
"${ISAACSIM_DIR:-$HOME/isaacsim-6.2.0}/python.sh" scripts/m20_standalone.py --external
# or everything at once (rebuild with --build after node changes):
scripts/run_m20_policy.sh --policy /path/to/exported/policy.pt
```

## Jog panel (`m20_jog`)

The sim has no keyboard control (it lagged). `ros2_ws/src/m20_jog` is a PyQt5 panel running outside the sim that publishes `/cmd_vel` at 10 Hz (hold buttons or W/S/A/D/Q/E/arrow keys, Space = stop; releasing or losing focus publishes zero). Start the sim with `--no-cmd-vel` so only the panel publishes the command:

```bash
"${ISAACSIM_DIR:-$HOME/isaacsim-6.2.0}/python.sh" scripts/m20_standalone.py --external --no-cmd-vel
ros2 run m20_jog m20_jog   # needs a display; rebuild the docker image once for python3-pyqt5
```

`scripts/run_m20_policy.sh` starts the jog panel automatically (and passes `--no-cmd-vel` to the sim); use `--no-jog` to skip it (the sim then publishes a zero `/cmd_vel`).

Panel only (e.g. sim started separately with `--no-cmd-vel`): `scripts/run_m20_jog.sh [--build]`.
