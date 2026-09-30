# FASTLIO2 ROS 2 bridge

This workspace includes FASTLIO2_ROS2, `livox_ros_driver2`, Livox-SDK2, and
Sophus 1.22.10 as Git submodules. `standalone.py` publishes:

- `/isaac/lidar_points` (`sensor_msgs/msg/PointCloud2`)
- `/isaac/imu` (`sensor_msgs/msg/Imu`)
- `/clock` (`rosgraph_msgs/msg/Clock`)

FASTLIO2_ROS2 expects `/livox/lidar` as
`livox_ros_driver2/msg/CustomMsg`. The `isaac_fastlio_adapter` node converts
the Isaac Sim point cloud and supplies synthetic `line`, `tag`, `offset_time`,
and reflectivity values. Isaac Sim's Nova Carter RTX point cloud contains XYZ
only, so these Livox-specific fields are approximations for simulation.

The upstream FASTLIO2 fork multiplies incoming standard IMU acceleration by
`10`. The included `imu_scale_adapter` compensates for that behavior and
publishes the corrected input on `/livox/imu`.

`src/FAST_LIO_LOCALIZATION2` is a pinned upstream submodule;
`src/isaac_localization_3d` contains the separate 3D localization launch,
ROS pose/TF publisher, and RViz configuration. `run_nav.sh --mode 3d` starts
this package; `--mode 2d` uses `isaac_localization_2d` and shared `isaac_nav` inputs. The Docker
workspace build below also builds the 3D packages. Launch the Office
simulator and RViz together with `./run_3d_localization.sh` (use `--help` for
map, headless, and initialization options). The Office spawn near `(0, 0)` is
sent as an approximate initial pose by default; use RViz's **2D Pose Estimate**
to reinitialize at the robot's current location, or `--manual-initial-pose`
to require a manual estimate. RViz displays the PGM map, registered scan,
map-frame Carter arrow, and path. The 3D launch uses a thin wrapper around
the pinned upstream localization node: its `/submap` visualization cloud
contains only its actual XYZ fields (and intensity if present), without
synthesizing RGB/intensity or changing the original `/isaac/lidar_points`
FAST-LIO input. The redundant `/cur_scan_in_map` copy is not published;
ICP still consumes `/cloud_registered` directly. This mode owns
`map -> camera_init` only;
do not run it alongside `run_nav.sh`, whose AMCL owns `map -> odom`.

## Docker setup

The ROS 2 Humble nodes (FASTLIO2, PGO, localization, Nav2 and RViz) run in the
`ros` Docker Compose service; Isaac Sim (`standalone.py`) runs on the host.
Requirements: Docker with Compose v2, the NVIDIA Container Toolkit, an X11
display, and Isaac Sim installed at the path in `standalone.py`'s shebang. The host
does not need ROS 2.

`docker/Dockerfile` installs every apt/pip dependency (GTSAM, Nav2,
`robot_localization`, `pointcloud_to_laserscan`, `pcl_ros`, Open3D, ...)
and builds Livox-SDK2 and Sophus from their submodules. The project directory
is mounted at `/workspace`, so `ros2_ws/src` edits need no image rebuild; only
dependency changes do. Initialize submodules, then build the image and the
workspace:

```bash
git submodule update --init --recursive
docker compose build
docker compose run --rm ros build
```

Rerun `docker compose run --rm ros build` after changing `ros2_ws/src`; extra
arguments are passed to `colcon build`, for example
`docker compose run --rm ros build --packages-select isaac_nav`. The image
user matches UID/GID 1000 by default; if your IDs differ, build with
`HOST_UID=$(id -u) HOST_GID=$(id -g) docker compose build`.

Each `run_*.sh` script validates its options, starts the `ros` service with
`docker compose up`, follows its logs, then launches `standalone.py` on the
host with matching options. The container is stopped when Isaac Sim exits or
on Ctrl+C. The container uses host networking and IPC, so DDS (including
shared-memory transport) reaches Isaac Sim's ROS 2 bridge directly;
`ROS_DOMAIN_ID`, `ROS_LOCALHOST_ONLY` and `RMW_IMPLEMENTATION` are passed through
from your shell. Map, PCD and output paths must be inside the project
directory, because only it is mounted in the container.

Run other ROS commands in the container with, for example:

```bash
docker compose run --rm ros ros2 topic list     # standalone container
docker compose exec ros /workspace/docker/entrypoint.sh ros2 topic list  # while a run script is active
```

Run the GUI simulation, adapters, FASTLIO2, its existing PGO backend, and RViz
together:

```bash
./run_slam.sh
```

RViz opens with `lidar` as its fixed frame and displays
`/fastlio2/world_cloud`, the FASTLIO trajectory, and PGO loop-closure markers.
The PGO node selects keyframes from `/fastlio2/body_cloud` and
`/fastlio2/lio_odom`, searches prior poses with a KD-tree, verifies candidates
against a local submap with ICP, and optimizes the pose graph with GTSAM iSAM2. The PGO node still publishes the
`map` to `lidar` correction transform, but RViz keeps `lidar` as its fixed
frame so delayed PGO transforms cannot block the live point cloud.

`docker/build_workspace.sh` applies `patches/fastlio2-pgo-sync.patch` while compiling
the upstream PGO node. The patch uses exact cloud/odometry timestamp matching,
recovers from simulation clock resets, atomically consumes the newest queued
measurement, and logs accepted keyframes and loop closures. The external
FASTLIO2 submodule is restored after the build so it remains clean.

Save the optimized PCD map and optional keyframe patches after mapping:

```bash
./save_map.sh
```

Run `./save_map.sh --help` to list the named options. The output directory
defaults to `maps/office`, keyframe patch saving defaults to `true`, and the
voxel size defaults to the PGO configuration. For example, save with 5 cm
voxels using:

```bash
./save_map.sh --voxel-size 0.05
```

To select every option explicitly:

```bash
./save_map.sh \
  --output-dir maps/office \
  --save-patches false \
  --voxel-size 0.1
```

This writes `map.pcd`, `poses.txt`, and (when requested) a `patches/`
directory. Before writing `map.pcd`, PGO applies the voxel size configured by
`save_map_resolution` in `src/FASTLIO2_ROS2/pgo/config/pgo.yaml` (default:
`0.1` m). The `--voxel-size` option overrides it for that running PGO node;
set it to `0` to disable final-map downsampling. Individual files in
`patches/` retain their original keyframe resolution. The PGO implementation
uses pose-proximity loop candidates, so the current FASTLIO trajectory must
remain within the configured search radius of the earlier visit before ICP can
verify a closure.

Convert the optimized 3D PCD into a Nav2-compatible 2D occupancy map:

```bash
./pcd2pgm.py
```

The defaults read `maps/office/map.pcd` and write
`maps/office/map_2d.pgm` plus `maps/office/map_2d.yaml`. The converter projects
points between 0.1 m and 2.0 m at 0.05 m resolution. For example:

```bash
./pcd2pgm.py maps/office/map.pcd maps/office/map_2d \
  --z-min 0.1 --z-max 2.0 --resolution 0.05
```

Use `--min-points` to reject sparsely populated cells, `--inflation` to widen
obstacles, or `--background unknown` to leave unoccupied cells unknown.
Obstacle inflation is normally better handled by the Nav2 costmap. Because a
merged PCD contains obstacle returns but not the original LiDAR rays, the
converter cannot reconstruct observed free and unknown space exactly; its
default free background matches common PCD-to-PGM tools.

The IMU is colocated with the RTX LiDAR in `standalone.py`, so the supplied
`isaac_lio.yaml` uses identity LiDAR-to-IMU extrinsics. Drive Carter with
W/S/A/D or the arrow keys; press Space to stop. The main Isaac Sim viewport
automatically follows Carter from behind while jogging.

Motion BVH is enabled for RTX sensor motion tracking. The LiDAR publisher
includes native per-point timestamps, intensity, emitter IDs, and channel IDs.
The raw RTX output is explicitly set to `NONCOMPENSATED`, preserving motion
distortion for FASTLIO to remove using the simulated IMU.
The adapter uses the native timestamps for FASTLIO deskew instead of
synthetically distributing points over a scan. XT-32 channel IDs are folded
into FASTLIO's four accepted Livox line IDs, while preserving the original
point order and timing.
# 2D Localization

`isaac_nav` provides wheel encoder odometry, the navigation IMU adapter,
ground obstacle filtering, the Nav2 goal-heading controller and goal checker
plugins, and the shared Local EKF/scan settings in
`config/local_odometry.yaml`. `isaac_localization_2d` owns the AMCL settings,
2D map/AMCL launch and RViz configuration. The 3D fusion launch reuses the
shared inputs without starting AMCL. `isaac_fastlio_adapter` remains dedicated
to FAST-LIO mapping inputs.

The navigation dependencies are installed in the Docker image (see
[Docker setup](#docker-setup)).

While the apt `ros-humble-tf2` is older than 0.25.24, the Dockerfile builds
upstream tf2 0.25.24 and replaces only the image's `libtf2.so`. Older tf2 has a
lock-order deadlock (ros2/geometry2#990) between the TF listener and costmap
message filters that freezes a Nav2 server's TF buffer after minutes, producing
`Transform data too old` and a rotated local costmap. Rebuilding the image
skips the replacement once the apt tf2 contains the fix.

Start Isaac Sim and the 2D localization stack with the default Office map:

```bash
./run_nav.sh --mode 2d
```

`run_nav.sh` requires `--mode` and delegates 2D startup to
`run_2d_localization.sh`; the latter can also be run directly with the same
2D options, without `--mode`.

The default map is `maps/office/map_2d.yaml`; that YAML loads
`maps/office/map_2d.pgm`. Select another Nav2 map YAML with:

```bash
./run_nav.sh --mode 2d --map maps/warehouse/map.yaml
```

For a headless automatic localization test:

```bash
./run_nav.sh --mode 2d --headless --auto-jog --no-rviz
```

The localization stack:

- publishes Carter wheel joint angles from Isaac Sim;
- generates `/wheel/odom` from simulated encoder ticks, calibration bias, and noise;
- converts `/isaac/lidar_points` into `/scan` using a height-filtered
  `pointcloud_to_laserscan`;
- runs a Local EKF for `odom -> base_link`;
- runs AMCL against the selected PGM map as the only publisher of `map -> odom`.

The shared Local EKF fuses the wheel encoder's yaw pose and forward speed with the
navigation IMU's angular rate. Wheel yaw anchors the heading when the robot
stops, so a small nonzero simulated gyro rate cannot accumulate indefinitely;
the IMU still contributes during turns. The same Local EKF configuration is
used by 2D and 3D fusion. Wheel yaw is dead-reckoned and can drift if the
wheels slip. Wheel covariance grows with the distance and angle actually
driven; calibrate it for a real robot with `covariance_calibration.py` (see
`docs/covariance_calibration.md`) rather than assuming the simulated values apply.

The 2D-only stack does not start a Global EKF. In a move-then-stop test, the
previous Global EKF continued shifting `map -> odom` while `/amcl_pose` and the
local odometry remained nearly stationary. Letting AMCL own this transform
prevents that drift; its `map -> odom` correction remains fixed between AMCL
updates. To test PCD-based **global fusion without Nav2 costmaps**, build the
workspace (see [Docker setup](#docker-setup)), then use:

```bash
./run_3d_localization.sh --global-fusion
```

This isolated mode uses 3D map registration as a gated global pose observation
and wheel/IMU inputs in both Local and Global EKFs. Local EKF owns
`odom -> base_link`; the Global EKF publishes `/odometry/global` with TF disabled,
and a freshness gate alone publishes `map -> odom` while PCD corrections remain
recent. It does **not** start AMCL or Nav2 costmaps by default; the PGM is displayed in RViz.
Nova Carter's USD drive-wheel contact radius is 0.14 m (0.4132 m wheelbase);
both the simulator controller and wheel odometry use these dimensions. Using
the smaller controller-only radius for wheel odometry makes the moving scan
drift relative to the map between PCD corrections.
The same height-filtered `pointcloud_to_laserscan` configuration as the 2D
mode also projects `/isaac/lidar_points` into `/scan` (in `base_link`), shown
against the PGM in RViz. The height window is 0.1–2.0 m above `base_link`:
this is a 2D obstacle projection, **not** ground segmentation or a 3D costmap.
Obstacles below 0.1 m or beyond the sensor/range limits may be missed; Nav2
does not consume `/scan` until its costmap is configured and launched.
For experimental 3D perception, run
`./run_3d_localization.sh --global-fusion --obstacle-cloud`.
The optional ground filter transforms the raw LiDAR cloud into `base_link`,
fits a near-horizontal ground plane with RANSAC, and publishes height-limited,
8 cm voxelized `/perception/obstacles` as a `PointCloud2`. It warns and
withholds a scan if the ground or timestamped TF is unavailable. The RViz 3D
obstacle display is off by default to avoid additional rendering load. This Office-floor prototype does not track moving objects, compensate each point's
motion, or handle ramps; it is not safe obstacle avoidance.
To observe Nav2 costmaps without starting a planner or controller:

```bash
./run_3d_localization.sh --global-fusion --costmaps
```

The unified entry point is `./run_nav.sh --mode 3d`. It launches this same
PCD fusion and costmap observation mode, not autonomous driving. Both modes
require `--mode` explicitly: `2d` starts AMCL, while `3d` starts PCD localization
and costmaps without AMCL. `--map` selects a PGM map in either mode; `--pcd`
and `--manual-initial-pose` apply only to `3d`. The separate
`run_3d_localization.sh` remains available directly for its visualization-only and
fusion-without-costmaps variants. Never run the modes concurrently.

To navigate with Nav2 in the Office simulation, start:

```bash
./run_nav.sh --mode 3d --navigate
```

After the planner, controller and navigator report active, click RViz's
**2D Goal Pose** tool on the map and drag to set the target heading. RViz
publishes `/goal_pose` directly to Nav2's `NavigateToPose` navigator; there
is no short-distance or forward-only goal test. In observation-only mode,
this RViz tool does not drive Carter.

Nav2 plans on the global costmap (PGM static layer plus 3D obstacle marking) and
follows paths using the local obstacle costmap. Obstacles missing from the PGM,
such as desks seen above the 2D slice or objects moved after mapping, are marked
in both costmaps, so the 1 Hz replanning routes around them. Its `/nav2/cmd_vel` passes through a ROS 2 safety node (PCD correction
freshness, planar command validation and speed limiting) to `/cmd_vel`, which
Isaac Sim's native ROS 2 Subscribe Twist node receives to drive Carter. There
is no Unix socket or separate command transport.
The controller is `isaac_nav::GoalHeadingLatchedRPP`, a thin wrapper around
Humble's Regulated Pure Pursuit, paired with `isaac_nav::LatchedGoalChecker`.
Humble RPP re-checks `xy_goal_tolerance` every cycle and stops right at that
boundary, and the controller server resets goal checkers on every 1 Hz replan.
Small drift while rotating in place used to flip Carter between path tracking
and the final heading until the progress checker aborted. Both plugins now keep
the reached-position state for the same goal until it succeeds, a new goal
arrives, or Carter drifts beyond `latch_release_distance` (0.5 m).
RViz shows Nav2's `/plan` path alongside both costmap layers.
The local costmap is intentionally in `odom`, not `map`, so its entire grid
can appear rotated in RViz's `map` fixed frame by the current `map -> odom`
correction. This is not a footprint rotation; inspect that TF and compare
wheel and Local EKF heading before changing the costmap frame.
The safety node rejects non-finite or nonplanar commands and limits speed to
0.75 m/s and 0.7 rad/s. Isaac Sim also checks command bounds and stops on a
0.5 s command timeout if ROS messages stop; stale PCD corrections suppress
movement. Nav2's regulated pure pursuit controller targets 0.5 m/s
(`desired_linear_vel` in `config/navigation.yaml`), subject to its approach,
curvature, and collision speed reductions. It uses a fixed 0.8 m lookahead
to reduce side-to-side corrections on straight paths; check corner tracking
and clearance before using longer or tighter routes. Manual W/S keyboard
jogging commands
0.75 m/s in either direction; auto-jog remains at 0.2 m/s. Higher navigation
speeds require controller, footprint, and stopping-distance validation.
`--navigate` cannot be combined with
`--auto-jog`. Unlike observation-only costmaps, Nav2's planner and controller
own both costmaps. This configuration replans periodically and recovers from
planner or controller failures (`config/navigate_to_pose.xml`): a failed
planner or controller clears its costmap and retries once; if navigation still
fails, `behavior_server` runs one recovery per failure in rotation (clear both
costmaps, wait 5 s, back up 0.3 m, wait 10 s) and retries, up to 6 times
(about 30 s) before aborting. Back-up checks the local costmap footprint and
is skipped when blocked, and its `cmd_vel` goes through the same safety node.
Carter's rectangular footprint and inflation are estimates,
and dynamic obstacle clearing and localization failure response are not yet
validated for safe autonomous operation. Test only in a clear Office
simulation and inspect the costmaps and planned path before longer drives.

`--costmaps` also enables `--obstacle-cloud`. RViz overlays the Office PGM
with `/global_costmap/costmap` (static PGM, 3D obstacle marking and inflation) and
`/local_costmap/costmap` (rolling 8 m window, 3D obstacle marking, `/scan`
ray clearing, and inflation). Toggle the Map displays in RViz to compare
the occupied, inflated, and free areas; enable "3D ground-filtered obstacles
(optional)" to compare the local obstacle inputs. Both costmaps wait for
the map and a recent localization TF before activation. With the default
Office map, the 3D pose adapter automatically sends an initial pose near
Carter's spawn; no RViz click is needed. `--manual-initial-pose` is for
other maps or a different starting location. Both costmaps use the same
`base_link`-relative rectangular footprint (front 0.65 m, rear 0.20 m, left
and right 0.32 m). It approximates the Nova Carter USD body and wheels with
about 6 cm of clearance; replace it when changing robots. The 0.9 m inflation
radius remains an estimate, and neither setting is a validated safety clearance.
Low obstacles absent from the height-filtered
`/scan` might not clear reliably after moving; verify marking and clearing
in your scene before using these layers for navigation. Without `--navigate`,
this mode does not publish driving commands or start autonomous navigation.
Do not run it alongside `run_nav.sh` or the original 3D demo. Custom PCD/PGM
maps require `--manual-initial-pose` and an approximate position from RViz.
The ICP node publishes its registration covariance; the adapter adds the
calibrated `min_covariance_xy/yaw` floors and stamps the pose with the FAST-LIO
time it was composed from. `covariance_calibration.py` estimates these values
from a rosbag without ground truth (`docs/covariance_calibration.md`) and was
validated against Isaac Sim ground truth. Real-vehicle navigation safety is not
yet validated.
The RViz "Accepted PCD Position" display shows the accepted pose without its
covariance geometry: unobserved height/tilt axes carry deliberately large
variances and otherwise draw misleading vertical lines. Covariance remains in
the published message for the EKF.

Use `./run_nav.sh --help` for all options.
