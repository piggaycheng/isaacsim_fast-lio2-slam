# FASTLIO2 ROS 2 bridge

This workspace includes FASTLIO2_ROS2, `livox_ros_driver2`, Livox-SDK2, and
Sophus 1.22.10 as Git submodules. `scripts/standalone.py` publishes:

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
`src/slam_localization_3d` contains the separate 3D localization launch,
ROS pose/TF publisher, and RViz configuration. `scripts/run_nav.sh --mode 3d` starts
this package; `--mode 2d` uses `slam_localization_2d` and shared `slam_nav` inputs. The Docker
workspace build below also builds the 3D packages. Launch the Office
simulator and RViz together with `./scripts/run_3d_localization.sh` (use `--help` for
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
do not run it alongside `scripts/run_nav.sh`, whose AMCL owns `map -> odom`.

## Docker setup

The ROS 2 Humble nodes (FASTLIO2, PGO, localization, Nav2 and RViz) run in the
`ros` Docker Compose service; Isaac Sim (`scripts/standalone.py`) runs on the host.
Requirements: Docker with Compose v2, the NVIDIA Container Toolkit, an X11
display, and Isaac Sim installed at the path in `scripts/standalone.py`'s shebang. The host
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
`docker compose run --rm ros build --packages-select slam_nav`. The image
user matches UID/GID 1000 by default; if your IDs differ, build with
`HOST_UID=$(id -u) HOST_GID=$(id -g) docker compose build`.

Each `run_*.sh` script validates its options, starts the `ros` service with
`docker compose up`, follows its logs, then launches `scripts/standalone.py` on the
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
./scripts/run_slam.sh
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
./scripts/save_map.sh
```

Run `./scripts/save_map.sh --help` to list the named options. The output directory
defaults to `maps/office`, keyframe patch saving defaults to `true`, and the
voxel size defaults to the PGO configuration. For example, save with 5 cm
voxels using:

```bash
./scripts/save_map.sh --voxel-size 0.05
```

To select every option explicitly:

```bash
./scripts/save_map.sh \
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
./scripts/pcd2pgm.py
```

The defaults read `maps/office/map.pcd` and write
`maps/office/map_2d.pgm` plus `maps/office/map_2d.yaml`. The converter projects
points between 0.1 m and 2.0 m at 0.05 m resolution. For example:

```bash
./scripts/pcd2pgm.py maps/office/map.pcd maps/office/map_2d \
  --z-min 0.1 --z-max 2.0 --resolution 0.05
```

Use `--min-points` to reject sparsely populated cells, `--inflation` to widen
obstacles, or `--background unknown` to leave unoccupied cells unknown.
Obstacle inflation is normally better handled by the Nav2 costmap. Because a
merged PCD contains obstacle returns but not the original LiDAR rays, the
converter cannot reconstruct observed free and unknown space exactly; its
default free background matches common PCD-to-PGM tools.

The IMU is colocated with the RTX LiDAR in `scripts/standalone.py`, so the supplied
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

`slam_nav` provides wheel encoder odometry, the navigation IMU adapter,
ground obstacle filtering, the Nav2 goal-heading controller and goal checker
plugins, and the shared Local EKF/scan settings in
`config/local_odometry.yaml`. Vehicle-specific wheel and covariance values are
not in that file; launches merge them from the robot profile in
`slam_localization_3d/config/robots/`. `slam_localization_2d` owns the AMCL settings,
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
./scripts/run_nav.sh --mode 2d
```

`scripts/run_nav.sh` requires `--mode` and delegates 2D startup to
`scripts/run_2d_localization.sh`; the latter can also be run directly with the same
2D options, without `--mode`.

The default map is `maps/office/map_2d.yaml`; that YAML loads
`maps/office/map_2d.pgm`. Select another Nav2 map YAML with:

```bash
./scripts/run_nav.sh --mode 2d --map maps/warehouse/map.yaml
```

For a headless automatic localization test:

```bash
./scripts/run_nav.sh --mode 2d --headless --auto-jog --no-rviz
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
./scripts/run_3d_localization.sh --global-fusion
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
`./scripts/run_3d_localization.sh --global-fusion --obstacle-cloud`.
The optional ground filter transforms the raw LiDAR cloud into `base_link`,
fits a near-horizontal ground plane with RANSAC, and publishes height-limited,
8 cm voxelized `/perception/obstacles` as a `PointCloud2`. It warns and
withholds a scan if the ground or timestamped TF is unavailable. The RViz 3D
obstacle display is off by default to avoid additional rendering load. This Office-floor prototype does not track moving objects, compensate each point's
motion, or handle ramps; it is not safe obstacle avoidance.
To observe Nav2 costmaps without starting a planner or controller:

```bash
./scripts/run_3d_localization.sh --global-fusion --costmaps
```

The unified entry point is `./scripts/run_nav.sh --mode 3d`. It launches this same
PCD fusion and costmap observation mode, not autonomous driving. Both modes
require `--mode` explicitly: `2d` starts AMCL, while `3d` starts PCD localization
and costmaps without AMCL. `--map` selects a PGM map in either mode; `--pcd`
and `--manual-initial-pose` apply only to `3d`. The separate
`scripts/run_3d_localization.sh` remains available directly for its visualization-only and
fusion-without-costmaps variants. Never run the modes concurrently.

To navigate with Nav2 in the Office simulation, start:

```bash
./scripts/run_nav.sh --mode 3d --navigate
```

After the planner, controller and navigator report active, click RViz's
**2D Goal Pose** tool on the map and drag to set the target heading. RViz
publishes `/goal_pose` directly to Nav2's `NavigateToPose` navigator; there
is no short-distance or forward-only goal test. In observation-only mode,
this RViz tool does not drive Carter. The navigation architecture (costmaps,
planner/controller, recovery and the `cmd_vel` safety chain) is described in
`docs/nav.md`; the localization data flow is in `docs/3d_localization.md`.

For live RViz annotation, run `./scripts/run_nav.sh --mode 3d --navigate --filter-editor`.
Use **Publish Point** to draw polygon vertices, then **Interact** and right-click
the draft's center cube to apply Keepout or a percentage Speed zone. Existing
zones have menus for vertex editing and deletion. Updates are published live and
atomically saved to `maps/costmap_filters/editor.json` alongside map data, or the project-local
path given by `--filter-state FILE`. The editor restores matching map state on
restart and refuses mismatched maps or invalid polygons.
Keepout applies to both costmaps with additional inflation after the filter;
Speed applies to the local costmap and publishes percentage limits on
`/speed_limit` for RPP path-following linear velocity (not recovery, rotation or
direct velocity commands). Speed mask value 0 means unrestricted, not stop.
Without editor mode, no filter editor or plugins are added.
The editor publishes both masks and filter info, and costmap readiness
waits for both masks before activation. See `docs/nav.md` for the workflow
and limitations.

With `--navigate`, `direction_zones.py` switches collision-monitor zone sets by
command direction, like safety-scanner field sets (Humble has no
`VelocityPolygon`). Forward enables Stop, Slow and the Surround trimmed to
0.15 m behind the padded footprint; reverse enables the Surround trimmed to
0.15 m ahead of it; in-place rotation enables the full Surround. Both trimmed
zones are derived at launch from each robot profile (`config/direction_zones.yaml`).
Zones switch atomically through native `.enabled` parameters only after a zero
command and fresh odometry confirming standstill, so BackUp is no longer
blocked by an obstacle in the front Stop zone. FootprintApproach stays enabled.
`--static-zones` (`direction_zones:=false`) restores all zones in every direction;
`--adaptive-surround` takes precedence over direction zones.

For experimental speed-adaptive Surround on Humble, run
`./scripts/run_nav.sh --mode 3d --navigate --adaptive-surround` (or
`scripts/run_3d_localization.sh --global-fusion --navigate --adaptive-surround`).
It replaces the direction zones. The opt-in selector atomically
switches native polygon enable flags, not unsupported runtime point updates:
full x [-1.35, 0.80], y +/-0.75 m; crawl x [-0.90, 0.45], y +/-0.55 m.
Crawl commands are capped at 0.10 m/s and 0.20 rad/s by both the selector and
the final safety gate. RPP requests 0.10 m/s and 0.15 rad/s in this mode;
the smoother also caps curved-path angular commands at +/-0.20 rad/s.
Shrinking requires both bounded commands and fresh local EKF twist/pose-difference
motion within 0.12 m/s and 0.22 rad/s for 0.5 s. Switching blocks motion until
the final gate acknowledges zero limits and the native monitor acknowledges
the atomic update. Missing odometry/commands, invalid geometry or failed
switches do not allow motion. The final gate requires a stamped limits heartbeat;
its 0.2 s expiry is checked every 0.1 s. Selector exit leaves the safety gates
running to enforce the stop; restart is required. Near-future DDS messages are
buffered until the receiver's ROS clock catches up, not used as fresh evidence
before their timestamp. The active acknowledged polygon uses the existing RViz
Surround topic, with no competing native polygon publishers.
Physical footprints, inflation, ordinary obstacle clearing, other collision zones,
sensor watchdogs and the latched emergency stop are unchanged.
This is a bounded two-profile experiment, not continuous braking-distance
scaling, heading-aware planning or moving-actor trajectory prediction.

Use `bash tests/run_navigation_environment.sh --adaptive-surround --cases ...`
for opt-in corridor/crossing trials. The nominal aligned threshold then uses
the 1.10 m crawl width instead of the 1.50 m fixed width; 1.4 m is expected to
traverse and 0.6 m to hold safely. Width alone does not guarantee traversal.
Reports preserve heartbeat samples, the adaptive-config hash and live
physical-footprint/final-gate checks. For qualified crawl braking, use
`bash tests/run_navigation_environment.sh --braking --adaptive-surround --linear-speeds 0.1 --angular-speeds 0.2 --repeats 1`.
The rear/side obstacles are placed at the crawl boundary, while the front
case retains the unchanged frontal Stop boundary. A case must have confirmed
cruise speed and the crawl profile together before obstacle insertion;
already-stopped cases cannot count as successful braking.
The adaptive braking run also forcibly kills the selector last: it requires
a final zero command within 0.35 s and confirmed physical standstill within
0.5 s, while the other safety nodes remain running.
In `20261002_132325`, fast crossing at crawl speed completed twice with
continuous conservative clearance lower bounds of 0.636/0.645 m, but both
1.4 m corridor trials timed out before entering, despite remaining collision-free.
Recorded-pose replay found crawl polygon/wall overlap of about 1.8/1.9 cm.
Do not count those safe holds as traversal or treat the nominal 1.10 m width
as heading-aware feasibility. These low-speed crossing results are not
acceptance of high-speed navigation or an actor-trajectory predictor.
The feature remains experimental and disabled by default.
In `20261002_133659`, baseline, 1.6/1.8 m corridor traversal, 0.6 m safe holding
and slow crossing all passed once. The 0.6 m goal was manually cancelled after
12 s; this is not autonomous rejection. In `20261002_135434`, four qualified
crawl obstacle-braking cases and both fault stops passed; forcibly killing
the selector produced final zero after 0.167 s and physical standstill after
0.217 s. These are limited simulation samples, not safety certification.
The consolidated local evidence is
`ros2_ws/log/navigation_environment/summary_adaptive_20261002.json`.

For opt-in physical navigation regression, stop the ordinary simulation and run
`bash tests/run_navigation_environment.sh` from the repository root. It uses
Isaac Sim's Python launcher, a separate Compose project and ROS domain 189
(`VALIDATION_ROS_DOMAIN_ID` overrides it), without RViz or costmap filter zones.
The default two repetitions cover baseline navigation, two crossing speeds
(0.35/0.65 m/s), blocking obstacles that move away after a confirmed stop in
open space and a 1.8 m corridor, and
1.8/1.6/1.4/0.6 m corridors. `--cases ... --repeats N` selects a subset.
Each run saves geometry, ground-truth trajectories, actual obstacle poses, commands, action outcomes,
and logs under `ros2_ws/log/navigation_environment/<timestamp>/`.
Acceptance requires goal completion for traversable scenarios, safe stopping or action rejection
for corridors no wider than the larger of the padded physical footprint and the
fixed Surround width (currently 1.50 m: the 0.6/1.4 m cases), and at least 2 cm conservative body clearance
including a between-sample motion allowance. Corridor traversal must enter
between the walls, not detour around them. Failures produce a nonzero exit
status; unsafe clearance latches the emergency stop and aborts the suite.
Protection parameters are not relaxed. Clearance uses a conservative
2D USD body envelope, not a PhysX contact sensor or safety certification.
Results separately diagnose the first clearance breach. `stationary_moving_actor_intrusion`
requires at least 0.25 s of verified linear/angular standstill, bounded pose drift,
fresh continuously zero final commands, and verified actor telemetry; freezing the
actor must remove the breach while freezing the robot must not. Missing evidence or
recent robot motion is not classified as passive. `robot_motion_clearance_breach`
means motion at the first breach; `insufficient_stationary_dwell` means the robot
is stationary then but has not satisfied the preceding 0.25 s window.
These are conservative envelope
intrusions, not confirmed PhysX contacts. Classification never waives the 2 cm criterion:
`collision_free_pass` and overall `pass` remain false for any unsafe clearance or overlap,
and emergency stops still latch and abort the suite. Old recordings without angular
velocity evidence cannot establish this stationary classification. Uncooperative moving
actors can intrude into an already stationary robot; trajectory prediction is not implemented.
A safe stop is not counted as successful navigation where reaching the goal is required.
Runtime global/local footprint parameters are checked against the configuration before
trials, and results record the planning-config hash and nominal corridor-width threshold.
Blocked cases are observed for 12 s and then manually cancelled: safe holding is not
proof of autonomous action rejection or successful replanning. In the 2026-10-02
rerun (14 trials, using the temporary 1.62 m global planning footprint),
0.6/1.4/1.6 m corridors safely held in both repetitions, 1.8 m
completed once in two attempts, slow crossing completed twice, and fast crossing
completed once with one clearance/overlap failure. The failed fast trial was
stationary for only about 0.117 s before its first continuous-clearance breach,
so it does not establish a verified stationary-actor intrusion. The 1.8 m failure
still had Surround overlapping a wall; expanding global planning does not solve
heading-dependent deadlocks. The consolidated local evidence is
`ros2_ws/log/navigation_environment/summary_alignment_20261002_103944.json`.
That temporary planning footprint has since been removed; the historical results
are not a navigation acceptance run of the restored physical footprint.

Nav2 plans on the global costmap (PGM static layer plus 3D obstacle marking) and
follows paths using the local obstacle costmap. Both footprints represent the
physical robot: x [-0.65, 0.20], y +/-0.32 m, plus 1 cm padding (0.66 m total width).
Surround is not included in the global footprint. NavFn still uses a 2D inflated
grid, not a heading-aware safety-polygon sweep. Both obstacle layers retain the
default `footprint_clearing_enabled: true`: only the physical footprint is cleared,
not the Surround region. Self filtering remains upstream and scan raytracing remains enabled. Obstacles missing from the PGM,
such as desks seen above the 2D slice or objects moved after mapping, are marked
in both costmaps, so the 1 Hz replanning routes around them. Controller and
recovery commands go to `/nav2/cmd_vel_nav`; Nav2's `velocity_smoother`
(`config/navigation.yaml`, open loop, 0.8 m/s² acceleration and 1.5 m/s²
deceleration) publishes the ramped `/nav2/cmd_vel`. Safety stops happen after
the smoother, so they remain immediate. `/nav2/cmd_vel` then passes through the direction-zone
selector (`/nav2/cmd_vel_direction`) and Nav2's `collision_monitor`
(`config/collision_monitor.yaml`: front and surround stop zones, a front slowdown zone and
a footprint time-to-collision check on `/perception/obstacles` and `/scan`),
then through a ROS 2 safety node (PCD correction and obstacle sensor
freshness, planar command validation, speed limiting and restart acceleration
limiting) to `/cmd_vel`, which
Isaac Sim's native ROS 2 Subscribe Twist node receives to drive Carter. There
is no Unix socket or separate command transport.
The safety node actively publishes zero velocity at 10 Hz when neither `/scan`
nor `/perception/obstacles` has a timestamp less than 1 second old, including
before the first sensor message arrives and when incoming commands stop.
Either source recovering allows new commands again; old commands are not replayed.
The final safety gate tracks its published commands and limits increases in
speed to 0.8 m/s² and 1.5 rad/s², including after collision-monitor or watchdog
stops. Braking and zero commands remain immediate; direction changes pass through
zero before accelerating in the opposite direction. These acceleration limits
and a 0.5 s command timeout are configured under `cmd_vel_safety` in
`config/collision_monitor.yaml`. Acceleration uses ROS time with at most 0.1 s
credited per command; command gaps and clock resets restart from zero.
`cmd_vel_safety.sensor_timeout` in `config/collision_monitor.yaml` must match
the monitor's `source_timeout`. Freshness uses ROS time, so pausing `/clock`
also pauses expiry; clock resets clear sensor and correction freshness state.
For obstacle-enabled 3D navigation, `ground_obstacle_filter` removes returns
inside Carter's extruded footprint (`self_filter_bounds` in
`slam_nav/config/ground_obstacle_filter.yaml`), rather than discarding everything
within 0.5 m. Its `/perception/self_filtered_points` retains ground points and
feeds `/scan` with `range_min: 0.0`; `/perception/obstacles` retains nearby external
obstacles after ground removal. The surround stop zone spans x [-1.35, 0.80] m
and y ±0.75 m; the front stop zone extends to x 0.85 m. The earlier, smaller
zones failed conservative clearance checks at 0.75 m/s and were enlarged.
USD geometry checks cover Carter's visible body and 10 enabled collision shapes;
the footprint contains them with about 6 cm longitudinal and 7 cm lateral margin,
plus an explicit 1 cm costmap padding. Before correcting the front convention
to drive-wheels-first, thirty physical-box stopping trials passed
at commanded ±0.25/0.50/0.75 m/s and ±0.35/0.70 rad/s, with at least 2 cm
clearance for both the measured body envelope and padded footprint.
See `docs/nav.md` for measured distances, exact test conditions and reproduction
commands (`tests/validate_carter_geometry.py`, `tests/validate_braking.py`).
The wider zones can block narrow passages; sensor loss still allowed about
0.66 m travel in the 0.75 m/s fault-injection trial before the robot stopped.
Physical LiDAR occlusion and minimum measurement range
remain limitations; these settings are not certified safety clearances.
The controller is `slam_nav::GoalHeadingLatchedRPP`, a thin wrapper around
Humble's Regulated Pure Pursuit, paired with `slam_nav::LatchedGoalChecker`.
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
`max_linear_speed` 0.75 m/s and `max_angular_speed` 0.5 rad/s
(`cmd_vel_safety` in `config/collision_monitor.yaml`, overridable per robot profile). Isaac Sim also checks command bounds and stops on a
0.5 s command timeout if ROS messages stop; stale PCD corrections suppress
movement. Nav2's regulated pure pursuit controller targets 0.75 m/s
(`desired_linear_vel` in `config/navigation.yaml`), subject to its approach,
curvature, and collision speed reductions. It uses a fixed 0.8 m lookahead
to reduce side-to-side corrections on straight paths; check corner tracking
and clearance before using longer or tighter routes. Manual W/S keyboard
jogging commands
0.75 m/s in either direction; manual rotation is 0.5 rad/s and auto-jog remains
at 0.2 m/s. Both robot profiles use these limits. Acceleration and collision zones
are unchanged; reducing top speed does not establish slip-free operation.
Historically, at the former 1.0 m/s / 0.75 rad/s limits, one Isaac Sim run passed
four physical-obstacle stopping cases and one sensor-watchdog case, plus a
baseline Nav2 goal. Measured linear cruise was about 1.0 m/s; measured angular
cruise was 0.65-0.69 rad/s for a 0.75 rad/s command. Sensor loss allowed about
0.85 m travel before stopping. See `docs/nav.md` for results and reproduction
commands; this single simulation run does not validate all scenarios or real hardware.
`--navigate` cannot be combined with
`--auto-jog`. Unlike observation-only costmaps, Nav2's planner and controller
own both costmaps. This configuration replans periodically and recovers from
planner or controller failures (`config/navigate_to_pose.xml`): a failed
planner or controller clears its costmap and retries once; if navigation still
fails, `behavior_server` runs one recovery per failure in rotation (clear both
costmaps, wait 5 s, back up 0.3 m, wait 10 s) and retries, up to 6 times
(about 30 s) before aborting. Back-up checks the local costmap footprint and
is skipped when blocked, and its `cmd_vel` goes through the same collision
monitor and safety node.
Carter's rectangular footprint and inflation are estimates,
and dynamic obstacle clearing and localization failure response are not yet
validated for safe autonomous operation. Test only in a clear Office
simulation and inspect the costmaps and planned path before longer drives.

To test obstacle avoidance, `--box X,Y[,SX,SY,SZ]` places a static collision
box (default 0.6 x 0.6 x 1.0 m) in the Isaac Sim scene at Office map X,Y
meters; repeat it for more boxes. It works in both modes. Carter spawns at
(0, 0) facing +x (drive wheels leading), and x from -1 to 4.5 m is open floor, for example:

```bash
./scripts/run_nav.sh --mode 3d --navigate --box 2.0,0.0
```

The box is not in the saved PGM/PCD maps; the costmaps only see it through the
LiDAR. A goal at (3.5, 0) then plans around it.

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
`base_link`-relative rectangular footprint (front 0.20 m, rear 0.65 m, left
and right 0.32 m). It approximates the Nova Carter USD body and wheels with
about 6 cm of clearance; replace it when changing robots. To measure another
robot, run `scripts/usd_bbox.py` from the repository root with Isaac Sim's Python, for example
`./scripts/usd_bbox.py /Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd --frame chassis_link --padding 0.06`
(Carter's ROS `base_link` faces USD `chassis_link` +x). It prints the
bounding box and a Nav2 `footprint` string (`--shape hull` for a convex hull,
`--json` for machine-readable output). The collision_monitor stop and slowdown
polygons are separate hardcoded values and must be updated by hand to match.
The 0.9 m inflation
radius remains an estimate, and neither setting is a validated safety clearance.
Low obstacles absent from the height-filtered
`/scan` might not clear reliably after moving; verify marking and clearing
in your scene before using these layers for navigation. Without `--navigate`,
this mode does not publish driving commands or start autonomous navigation.
Do not run it alongside `scripts/run_nav.sh` or the original 3D demo. Custom PCD/PGM
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

Use `./scripts/run_nav.sh --help` for all options.
