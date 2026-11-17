# glim_ros2 (modified fork)

> **This is a modified version of [koide3/glim_ros2](https://github.com/koide3/glim_ros2).**
> It keeps GLIM itself untouched and reworks the ROS 2 interface so that GLIM can be used as a
> component of a navigation stack (robot_localization, Nav2) instead of only as a standalone
> mapping/visualization tool. Issues about the SLAM algorithm belong to the
> [main GLIM repository](https://github.com/koide3/glim); issues about the changes below belong here.

All upstream topics, parameters and defaults are kept unless stated otherwise, so existing launch
files keep working. The work happens on the `dev` branch.

## What is different from upstream

| Area | Upstream | This fork |
|---|---|---|
| Publishing | Done synchronously inside GLIM's odometry thread; every subscriber adds latency to SLAM | Callbacks only enqueue frames; a worker thread converts and publishes (bounded queue) |
| TF | Always on (only `imu->lidar` configurable); `imu->lidar` carries the inverse transform | Per-edge switches as ROS parameters; one TF per scan, from the optimized estimate; `imu->lidar` fixed |
| `map->odom` | Only relative to GLIM's own odometry, so GLIM must own the whole tree | `map_odom_mode:=external`: `map->odom` computed against an external `odom->base` (e.g. robot_localization), like AMCL / slam_toolbox |
| `nav_msgs/Odometry` twist | World-frame velocity labelled as body frame; angular velocity always 0 | Twist in `child_frame_id` (with lever arm); angular velocity = bias-corrected gyro |
| Frame origin | `map`/`odom` start at the IMU pose at initialization (offset by the IMU lever arm from `base_link`) | `anchor_frame:=base`: `map`/`odom` start at the initial base pose (x, y, z, yaw), as AMCL / slam_toolbox |
| Odometry child frame | Always the IMU frame | `odom_child_frame`: `imu`, `lidar` or `base` (poses, twists and covariances transformed) |
| Outputs | ~30 publishers always created | `outputs` parameter selects which topics exist |
| Topic naming | `odom_corrected` means "optimized", not "loop-closure corrected" | Added `~/odometry/local` and `~/odometry/global` with explicit semantics |
| Viewer | Closing a viewer window shuts the whole node down | SLAM keeps running (`shutdown_on_extension_exit`) |
| Clouds | `aligned_points` in GLIM's odom frame but labelled `map` | Transformed into `map` |
| Map | Full-resolution concatenation of all submaps every 10 s, built in the publishing thread | Per-submap voxelized cache, rebuilt with the current (loop-closed) poses only when changed, in its own thread |
| Navigation map | None | `~/grid` 2D occupancy grid (`nav_msgs/OccupancyGrid`) |
| Saving | Only at shutdown, to `/tmp/dump` | `~/save_map` service at any time: GLIM dump + `map.pcd` + `map.pgm`/`map.yaml` (nav2_map_server) |
| Images | Always subscribed (with cv_bridge), raw transport only | Only if `image_topic` is non-empty; transport from `image_transport` or the topic suffix (`.../compressed`) |
| Configuration | JSON files only | `config_ros.json` keys (topics, frames, offsets, modules...) can be overridden by ROS parameters; several parameters can be changed at runtime |
| Covariances | All zeros unless GLIM's `compute_covs` is enabled; body-frame | Configurable fallbacks; pose covariance expressed about the parent frame's axes (REP-103/ROS convention) |
| Execution | Single-threaded; point cloud preprocessing blocks IMU callbacks; 1 kHz polling timer | Multi-threaded executor with separate IMU / points / timer callback groups; 100 Hz timer |
| Monitoring | None | `/diagnostics` (input/output rates, workload, stalls); `~/odometry/imu_rate` |

## How it works

```
 /imu, /points ──► GlimROS node (glim_rosnode)
                    │  preprocess → odometry estimation → sub mapping → global mapping   (GLIM library threads)
                    │
                    │  GLIM callbacks (on_insert_imu, on_new_frame, on_update_new_frame, on_update_submaps)
                    ▼
                 ROS output module (librviz_viewer.so, loaded as a GLIM extension module)
                    │  callback side (estimation threads): enqueue frame + map->odom snapshot, buffer IMU
                    │  worker thread: build messages, transform frames/covariances, publish, broadcast TF
                    ▼
                 ~/odometry/*, ~/odom*, ~/pose*, ~/points*, TF
                    │
                    │  map thread: cache each new submap (voxelized), keep the latest submap poses,
                    ▼  rebuild ~/map and ~/grid when changed (separate thread, never blocks odometry/TF)
                 ~/map, ~/grid, map.pcd / map.pgm / map.yaml on save
```

* **GlimROS** (`src/glim_ros/glim_ros.cpp`) subscribes to IMU / point cloud (/ image) topics, feeds GLIM
  and moves results between the asynchronous GLIM stages. IMU and point clouds are handled in separate
  callback groups of a multi-threaded executor, so cloud conversion/preprocessing never delays IMU
  input. GLIM's configuration is read from the JSON files in `config_path`; the ROS-side keys can be
  overridden by ROS parameters.
* **The ROS output module** (`src/glim_ros/rviz_viewer.cpp`, historically called "rviz viewer") turns
  GLIM estimates into ROS messages. GLIM invokes its callbacks synchronously from the estimation
  threads, so the module only stores the (immutable) estimation frame and returns; everything else
  happens in its own thread. If publishing falls behind, the oldest frames are dropped
  (`max_publish_queue`) rather than slowing down SLAM.
* For each scan GLIM emits two estimates: a prediction (`on_new_frame`, IMU integration only) and the
  optimized one (`on_update_new_frame`). Topics without the `_corrected` suffix carry the prediction;
  `_corrected` topics, `odometry/*` and TF carry the optimized estimate. Odometry in the `odom` frame
  never includes loop closures; those are applied through `map->odom` (`T_world_odom`, updated when the
  global map is optimized), which `pose*`, `odometry/global` and the map frame use.

### Map and occupancy grid

* Each new submap is downsampled once (`map_voxel_size` for the cloud, `grid_resolution` for the grid)
  and cached in its own frame; maps are assembled with the current submap poses, so loop closures are
  reflected. Map and grid are rebuilt at most every `map_publish_period`, only when they changed and
  someone is subscribed (or a new subscriber appears; both topics are latched).
* Grid: for every submap, the local ground is the lowest point of that submap in each
  `grid_ground_cell` column (per submap, because overlapping submaps can be vertically offset by more
  than the obstacle band). Points `grid_z_min`..`grid_z_max` above the local ground are obstacles
  (a cell needs `grid_min_points` of them); lower points are ground. Free space is carved by rays from
  the submap origin to every observed cell (up to `grid_max_range`). Unobserved cells are unknown.
  The grid is a static map for localization/planning; Nav2 costmaps should still use live sensor data.
* `~/save_map` (`std_srvs/Trigger`) saves to `map_save_directory/<YYYYmmdd_HHMMSS>/` in a background
  thread and returns the path immediately. The same files are written to `dump_path` at shutdown.
  The dump can be opened with `offline_viewer`; `map.yaml` loads in `nav2_map_server`.

### Frames

* `odom` (`odom_frame_id`): GLIM's odometry frame — smooth, drifts.
* `map` (`map_frame_id`): loop-closure-corrected frame. `T_map_body = T_map_odom * T_odom_body`.
* origin (`anchor_frame`): GLIM's world frame is gravity aligned with its origin at the IMU at
  initialization. With `anchor_frame:=imu` (default, upstream) `map` and `odom` are that frame, so with the
  robot at rest `map -> base_link` starts at the IMU -> base lever arm. With `anchor_frame:=base` every
  output (TF, `odom*`, `pose*`, `odometry/*`, clouds, `~/map`, `~/grid`, `map.pcd`, `map.yaml`) is
  re-anchored by `A = T_world_base0^-1`, where `T_world_base0` is the base pose of the first frame reduced
  to x, y, z and yaw (so the frames stay gravity aligned): `map -> base_link` starts at (0, 0, 0, yaw 0)
  up to the base's own roll/pitch. Outputs wait until `imu -> base` is in TF. GLIM's own dump stays in its
  world frame; `map_anchor.txt` in the saved directory holds `A`.
* body frames: `imu_frame_id`, `lidar_frame_id`, `base_frame_id` (from `config_ros.json`; IMU / LiDAR
  are auto-detected from the message headers when empty). `imu -> base` is looked up once from TF
  (normally `robot_state_publisher` / URDF) and cached.

### TF modes

**Standalone (default, `map_odom_mode:=internal`)** — GLIM owns `map -> odom -> base`
(and optionally `imu -> lidar`).

**With an external odometry (`map_odom_mode:=external`)** — e.g. robot_localization publishes
`odom -> base_link` fusing GLIM odometry, wheel odometry, IMU... GLIM then publishes only `map -> odom`:

```
T_map_odom = T_map_base(GLIM, with loop closure, at scan stamp) * T_odom_base(external, same stamp)^-1
```

It is republished at `map_odom_rate` with a stamp of `now + map_odom_tolerance` (as AMCL's
`transform_tolerance`), so lookups at the current time succeed. `publish_tf_odom_base` is forced to
false in this mode. `map_odom_planar:=true` keeps only x, y, yaw (useful with a 2D EKF / Nav2).

## Parameters (ROS, on the `glim_ros` node)

| Parameter | Default | Description |
|---|---|---|
| `config_path` | `config` | GLIM config directory (relative paths resolved against the `glim` package) |
| `imu_topic`, `points_topic`, `image_topic` | from `config_ros.json` | Input topics (empty `image_topic`: no image subscription) |
| `image_transport` | `raw` | image_transport plugin; overridden by a transport suffix in `image_topic` (`/compressed`, `/compressedDepth`, `/theora`, `/zstd`, `/ffmpeg`) |
| `imu_frame_id`, `lidar_frame_id`, `base_frame_id`, `odom_frame_id`, `map_frame_id` | from `config_ros.json` | Frame IDs (IMU / LiDAR auto-detected when empty; base defaults to IMU) |
| `imu_time_offset`, `points_time_offset`, `acc_scale`, `ang_scale`, `keep_raw_points` | from `config_ros.json` | Input corrections |
| `enable_local_mapping`, `enable_global_mapping`, `extension_modules` | from `config_ros.json` | Pipeline composition |
| `intensity_field`, `ring_field` | from `config_sensors.json` | Point cloud fields |
| `tf_time_offset` | from `config_ros.json` | [s] added to TF stamps |
| `publish_tf` | `true` | Master default for the three switches below |
| `publish_tf_map_odom` | `publish_tf` | Publish `map -> odom` |
| `publish_tf_odom_base` | `publish_tf` | Publish `odom -> base` (ignored in external mode) |
| `publish_tf_imu_lidar` | `publish_tf && publish_imu2lidar` | Publish `imu -> lidar` (usually leave to the URDF) |
| `map_odom_mode` | `internal` | `internal` or `external` (see TF modes) |
| `map_odom_planar` | `false` | Flatten `map -> odom` to x, y, yaw |
| `map_odom_rate` | `20.0` | [Hz] `map -> odom` republish rate (external mode) |
| `map_odom_tolerance` | `0.1` | [s] future-dating of the republished `map -> odom` |
| `external_odom_timeout` | `0.1` | [s] wait for the external `odom -> base` at the scan stamp |
| `anchor_frame` | `imu` | Origin of `map` / `odom`: `imu` (upstream) or `base` (initial base pose, see Frames) |
| `odom_child_frame` | `imu` | Body frame of `odom*`, `pose*`, `odometry/*`: `imu`, `lidar`, `base` |
| `twist_angular_window` | `0.1` | [s] gyro averaging window around the scan stamp |
| `twist_angular_cov` | `0.01` | [rad²/s²] variance reported for the angular velocity |
| `fallback_position_var`, `fallback_rotation_var`, `fallback_velocity_var` | `0.01`, `0.001`, `0.01` | Variances reported when GLIM does not compute covariances (`compute_covs: false`) |
| `outputs` | `["all"]` | Topics to create (names below, without `~/`) |
| `max_publish_queue` | `64` | Frames buffered for publishing before dropping the oldest |
| `shutdown_on_extension_exit` | `false` | Upstream behavior: shut down when a viewer window is closed |
| `map_save_directory` | `/tmp/glim_maps` | Where `~/save_map` writes (`~` expanded) |
| `map_voxel_size` | `0.2` | [m] voxel size of `~/map` and `map.pcd` |
| `map_publish_period` | `10.0` | [s] minimum period between `~/map` / `~/grid` rebuilds |
| `grid_resolution` | `0.1` | [m] occupancy grid cell size |
| `grid_z_min`, `grid_z_max` | `0.3`, `1.5` | [m] obstacle band above the local ground |
| `grid_min_points` | `3` | Points in the band (at grid resolution) for a cell to be occupied |
| `grid_ground_cell` | `1.0` | [m] column size for the local ground estimate |
| `grid_max_range` | `30.0` | [m] maximum range used for obstacles and free-space carving |
| `debug`, `dump_on_unload`, `dump_path` | | As upstream |
| `debug_log_path` | `/tmp/glim_log.log` | Log file when `debug` is true |

QoS profiles and the GLIM algorithm parameters stay in the JSON files of `config_path`.

**Runtime changes** (`ros2 param set /glim_ros <name> <value>`): `publish_tf*`, `tf_time_offset`,
`map_odom_planar`, `map_odom_rate`, `map_odom_tolerance`, `external_odom_timeout`, `twist_angular_*`,
`fallback_*_var`, `map_publish_period`, `grid_z_min`, `grid_z_max`, `grid_min_points`, `grid_max_range`,
`grid_ground_cell`. Frame IDs, `odom_child_frame`, `anchor_frame`, `outputs`, `map_odom_mode`, `map_voxel_size` and
`grid_resolution` are rejected at runtime (restart required); the others are read at startup.

## Topics (all under the node namespace, `/glim_ros/...` by default)

Publishing is lazy: a message is only built when the topic has subscribers.

| Name | Type | Frame | Content |
|---|---|---|---|
| `odometry/local` | `nav_msgs/Odometry` | `odom` → child | Optimized odometry (no loop closure); twist in child frame |
| `odometry/global` | `nav_msgs/Odometry` | `map` → child | Same with loop closure applied |
| `odometry/imu_rate` | `nav_msgs/Odometry` | `odom` → child | Latest optimized state propagated with every new IMU sample (bias-corrected strapdown); low latency, but inherits GLIM's velocity error between scans |
| `odom`, `odom_corrected` | `nav_msgs/Odometry` | `odom` → child | Prediction / optimized (upstream names) |
| `odom_scanend`, `odom_scanend_corrected` | `nav_msgs/Odometry` | `odom` → child | Pose at the end of the scan (needs `save_imu_rate_trajectory`) |
| `lidar_odom*` | `nav_msgs/Odometry` | `odom` → lidar | As above, always in the LiDAR frame |
| `pose`, `pose_corrected`, `pose_scanend*` | `geometry_msgs/PoseStamped` | `map` | Body pose with loop closure |
| `pose_corrected_with_cov`, `pose_scanend_corrected_with_cov` | `PoseWithCovarianceStamped` | `map` | With covariance (GLIM's, or the fallback variances) |
| `lidar_pose*` | Pose(WithCovariance)Stamped | `map` | LiDAR pose |
| `imu_bias`, `imu_bias_with_cov` | `Twist(WithCovariance)Stamped` | imu | [acc bias; gyro bias] |
| `points`, `points_corrected` | `PointCloud2` | sensor | Preprocessed scan |
| `aligned_points`, `aligned_points_corrected` | `PointCloud2` | `map` | Scan in the map frame |
| `map` | `PointCloud2` (latched) | `map` | Voxelized map, rebuilt when changed (see above) |
| `grid` | `nav_msgs/OccupancyGrid` (latched) | `map` | 2D occupancy grid |

Service: `save_map` (`std_srvs/Trigger`) — response `message` is the output directory.

Diagnostics: `/diagnostics`, status `GLIM` — IMU / point cloud / odometry rates, rejected clouds,
odometry workload; WARN when inputs stop or odometry falls behind, ERROR when clouds arrive but no
odometry is produced. Covariances in `odom*`, `pose*_with_cov` and `odometry/*` are expressed about
the axes of the header frame (rotated from GTSAM's body frame).

Saving at shutdown happens after Ctrl-C and can take a few seconds on large maps; increase the launch
`sigterm_timeout` (default 5 s) or call `save_map` before stopping.

## Example: GLIM + robot_localization + Nav2

```python
Node(
    package='glim_ros', executable='glim_rosnode',
    parameters=[{
        'config_path': glim_config_dir,
        'map_odom_mode': 'external',      # EKF owns odom -> base_link, GLIM owns map -> odom
        'map_odom_planar': True,          # 2D EKF / Nav2
        'odom_child_frame': 'base',       # odometry expressed in base_link
        'anchor_frame': 'base',           # map / odom origin at the initial base_link pose
        'outputs': ['odometry/local', 'odometry/global', 'grid'],
        'map_save_directory': '~/glim_maps',
    }],
)
```

and in the EKF: `odom0: /glim_ros/odometry/local`, `world_frame: odom`, `publish_tf: true`.

## Build

Same as upstream (see the [GLIM docs](https://koide3.github.io/glim/)), plus the ROS packages
`std_srvs`, `diagnostic_updater` and `diagnostic_msgs` (`rosdep install --from-paths src -y`).

GLIM must provide frame covariances (`EstimationFrame::pose_cov`, added in koide3/glim `d0eeebe`,
included in v1.2.2); older commits such as `a9be365` do not compile. Tested with GLIM `1a197c9`
(Oct 2026), which also brings robust initial state estimation, a fix for crashes after long
interruptions and a fmt ≥ 11 build fix. Building v1.2.2 itself has not been tested.

Note on gtsam_points: GLIM master's GPU odometry (`odometry_estimation_gpu.cpp`) uses
`IndexedSlidingWindow::has_index()`, which is newer than gtsam_points 1.2.2 (the latest PPA package).
A CPU-only build works with the PPA's `libgtsam-points-dev` 1.2.2; a CUDA build of GLIM master needs
gtsam_points built from source (a commit that has `has_index`).

## Known limitations

* **No localization-only mode** (load a saved map, relocalize, `/initialpose`). It needs changes inside
  GLIM's mapping modules, not only in this ROS interface, so it is out of scope for this fork.
* **Per-point LiDAR timestamps are needed for deskewing and the `*_scanend` outputs.** Clouds stamped at
  the *end* of the sweep (e.g. the Velodyne driver, per-point times ≈ −0.1…0 s) are fine: GLIM detects the
  negative times and moves the frame stamp to the first point (measured on a Velodyne bag: 0.10 s scans).
  With `global_shutter_lidar: true` every scan has zero duration: no deskewing, and `*_scanend` equals the
  scan pose. The node warns after 10 consecutive zero-duration frames; isolated ones are normal (about 1 in 4
  on the Velodyne bag, never more than 5 in a row) and not reported.
* `odometry/imu_rate` propagates GLIM's velocity estimate between scans, so its error grows with the
  velocity error; prefer `odometry/local` as an EKF input unless the velocity estimate is good.
* The occupancy grid is a 2.5D height-band heuristic (good for static maps, not a costmap); tall
  vegetation and overhangs need tuning of `grid_z_*`.
* One GLIM instance per process (GLIM uses process-wide singletons and static callbacks).
* The ROS output module keeps the upstream library name `librviz_viewer.so` so existing configs work.
* No lifecycle-node variant.

## Validation

Changes were validated by replaying recorded robot bags (Velodyne + IMU, CPU odometry) into
`glim_rosnode` on sim time and comparing against the unmodified upstream build:

* TF: no duplicated stamps (upstream: every `map->odom`/`odom->base` stamp sent twice); one parent per
  frame with robot_localization in `map_odom_mode:=external`, and `map->odom ∘ odom->base` reproducing
  GLIM's global pose (median 3 cm with the same-scan transform).
* Twist: body-frame velocity matches pose differentiation better than a world-frame reading (upstream:
  the opposite); yaw rate correlates 0.995 with pose differentiation (upstream: always 0).
* Latency of `odom_corrected` unchanged (~0.19 s) with every heavy topic subscribed.
* `save_map` during a run (no odometry stall), `map.yaml` loaded by `nav2_map_server`; runtime
  parameters, restart-only rejection, `/diagnostics`, `imu->lidar` conflict detection; `glim_rosbag`.

## Upstream

Original package: [koide3/glim_ros2](https://github.com/koide3/glim_ros2) — GLIM:
[koide3/glim](https://github.com/koide3/glim). Please follow the licenses of GLIM and its extension
modules.
