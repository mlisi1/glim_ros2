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
| Odometry child frame | Always the IMU frame | `odom_child_frame`: `imu`, `lidar` or `base` (poses, twists and covariances transformed) |
| Outputs | ~30 publishers always created | `outputs` parameter selects which topics exist |
| Topic naming | `odom_corrected` means "optimized", not "loop-closure corrected" | Added `~/odometry/local` and `~/odometry/global` with explicit semantics |
| Viewer | Closing a viewer window shuts the whole node down | SLAM keeps running (`shutdown_on_extension_exit`) |
| Clouds | `aligned_points` in GLIM's odom frame but labelled `map` | Transformed into `map` |
| Map | Full-resolution concatenation of all submaps every 10 s, built in the publishing thread | Per-submap voxelized cache, rebuilt with the current (loop-closed) poses only when changed, in its own thread |
| Navigation map | None | `~/grid` 2D occupancy grid (`nav_msgs/OccupancyGrid`) |
| Saving | Only at shutdown, to `/tmp/dump` | `~/save_map` service at any time: GLIM dump + `map.pcd` + `map.pgm`/`map.yaml` (nav2_map_server) |

## How it works

```
 /imu, /points ──► GlimROS node (glim_rosnode)
                    │  preprocess → odometry estimation → sub mapping → global mapping   (GLIM library threads)
                    │
                    │  GLIM callbacks (on_insert_imu, on_new_frame, on_update_new_frame, on_update_submaps)
                    ▼
                 ROS output module (librviz_viewer.so, loaded as a GLIM extension module)
                    │  callback side (estimation threads): enqueue frame + map->odom snapshot, buffer gyro
                    │  worker thread: build messages, transform frames/covariances, publish, broadcast TF
                    ▼
                 ~/odometry/*, ~/odom*, ~/pose*, ~/points*, TF
                    │
                    │  map thread: cache each new submap (voxelized), keep the latest submap poses,
                    ▼  rebuild ~/map and ~/grid when changed (low priority, never blocks odometry/TF)
                 ~/map, ~/grid, map.pcd / map.pgm / map.yaml on save
```

* **GlimROS** (`src/glim_ros/glim_ros.cpp`) subscribes to IMU / point cloud (/ image) topics, feeds GLIM
  and moves results between the asynchronous GLIM stages. GLIM's configuration (sensor topics, frame
  IDs, modules) is read from the JSON files in `config_path`.
* **The ROS output module** (`src/glim_ros/rviz_viewer.cpp`, historically called "rviz viewer") turns
  GLIM estimates into ROS messages. GLIM invokes its callbacks synchronously from the estimation
  threads, so the module only stores the (immutable) estimation frame and returns; everything else
  happens in its own thread. If publishing falls behind, the oldest frames are dropped
  (`max_publish_queue`) rather than slowing down SLAM.
* For each scan GLIM emits two estimates: a prediction (`on_new_frame`, IMU integration only) and the
  optimized one (`on_update_new_frame`). Topics without the `_corrected` suffix carry the prediction,
  `_corrected` topics and TF carry the optimized estimate. Neither includes loop closures; those are
  applied through `map->odom` (`T_world_odom`, updated when the global map is optimized).

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
| `publish_tf` | `true` | Master default for the three switches below |
| `publish_tf_map_odom` | `publish_tf` | Publish `map -> odom` |
| `publish_tf_odom_base` | `publish_tf` | Publish `odom -> base` (ignored in external mode) |
| `publish_tf_imu_lidar` | `publish_tf && publish_imu2lidar` | Publish `imu -> lidar` (usually leave to the URDF) |
| `map_odom_mode` | `internal` | `internal` or `external` (see TF modes) |
| `map_odom_planar` | `false` | Flatten `map -> odom` to x, y, yaw |
| `map_odom_rate` | `20.0` | [Hz] `map -> odom` republish rate (external mode) |
| `map_odom_tolerance` | `0.1` | [s] future-dating of the republished `map -> odom` |
| `external_odom_timeout` | `0.1` | [s] wait for the external `odom -> base` at the scan stamp |
| `odom_child_frame` | `imu` | Body frame of `odom*`, `pose*`, `odometry/*`: `imu`, `lidar`, `base` |
| `twist_angular_window` | `0.1` | [s] gyro averaging window around the scan stamp |
| `twist_angular_cov` | `0.01` | [rad²/s²] variance reported for the angular velocity |
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

Sensor topics, frame IDs, QoS and GLIM modules are still configured in `config_ros.json` /
`config_*.json` of `config_path`.

## Topics (all under the node namespace, `/glim_ros/...` by default)

Publishing is lazy: a message is only built when the topic has subscribers.

| Name | Type | Frame | Content |
|---|---|---|---|
| `odometry/local` | `nav_msgs/Odometry` | `odom` → child | Optimized odometry (no loop closure); twist in child frame |
| `odometry/global` | `nav_msgs/Odometry` | `map` → child | Same with loop closure applied |
| `odom`, `odom_corrected` | `nav_msgs/Odometry` | `odom` → child | Prediction / optimized (upstream names) |
| `odom_scanend`, `odom_scanend_corrected` | `nav_msgs/Odometry` | `odom` → child | Pose at the end of the scan (needs `save_imu_rate_trajectory`) |
| `lidar_odom*` | `nav_msgs/Odometry` | `odom` → lidar | As above, always in the LiDAR frame |
| `pose`, `pose_corrected`, `pose_scanend*` | `geometry_msgs/PoseStamped` | `map` | Body pose with loop closure |
| `pose_corrected_with_cov`, `pose_scanend_corrected_with_cov` | `PoseWithCovarianceStamped` | `map` | With covariance (when GLIM computes it) |
| `lidar_pose*` | Pose(WithCovariance)Stamped | `map` | LiDAR pose |
| `imu_bias`, `imu_bias_with_cov` | `Twist(WithCovariance)Stamped` | imu | [acc bias; gyro bias] |
| `points`, `points_corrected` | `PointCloud2` | sensor | Preprocessed scan |
| `aligned_points`, `aligned_points_corrected` | `PointCloud2` | `map` | Scan in the map frame |
| `map` | `PointCloud2` (latched) | `map` | Voxelized map, rebuilt when changed (see above) |
| `grid` | `nav_msgs/OccupancyGrid` (latched) | `map` | 2D occupancy grid |

Service: `save_map` (`std_srvs/Trigger`) — response `message` is the output directory.

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
        'outputs': ['odometry/local', 'odometry/global', 'grid'],
        'map_save_directory': '~/glim_maps',
    }],
)
```

and in the EKF: `odom0: /glim_ros/odometry/local`, `world_frame: odom`, `publish_tf: true`.

## Build

Same as upstream (see the [GLIM docs](https://koide3.github.io/glim/)). Requires a GLIM version that
provides frame covariances (`EstimationFrame::pose_cov`, koide3/glim ≥ 1a197c9, Oct 2026).

## Upstream

Original package: [koide3/glim_ros2](https://github.com/koide3/glim_ros2) — GLIM:
[koide3/glim](https://github.com/koide3/glim). Please follow the licenses of GLIM and its extension
modules.
