#pragma once

#include <any>
#include <atomic>
#include <deque>
#include <mutex>
#include <optional>
#include <thread>
#include <chrono>
#include <condition_variable>

#include <rclcpp/rclcpp.hpp>
#if __has_include(<tf2_ros/buffer.hpp>)
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#else
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#endif

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <glim/odometry/estimation_frame.hpp>
#include <glim/mapping/sub_map.hpp>
#include <glim/util/extension_module.hpp>
#include <glim/util/extension_module_ros2.hpp>
#include <gtsam_points/types/point_cloud_cpu.hpp>

namespace spdlog {
class logger;
}

namespace glim {

class TrajectoryManager;

/**
 * @brief ROS2 output module (odometry, poses, TF, point clouds, map).
 *
 * GLIM callbacks are invoked synchronously from the estimation threads. To keep the cost of
 * ROS publishing (message conversion, DDS serialization) out of the SLAM pipeline, the callbacks
 * only enqueue the (immutable) estimation frames; all conversion and publishing happen in a
 * dedicated worker thread.
 */
class RvizViewer : public ExtensionModuleROS2 {
public:
  RvizViewer();
  ~RvizViewer();

  virtual std::vector<GenericTopicSubscription::Ptr> create_subscriptions(rclcpp::Node& node) override;

  /// Called by GlimROS::save(): writes map.pcd and map.pgm/map.yaml (nav2_map_server format) into dump_path
  virtual void at_exit(const std::string& dump_path) override;

private:
  /// Frame in which body poses/twists are expressed
  enum class BodyFrame { IMU, LIDAR, BASE };

  struct FrameTask {
    EstimationFrame::ConstPtr frame;
    bool corrected;
    Eigen::Isometry3d T_world_odom;  // map -> odom snapshot taken when the frame was produced
  };

  void set_callbacks();
  void on_new_frame(const EstimationFrame::ConstPtr& new_frame, bool corrected);
  void on_insert_imu(double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel);
  void publish_imu_rate();
  rcl_interfaces::msg::SetParametersResult on_set_parameters(const std::vector<rclcpp::Parameter>& params);
  Eigen::Vector3d mean_gyro(double t0, double t1);
  void globalmap_on_update_submaps(const std::vector<SubMap::Ptr>& submaps);
  void invoke(const std::function<void()>& task);

  // Map / grid (run in map_thread, guarded by map_mutex)
  struct SubmapCache {
    gtsam_points::PointCloud::ConstPtr points;      // voxelized map points (submap origin frame)
    std::vector<Eigen::Vector3d> grid_points;       // grid-resolution points (origin frame) for the occupancy grid
  };
  void map_loop();
  SubmapCache make_submap_cache(const SubMap::ConstPtr& submap);
  gtsam_points::PointCloudCPU::Ptr build_map(const Eigen::Isometry3d& anchor);
  std::shared_ptr<nav_msgs::msg::OccupancyGrid> build_grid(const Eigen::Isometry3d& anchor);
  void publish_map_and_grid();

  void worker_loop();
  void resolve_frame_ids();
  std::optional<Eigen::Isometry3d> lookup_T_imu_base();
  std::optional<Eigen::Isometry3d> resolve_anchor(const Eigen::Isometry3d& T_world_imu);
  std::optional<Eigen::Isometry3d> get_anchor();
  void publish_frame(const FrameTask& task);
  void publish_map_odom_periodic();
  void check_imu_lidar_tf();

  bool should_warn(std::chrono::steady_clock::time_point& last, double period_sec);

private:
  std::atomic_bool kill_switch;
  std::thread thread;
  std::atomic_bool initialized;

  std::mutex queue_mutex;
  std::condition_variable queue_cv;
  std::deque<FrameTask> frame_queue;
  std::vector<std::function<void()>> invoke_queue;
  size_t max_frame_queue;
  std::chrono::steady_clock::time_point last_drop_warn;
  std::chrono::steady_clock::time_point last_scanend_warn;
  int scanend_short_frames = 0;  // consecutive odometry frames whose scan end equals the frame time (publish thread only)

  rclcpp::Clock::SharedPtr clock;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

  rclcpp::Time last_globalmap_pub_time;

  std::string imu_frame_id;
  std::string lidar_frame_id;
  std::string base_frame_id;
  std::string odom_frame_id;
  std::string map_frame_id;
  std::atomic_bool frame_ids_resolved;
  std::atomic<double> tf_time_offset;

  // TF switches (ROS parameters on the glim_ros node)
  std::atomic_bool publish_tf_map_odom;
  std::atomic_bool publish_tf_odom_base;
  std::atomic_bool publish_tf_imu_lidar;

  // map->odom computation
  //  internal: relative to GLIM's own odometry frame (GLIM also owns odom->base)
  //  external: relative to an odom->base transform published by someone else (e.g. robot_localization)
  std::atomic_bool map_odom_external{false};
  std::atomic_bool map_odom_planar;          // keep only x, y, yaw of map->odom
  std::atomic<double> map_odom_rate;          // [Hz] republish rate in external mode
  std::atomic<double> map_odom_tolerance;     // [s] future-dating of republished map->odom (as AMCL's transform_tolerance)
  std::atomic<double> external_odom_timeout;  // [s] max wait for the external odom->base at the scan stamp
  std::optional<Eigen::Isometry3d> latest_T_map_odom;
  rclcpp::Time last_map_odom_pub;

  // Odometry/pose output frame
  BodyFrame odom_child_frame;
  std::atomic<double> twist_angular_cov;     // variance [rad^2/s^2] reported for angular velocity
  std::atomic<double> twist_angular_window;  // gyro averaging window [s] centered on the frame stamp

  // Recent IMU samples [t, ax, ay, az, wx, wy, wz] (angular velocity source, IMU-rate propagation)
  using ImuSample = Eigen::Matrix<double, 7, 1>;
  std::mutex imu_mutex;
  std::deque<ImuSample, Eigen::aligned_allocator<ImuSample>> imu_buffer;
  std::atomic_bool imu_rate_active{false};  // someone subscribes to ~/odometry/imu_rate
  std::atomic_bool new_imu{false};

  // Latest optimized state, starting point of the IMU-rate propagation (worker thread only)
  struct ImuState {
    double stamp;
    Eigen::Isometry3d T_odom_imu;
    Eigen::Vector3d v_odom_imu;
    Eigen::Matrix<double, 6, 1> bias;  // [acc; gyro]
    Eigen::Isometry3d T_lidar_imu;
  };
  std::optional<ImuState> latest_state;
  double last_imu_rate_stamp = 0.0;

  // Covariances used when GLIM does not compute them (compute_covs=false)
  std::atomic<double> fallback_position_var;  // [m^2]
  std::atomic<double> fallback_rotation_var;  // [rad^2]
  std::atomic<double> fallback_velocity_var;  // [m^2/s^2]

  bool imu_lidar_checked = false;
  std::atomic_bool imu_lidar_refused{false};  // LiDAR frame has another parent in TF
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback;
  // Anchor of the published map/odom frames w.r.t. GLIM's world frame (left correction of every output).
  //  imu : identity (upstream: origin at the IMU pose at initialization)
  //  base: inverse of the initial base pose reduced to x, y, z, yaw (keeps the frames gravity aligned)
  bool anchor_base = false;
  std::optional<Eigen::Isometry3d> first_T_world_imu;  // worker thread only
  std::mutex anchor_mutex;
  std::optional<Eigen::Isometry3d> T_map_world;  // set once; nullopt until resolved

  std::mutex T_imu_base_mutex;
  std::optional<Eigen::Isometry3d> T_imu_base;  // cached static transform

  // Publishers: nullptr if disabled through the 'outputs' parameter
  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odometry_local_pub;   // ~/odometry/local
  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odometry_global_pub;  // ~/odometry/global
  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odometry_imu_rate_pub;  // ~/odometry/imu_rate

  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> map_pub;

  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> points_pub;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> points_corrected_pub;

  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> aligned_points_pub;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> aligned_points_corrected_pub;

  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odom_pub;
  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odom_scanend_pub;
  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odom_corrected_pub;
  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odom_scanend_corrected_pub;
  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> lidar_odom_pub;  // Odometry in LiDAR frame
  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> lidar_odom_scanend_pub;
  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> lidar_odom_corrected_pub;
  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> lidar_odom_scanend_corrected_pub;

  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>> pose_pub;
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>> pose_scanend_pub;
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>> pose_corrected_pub;
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>> pose_scanend_corrected_pub;
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>> pose_corrected_with_cov_pub;
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>> pose_scanend_corrected_with_cov_pub;

  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>> lidar_pose_pub;  // Pose in LiDAR frame
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>> lidar_pose_scanend_pub;
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>> lidar_pose_corrected_pub;
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>> lidar_pose_scanend_corrected_pub;
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>> lidar_pose_corrected_with_cov_pub;
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>> lidar_pose_scanend_corrected_with_cov_pub;

  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::TwistStamped>> imu_bias_pub;
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>> imu_bias_with_cov_pub;

  std::mutex trajectory_mutex;
  std::unique_ptr<TrajectoryManager> trajectory;

  // Map data
  std::thread map_thread;
  std::mutex map_task_mutex;
  std::condition_variable map_task_cv;
  std::mutex map_mutex;
  std::vector<SubmapCache> submap_caches;
  std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> submap_poses;
  std::atomic_bool map_dirty{false};
  std::atomic_bool grid_dirty{false};
  size_t last_map_subs;
  size_t last_grid_subs;
  rclcpp::Time last_grid_pub_time;

  double map_voxel_size;      // [m] map downsampling
  std::atomic<double> map_publish_period;  // [s] min period between map/grid publications
  double grid_resolution;     // [m/cell]
  std::atomic<double> grid_z_min;          // [m] obstacle band, height above the local ground
  std::atomic<double> grid_z_max;
  std::atomic<double> grid_ground_cell;    // [m] size of the columns used to estimate the local ground height
  std::atomic_int grid_min_points;        // points in the band for a cell to be occupied
  std::atomic<double> grid_max_range;      // [m] max ray length for free-space carving

  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>> grid_pub;

  // Logging
  std::shared_ptr<spdlog::logger> logger;
};
}  // namespace glim
