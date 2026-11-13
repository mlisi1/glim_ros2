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
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <glim/odometry/estimation_frame.hpp>
#include <glim/mapping/sub_map.hpp>
#include <glim/util/extension_module.hpp>
#include <glim/util/extension_module_ros2.hpp>

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
  void on_insert_imu(double stamp, const Eigen::Vector3d& angular_vel);
  Eigen::Vector3d mean_gyro(double t0, double t1);
  void globalmap_on_update_submaps(const std::vector<SubMap::Ptr>& submaps);
  void invoke(const std::function<void()>& task);

  void worker_loop();
  void resolve_frame_ids();
  std::optional<Eigen::Isometry3d> lookup_T_imu_base();
  void publish_frame(const FrameTask& task);
  void publish_map_odom_periodic();

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
  bool frame_ids_resolved;
  double tf_time_offset;

  // TF switches (ROS parameters on the glim_ros node)
  bool publish_tf_map_odom;
  bool publish_tf_odom_base;
  bool publish_tf_imu_lidar;

  // map->odom computation
  //  internal: relative to GLIM's own odometry frame (GLIM also owns odom->base)
  //  external: relative to an odom->base transform published by someone else (e.g. robot_localization)
  bool map_odom_external;
  bool map_odom_planar;          // keep only x, y, yaw of map->odom
  double map_odom_rate;          // [Hz] republish rate in external mode
  double map_odom_tolerance;     // [s] future-dating of republished map->odom (as AMCL's transform_tolerance)
  double external_odom_timeout;  // [s] max wait for the external odom->base at the scan stamp
  std::optional<Eigen::Isometry3d> latest_T_map_odom;
  rclcpp::Time last_map_odom_pub;

  // Odometry/pose output frame
  BodyFrame odom_child_frame;
  double twist_angular_cov;     // variance [rad^2/s^2] reported for angular velocity
  double twist_angular_window;  // gyro averaging window [s] centered on the frame stamp

  // Recent gyro samples [t, wx, wy, wz] (angular velocity source for the odometry twist)
  std::mutex imu_mutex;
  std::deque<Eigen::Vector4d> gyro_buffer;
  std::optional<Eigen::Isometry3d> T_imu_base;  // cached static transform

  // Publishers: nullptr if disabled through the 'outputs' parameter
  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odometry_local_pub;   // ~/odometry/local
  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odometry_global_pub;  // ~/odometry/global

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

  std::vector<gtsam_points::PointCloud::ConstPtr> submaps;

  // Logging
  std::shared_ptr<spdlog::logger> logger;
};
}  // namespace glim
