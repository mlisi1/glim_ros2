#include <glim_ros/rviz_viewer.hpp>

#include <mutex>
#include <future>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <filesystem>
#include <unordered_map>
#include <unordered_set>
#include <set>
#include <spdlog/spdlog.h>
#include <rclcpp/clock.hpp>

#define GLIM_ROS2
#include <gtsam/geometry/Pose3.h>
#include <gtsam_points/types/point_cloud_cpu.hpp>
#include <glim/odometry/callbacks.hpp>
#include <glim/mapping/callbacks.hpp>
#include <glim/util/logging.hpp>
#include <glim/util/config.hpp>
#include <glim/util/trajectory_manager.hpp>
#include <glim/util/ros_cloud_converter.hpp>

namespace glim {

namespace {

using RowMajor6 = Eigen::Matrix<double, 6, 6, Eigen::RowMajor>;

// GTSAM's Pose3 covariance is ordered [rx, ry, rz, x, y, z]; ROS expects [x, y, z, rx, ry, rz]
RowMajor6 reorder_cov(const gtsam::Matrix6& C) {
  RowMajor6 cov;
  cov.block<3, 3>(0, 0) = C.block<3, 3>(3, 3);
  cov.block<3, 3>(0, 3) = C.block<3, 3>(3, 0);
  cov.block<3, 3>(3, 0) = C.block<3, 3>(0, 3);
  cov.block<3, 3>(3, 3) = C.block<3, 3>(0, 0);
  return cov;
}

geometry_msgs::msg::Pose to_pose_msg(const Eigen::Isometry3d& T) {
  const Eigen::Quaterniond q(T.linear());
  geometry_msgs::msg::Pose pose;
  pose.position.x = T.translation().x();
  pose.position.y = T.translation().y();
  pose.position.z = T.translation().z();
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();
  return pose;
}

geometry_msgs::msg::Transform to_transform_msg(const Eigen::Isometry3d& T) {
  const auto pose = to_pose_msg(T);
  geometry_msgs::msg::Transform trans;
  trans.translation.x = pose.position.x;
  trans.translation.y = pose.position.y;
  trans.translation.z = pose.position.z;
  trans.rotation = pose.orientation;
  return trans;
}

/**
 * @brief State of a body frame X rigidly attached to the IMU, expressed as ROS expects it:
 *        pose of X in the parent frame, twist of X expressed in X (nav_msgs/Odometry convention).
 */
struct BodyState {
  Eigen::Isometry3d T_parent_X;
  Eigen::Vector3d v_X;  // linear velocity of X's origin, in X
  Eigen::Vector3d w_X;  // angular velocity, in X
  RowMajor6 pose_cov;   // [x, y, z, rx, ry, rz], right-perturbation (body) frame of X
  RowMajor6 twist_cov;  // [vx, vy, vz, wx, wy, wz] in X
};

/**
 * @param T_parent_imu   IMU pose in the parent frame
 * @param v_parent_imu   IMU velocity expressed in the parent frame (as estimated by GLIM)
 * @param w_imu          IMU angular velocity expressed in the IMU frame
 * @param C_pose_imu     IMU pose covariance (GTSAM order), or nullptr
 * @param C_vel_parent   IMU velocity covariance in the parent frame, or nullptr
 * @param T_imu_X        Pose of X in the IMU frame
 */
BodyState make_body_state(
  const Eigen::Isometry3d& T_parent_imu,
  const Eigen::Vector3d& v_parent_imu,
  const Eigen::Vector3d& w_imu,
  const gtsam::Matrix6* C_pose_imu,
  const Eigen::Matrix3d* C_vel_parent,
  double angular_var,
  const Eigen::Isometry3d& T_imu_X) {
  BodyState s;
  const Eigen::Matrix3d R_parent_imu = T_parent_imu.linear();
  const Eigen::Matrix3d R_imu_X = T_imu_X.linear();
  const Eigen::Vector3d t_imu_X = T_imu_X.translation();

  s.T_parent_X = T_parent_imu * T_imu_X;

  const Eigen::Vector3d v_imu = R_parent_imu.transpose() * v_parent_imu;  // IMU velocity in IMU frame
  s.v_X = R_imu_X.transpose() * (v_imu + w_imu.cross(t_imu_X));            // lever arm
  s.w_X = R_imu_X.transpose() * w_imu;

  s.pose_cov.setZero();
  if (C_pose_imu) {
    const gtsam::Matrix6 Ad_T_X_imu = gtsam::Pose3(T_imu_X.inverse().matrix()).AdjointMap();
    // GTSAM's covariance is in the body (right-perturbation) frame of X; ROS expects it about the parent's fixed axes
    RowMajor6 R6 = RowMajor6::Zero();
    R6.block<3, 3>(0, 0) = s.T_parent_X.linear();
    R6.block<3, 3>(3, 3) = s.T_parent_X.linear();
    s.pose_cov = R6 * reorder_cov(Ad_T_X_imu * (*C_pose_imu) * Ad_T_X_imu.transpose()) * R6.transpose();
  }

  s.twist_cov.setZero();
  if (C_vel_parent) {
    const Eigen::Matrix3d R_X_parent = (R_parent_imu * R_imu_X).transpose();
    s.twist_cov.block<3, 3>(0, 0) = R_X_parent * (*C_vel_parent) * R_X_parent.transpose();
  }
  s.twist_cov.block<3, 3>(3, 3) = angular_var * Eigen::Matrix3d::Identity();

  return s;
}

nav_msgs::msg::Odometry to_odom_msg(const BodyState& s, const builtin_interfaces::msg::Time& stamp, const std::string& parent, const std::string& child) {
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = stamp;
  odom.header.frame_id = parent;
  odom.child_frame_id = child;
  odom.pose.pose = to_pose_msg(s.T_parent_X);
  odom.twist.twist.linear.x = s.v_X.x();
  odom.twist.twist.linear.y = s.v_X.y();
  odom.twist.twist.linear.z = s.v_X.z();
  odom.twist.twist.angular.x = s.w_X.x();
  odom.twist.twist.angular.y = s.w_X.y();
  odom.twist.twist.angular.z = s.w_X.z();
  std::copy(s.pose_cov.data(), s.pose_cov.data() + 36, odom.pose.covariance.begin());
  std::copy(s.twist_cov.data(), s.twist_cov.data() + 36, odom.twist.covariance.begin());
  return odom;
}

geometry_msgs::msg::PoseStamped to_pose_stamped(const BodyState& s, const builtin_interfaces::msg::Time& stamp, const std::string& parent) {
  geometry_msgs::msg::PoseStamped pose;
  pose.header.stamp = stamp;
  pose.header.frame_id = parent;
  pose.pose = to_pose_msg(s.T_parent_X);
  return pose;
}

geometry_msgs::msg::PoseWithCovarianceStamped to_pose_with_cov(const BodyState& s, const builtin_interfaces::msg::Time& stamp, const std::string& parent) {
  geometry_msgs::msg::PoseWithCovarianceStamped pose;
  pose.header.stamp = stamp;
  pose.header.frame_id = parent;
  pose.pose.pose = to_pose_msg(s.T_parent_X);
  std::copy(s.pose_cov.data(), s.pose_cov.data() + 36, pose.pose.covariance.begin());
  return pose;
}

template <typename PubT>
bool has_subs(const std::shared_ptr<PubT>& pub) {
  return pub && pub->get_subscription_count() > 0;
}

// Keep only x, y, yaw
Eigen::Isometry3d flatten(const Eigen::Isometry3d& T) {
  const Eigen::Matrix3d R = T.linear();
  const double yaw = std::atan2(R(1, 0), R(0, 0));
  Eigen::Isometry3d F = Eigen::Isometry3d::Identity();
  F.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  F.translation() << T.translation().x(), T.translation().y(), 0.0;
  return F;
}

Eigen::Isometry3d imu_traj_pose(const Eigen::Matrix<double, 8, 1>& x) {
  // [t, x, y, z, qx, qy, qz, qw]
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translation() = x.segment<3>(1);
  T.linear() = Eigen::Quaterniond(x(7), x(4), x(5), x(6)).normalized().toRotationMatrix();
  return T;
}

}  // namespace

RvizViewer::RvizViewer() : logger(create_module_logger("rviz")) {
  const Config config(GlobalConfig::get_config_path("config_ros"));

  imu_frame_id = config.param<std::string>("glim_ros", "imu_frame_id", "");
  lidar_frame_id = config.param<std::string>("glim_ros", "lidar_frame_id", "");
  base_frame_id = config.param<std::string>("glim_ros", "base_frame_id", "");
  odom_frame_id = config.param<std::string>("glim_ros", "odom_frame_id", "odom");
  map_frame_id = config.param<std::string>("glim_ros", "map_frame_id", "map");
  tf_time_offset = config.param<double>("glim_ros", "tf_time_offset", 1e-6);
  frame_ids_resolved = false;

  // Defaults; overridden by ROS parameters in create_subscriptions()
  publish_tf_map_odom = true;
  publish_tf_odom_base = true;
  publish_tf_imu_lidar = config.param<bool>("glim_ros", "publish_imu2lidar", true);
  odom_child_frame = BodyFrame::IMU;
  twist_angular_cov = 1e-2;
  twist_angular_window = 0.1;
  fallback_position_var = 1e-2;
  fallback_rotation_var = 1e-3;
  fallback_velocity_var = 1e-2;
  max_frame_queue = 64;
  map_odom_external = false;
  map_odom_planar = false;
  map_odom_rate = 20.0;
  map_odom_tolerance = 0.1;
  external_odom_timeout = 0.1;

  map_voxel_size = 0.2;
  map_publish_period = 10.0;
  grid_resolution = 0.1;
  grid_z_min = 0.3;
  grid_z_max = 1.5;
  grid_min_points = 3;
  grid_ground_cell = 1.0;
  grid_max_range = 30.0;
  map_dirty = grid_dirty = false;
  last_map_subs = last_grid_subs = 0;

  trajectory.reset(new TrajectoryManager);

  set_callbacks();

  initialized = false;
  kill_switch = false;
  thread = std::thread([this] { worker_loop(); });
  map_thread = std::thread([this] { map_loop(); });
}

RvizViewer::~RvizViewer() {
  kill_switch = true;
  queue_cv.notify_all();
  map_task_cv.notify_all();
  thread.join();
  map_thread.join();
}

std::vector<GenericTopicSubscription::Ptr> RvizViewer::create_subscriptions(rclcpp::Node& node) {
  clock = node.get_clock();
  tf_buffer = std::make_unique<tf2_ros::Buffer>(clock);
  tf_listener = std::make_unique<tf2_ros::TransformListener>(*tf_buffer);
  tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(node);

  // --- ROS parameters (declared on the glim_ros node, so they can be set from launch files) ---
  // Frame IDs: config_ros.json values are the defaults
  imu_frame_id = node.declare_parameter<std::string>("imu_frame_id", imu_frame_id);
  lidar_frame_id = node.declare_parameter<std::string>("lidar_frame_id", lidar_frame_id);
  base_frame_id = node.declare_parameter<std::string>("base_frame_id", base_frame_id);
  odom_frame_id = node.declare_parameter<std::string>("odom_frame_id", odom_frame_id);
  map_frame_id = node.declare_parameter<std::string>("map_frame_id", map_frame_id);
  tf_time_offset = node.declare_parameter<double>("tf_time_offset", tf_time_offset);

  // publish_tf is a master switch; the individual switches default to it.
  const bool publish_tf = node.declare_parameter<bool>("publish_tf", true);
  publish_tf_map_odom = node.declare_parameter<bool>("publish_tf_map_odom", publish_tf);
  publish_tf_odom_base = node.declare_parameter<bool>("publish_tf_odom_base", publish_tf);
  publish_tf_imu_lidar = node.declare_parameter<bool>("publish_tf_imu_lidar", publish_tf && publish_tf_imu_lidar);

  const std::string child = node.declare_parameter<std::string>("odom_child_frame", "imu");
  if (child == "imu") {
    odom_child_frame = BodyFrame::IMU;
  } else if (child == "lidar") {
    odom_child_frame = BodyFrame::LIDAR;
  } else if (child == "base") {
    odom_child_frame = BodyFrame::BASE;
  } else {
    logger->error("unknown odom_child_frame '{}' (expected imu|lidar|base). using imu", child);
    odom_child_frame = BodyFrame::IMU;
  }

  twist_angular_cov = node.declare_parameter<double>("twist_angular_cov", twist_angular_cov);
  twist_angular_window = node.declare_parameter<double>("twist_angular_window", twist_angular_window);
  fallback_position_var = node.declare_parameter<double>("fallback_position_var", fallback_position_var);
  fallback_rotation_var = node.declare_parameter<double>("fallback_rotation_var", fallback_rotation_var);
  fallback_velocity_var = node.declare_parameter<double>("fallback_velocity_var", fallback_velocity_var);
  max_frame_queue = std::max<int64_t>(1, node.declare_parameter<int64_t>("max_publish_queue", static_cast<int64_t>(max_frame_queue)));

  const std::string map_odom_mode = node.declare_parameter<std::string>("map_odom_mode", "internal");
  if (map_odom_mode != "internal" && map_odom_mode != "external") {
    logger->error("unknown map_odom_mode '{}' (expected internal|external). using internal", map_odom_mode);
  }
  map_odom_external = map_odom_mode == "external";
  map_odom_planar = node.declare_parameter<bool>("map_odom_planar", map_odom_planar);
  map_odom_rate = node.declare_parameter<double>("map_odom_rate", map_odom_rate);
  map_odom_tolerance = node.declare_parameter<double>("map_odom_tolerance", map_odom_tolerance);
  external_odom_timeout = node.declare_parameter<double>("external_odom_timeout", external_odom_timeout);
  if (map_odom_external && publish_tf_odom_base) {
    logger->warn("map_odom_mode=external: odom->base is owned by the external odometry; disabling publish_tf_odom_base");
    publish_tf_odom_base = false;
  }

  const std::string anchor_frame = node.declare_parameter<std::string>("anchor_frame", "imu");
  if (anchor_frame != "imu" && anchor_frame != "base") {
    logger->error("unknown anchor_frame '{}' (expected imu|base). using imu", anchor_frame);
  }
  anchor_base = anchor_frame == "base";
  if (!anchor_base) {
    std::lock_guard<std::mutex> lock(anchor_mutex);
    T_map_world = Eigen::Isometry3d::Identity();
  }

  logger->info(
    "TF: map->odom={} ({}{}) odom->base={} imu->lidar={} | odom_child_frame={} anchor_frame={}",
    publish_tf_map_odom.load(),
    map_odom_mode,
    map_odom_planar ? ", planar" : "",
    publish_tf_odom_base.load(),
    publish_tf_imu_lidar.load(),
    child,
    anchor_base ? "base" : "imu");

  map_voxel_size = node.declare_parameter<double>("map_voxel_size", map_voxel_size);
  map_publish_period = node.declare_parameter<double>("map_publish_period", map_publish_period);
  grid_resolution = node.declare_parameter<double>("grid_resolution", grid_resolution);
  grid_z_min = node.declare_parameter<double>("grid_z_min", grid_z_min);
  grid_z_max = node.declare_parameter<double>("grid_z_max", grid_z_max);
  grid_min_points = node.declare_parameter<int>("grid_min_points", grid_min_points);
  grid_max_range = node.declare_parameter<double>("grid_max_range", grid_max_range);
  grid_ground_cell = node.declare_parameter<double>("grid_ground_cell", grid_ground_cell);

  // --- Outputs ---
  const auto outputs_param = node.declare_parameter<std::vector<std::string>>("outputs", std::vector<std::string>{"all"});
  const std::set<std::string> outputs(outputs_param.begin(), outputs_param.end());
  std::set<std::string> known;
  const auto create = [&](auto& pub, const std::string& name, const rclcpp::QoS& qos) {
    using MsgT = typename std::decay_t<decltype(*pub)>::ROSMessageType;
    known.insert(name);
    if (outputs.count("all") || outputs.count(name)) {
      pub = node.create_publisher<MsgT>("~/" + name, qos);
    } else {
      pub = nullptr;
    }
  };

  rmw_qos_profile_t map_qos_profile = {
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    1,
    RMW_QOS_POLICY_RELIABILITY_RELIABLE,
    RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
    RMW_QOS_DEADLINE_DEFAULT,
    RMW_QOS_LIFESPAN_DEFAULT,
    RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
    RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
    false};
  const rclcpp::QoS map_qos(rclcpp::QoSInitialization(map_qos_profile.history, map_qos_profile.depth), map_qos_profile);
  const rclcpp::QoS qos(10);

  create(odometry_local_pub, "odometry/local", qos);
  create(odometry_global_pub, "odometry/global", qos);
  create(odometry_imu_rate_pub, "odometry/imu_rate", rclcpp::QoS(50));

  create(map_pub, "map", map_qos);
  create(grid_pub, "grid", map_qos);

  create(points_pub, "points", qos);
  create(points_corrected_pub, "points_corrected", qos);
  create(aligned_points_pub, "aligned_points", qos);
  create(aligned_points_corrected_pub, "aligned_points_corrected", qos);

  create(odom_pub, "odom", qos);
  create(odom_scanend_pub, "odom_scanend", qos);
  create(odom_corrected_pub, "odom_corrected", qos);
  create(odom_scanend_corrected_pub, "odom_scanend_corrected", qos);

  create(lidar_odom_pub, "lidar_odom", qos);
  create(lidar_odom_scanend_pub, "lidar_odom_scanend", qos);
  create(lidar_odom_corrected_pub, "lidar_odom_corrected", qos);
  create(lidar_odom_scanend_corrected_pub, "lidar_odom_scanend_corrected", qos);

  create(pose_pub, "pose", qos);
  create(pose_scanend_pub, "pose_scanend", qos);
  create(pose_corrected_pub, "pose_corrected", qos);
  create(pose_scanend_corrected_pub, "pose_scanend_corrected", qos);
  create(pose_corrected_with_cov_pub, "pose_corrected_with_cov", qos);
  create(pose_scanend_corrected_with_cov_pub, "pose_scanend_corrected_with_cov", qos);

  create(lidar_pose_pub, "lidar_pose", qos);
  create(lidar_pose_scanend_pub, "lidar_pose_scanend", qos);
  create(lidar_pose_corrected_pub, "lidar_pose_corrected", qos);
  create(lidar_pose_scanend_corrected_pub, "lidar_pose_scanend_corrected", qos);
  create(lidar_pose_corrected_with_cov_pub, "lidar_pose_corrected_with_cov", qos);
  create(lidar_pose_scanend_corrected_with_cov_pub, "lidar_pose_scanend_corrected_with_cov", qos);

  create(imu_bias_pub, "imu_bias", qos);
  create(imu_bias_with_cov_pub, "imu_bias_with_cov", qos);

  for (const auto& name : outputs) {
    if (name != "all" && !known.count(name)) {
      logger->warn("unknown output '{}' in 'outputs' parameter", name);
    }
  }

  last_globalmap_pub_time = clock->now();
  last_map_odom_pub = clock->now();
  last_grid_pub_time = clock->now();

  // Parameters that can be changed at runtime (ros2 param set)
  param_callback = node.add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter>& params) { return on_set_parameters(params); });

  initialized = true;
  queue_cv.notify_all();

  return {};
}

void RvizViewer::set_callbacks() {
  using std::placeholders::_1;
  OdometryEstimationCallbacks::on_insert_imu.add(
    [this](const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel) { on_insert_imu(stamp, linear_acc, angular_vel); });
  OdometryEstimationCallbacks::on_new_frame.add([this](const EstimationFrame::ConstPtr& new_frame) { on_new_frame(new_frame, false); });
  OdometryEstimationCallbacks::on_update_new_frame.add([this](const EstimationFrame::ConstPtr& new_frame) { on_new_frame(new_frame, true); });
  GlobalMappingCallbacks::on_update_submaps.add(std::bind(&RvizViewer::globalmap_on_update_submaps, this, _1));
}

bool RvizViewer::should_warn(std::chrono::steady_clock::time_point& last, double period_sec) {
  const auto now = std::chrono::steady_clock::now();
  if (now - last < std::chrono::duration<double>(period_sec)) {
    return false;
  }
  last = now;
  return true;
}

void RvizViewer::on_insert_imu(double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel) {
  {
    std::lock_guard<std::mutex> lock(imu_mutex);
    ImuSample sample;
    sample << stamp, linear_acc, angular_vel;
    imu_buffer.push_back(sample);
    while (!imu_buffer.empty() && imu_buffer.front()[0] < stamp - 5.0) {
      imu_buffer.pop_front();
    }
  }
  if (imu_rate_active) {
    {
      std::lock_guard<std::mutex> lock(queue_mutex);
      new_imu = true;
    }
    queue_cv.notify_one();
  }
}

// Mean gyro reading in [t0, t1]; falls back to the sample closest to the window center
Eigen::Vector3d RvizViewer::mean_gyro(double t0, double t1) {
  std::lock_guard<std::mutex> lock(imu_mutex);
  if (imu_buffer.empty()) {
    return Eigen::Vector3d::Zero();
  }

  Eigen::Vector3d sum = Eigen::Vector3d::Zero();
  int n = 0;
  const ImuSample* closest = &imu_buffer.front();
  const double tc = 0.5 * (t0 + t1);
  for (const auto& g : imu_buffer) {
    if (g[0] >= t0 && g[0] <= t1) {
      sum += g.tail<3>();
      n++;
    }
    if (std::abs(g[0] - tc) < std::abs((*closest)[0] - tc)) {
      closest = &g;
    }
  }
  return n ? Eigen::Vector3d(sum / n) : Eigen::Vector3d(closest->tail<3>());
}

// Called from the odometry estimation thread: keep it cheap.
void RvizViewer::on_new_frame(const EstimationFrame::ConstPtr& new_frame, bool corrected) {
  FrameTask task;
  task.frame = new_frame;
  task.corrected = corrected;

  {
    // Keep the odom->world bookkeeping in the estimation thread so that it stays ordered w.r.t. anchor updates
    std::lock_guard<std::mutex> lock(trajectory_mutex);
    trajectory->add_odom(new_frame->stamp, new_frame->T_world_imu, 1);
    task.T_world_odom = trajectory->get_T_world_odom();
  }

  {
    std::lock_guard<std::mutex> lock(queue_mutex);
    frame_queue.push_back(std::move(task));
    if (frame_queue.size() > max_frame_queue) {
      frame_queue.pop_front();
      if (should_warn(last_drop_warn, 5.0)) {
        logger->warn("ROS publishing is falling behind estimation; dropping oldest frames (max_publish_queue={})", max_frame_queue);
      }
    }
  }
  queue_cv.notify_one();
}

void RvizViewer::worker_loop() {
  while (!kill_switch) {
    std::deque<FrameTask> frames;
    {
      std::unique_lock<std::mutex> lock(queue_mutex);
      const double wait_sec = map_odom_external ? std::min(0.1, 0.5 / std::max(map_odom_rate.load(), 1.0)) : 0.1;
      queue_cv.wait_for(lock, std::chrono::duration<double>(wait_sec), [this] {
        return kill_switch || (initialized && (!frame_queue.empty() || new_imu));
      });
      if (kill_switch) {
        break;
      }
      if (!initialized) {
        continue;
      }
      frames.swap(frame_queue);
      new_imu = false;
    }

    for (const auto& frame : frames) {
      publish_frame(frame);
    }
    publish_map_odom_periodic();

    imu_rate_active = has_subs(odometry_imu_rate_pub);
    publish_imu_rate();
  }
}

// External mode: republish the latest map->odom at a fixed rate, future-dated so that lookups at "now" succeed
void RvizViewer::publish_map_odom_periodic() {
  if (!map_odom_external || !publish_tf_map_odom || !latest_T_map_odom) {
    return;
  }
  const rclcpp::Time now = clock->now();
  if ((now - last_map_odom_pub).seconds() < 1.0 / std::max(map_odom_rate.load(), 1e-3)) {
    return;
  }
  last_map_odom_pub = now;

  geometry_msgs::msg::TransformStamped trans;
  trans.header.stamp = now + rclcpp::Duration::from_seconds(map_odom_tolerance);
  trans.header.frame_id = map_frame_id;
  trans.child_frame_id = odom_frame_id;
  trans.transform = to_transform_msg(*latest_T_map_odom);
  tf_broadcaster->sendTransform(trans);
}

void RvizViewer::resolve_frame_ids() {
  if (frame_ids_resolved) {
    return;
  }

  if (imu_frame_id.empty()) {
    imu_frame_id = GlobalConfig::instance()->param<std::string>("meta", "imu_frame_id", "");
    if (imu_frame_id.empty()) {
      logger->warn("IMU frame ID is not set. Using 'imu' as default.");
      imu_frame_id = "imu";
    } else {
      logger->info("auto-detected IMU frame ID: {}", imu_frame_id);
    }
  }

  if (lidar_frame_id.empty()) {
    lidar_frame_id = GlobalConfig::instance()->param<std::string>("meta", "lidar_frame_id", "");
    if (lidar_frame_id.empty()) {
      logger->warn("LiDAR frame ID is not set. Using 'lidar' as default.");
      lidar_frame_id = "lidar";
    } else {
      logger->info("auto-detected LiDAR frame ID: {}", lidar_frame_id);
    }
  }

  if (base_frame_id.empty()) {
    base_frame_id = imu_frame_id;
    logger->info("base_frame_id is not set. using IMU frame ID '{}' as base frame ID.", imu_frame_id);
  }

  frame_ids_resolved = true;
}

std::optional<Eigen::Isometry3d> RvizViewer::lookup_T_imu_base() {
  if (!frame_ids_resolved) {
    return std::nullopt;
  }
  if (base_frame_id == imu_frame_id) {
    return Eigen::Isometry3d::Identity();
  }
  std::lock_guard<std::mutex> lock(T_imu_base_mutex);
  if (T_imu_base) {
    return T_imu_base;
  }

  // imu -> base is assumed to be static (URDF), so it is looked up once and cached
  try {
    const auto trans = tf_buffer->lookupTransform(imu_frame_id, base_frame_id, tf2::TimePointZero);
    const auto& t = trans.transform.translation;
    const auto& r = trans.transform.rotation;
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.translation() << t.x, t.y, t.z;
    T.linear() = Eigen::Quaterniond(r.w, r.x, r.y, r.z).normalized().toRotationMatrix();
    T_imu_base = T;
    logger->info("cached static transform {} -> {}", imu_frame_id, base_frame_id);
  } catch (const tf2::TransformException& e) {
    static std::chrono::steady_clock::time_point last_warn;
    if (should_warn(last_warn, 5.0)) {
      logger->warn("Failed to lookup transform from {} to {}: {}", imu_frame_id, base_frame_id, e.what());
    }
  }
  return T_imu_base;
}

std::optional<Eigen::Isometry3d> RvizViewer::get_anchor() {
  std::lock_guard<std::mutex> lock(anchor_mutex);
  return T_map_world;
}

// anchor_frame=base: the published map/odom frames start at the base pose of the first frame, reduced to x, y, z and
// yaw. GLIM's world frame is gravity aligned and rotating only about z keeps it so (occupancy grid heights, gravity in
// the IMU-rate propagation). The first frame is recorded even if imu->base is not known yet, so a late TF does not move
// the origin; nothing is published until it is known.
std::optional<Eigen::Isometry3d> RvizViewer::resolve_anchor(const Eigen::Isometry3d& T_world_imu) {
  if (auto anchor = get_anchor()) {
    return anchor;
  }
  if (!first_T_world_imu) {
    first_T_world_imu = T_world_imu;
  }
  const auto T_imu_base = lookup_T_imu_base();
  if (!T_imu_base) {
    return std::nullopt;
  }

  const Eigen::Isometry3d T_world_base0 = (*first_T_world_imu) * (*T_imu_base);
  const Eigen::Matrix3d R = T_world_base0.linear();
  Eigen::Isometry3d T_world_map = Eigen::Isometry3d::Identity();
  T_world_map.linear() = Eigen::AngleAxisd(std::atan2(R(1, 0), R(0, 0)), Eigen::Vector3d::UnitZ()).toRotationMatrix();
  T_world_map.translation() = T_world_base0.translation();

  std::lock_guard<std::mutex> lock(anchor_mutex);
  T_map_world = T_world_map.inverse();
  const Eigen::Vector3d t = T_world_map.translation();
  logger->info(
    "anchor_frame=base: {}/{} origin at the initial {} pose (GLIM world x={:.3f} y={:.3f} z={:.3f} yaw={:.3f})",
    map_frame_id,
    odom_frame_id,
    base_frame_id,
    t.x(),
    t.y(),
    t.z(),
    std::atan2(R(1, 0), R(0, 0)));
  return T_map_world;
}

void RvizViewer::publish_frame(const FrameTask& task) {
  const auto& new_frame = task.frame;
  const bool corrected = task.corrected;

  resolve_frame_ids();

  // Published frames = GLIM's frames re-anchored by A = T_map_world (identity for anchor_frame=imu):
  //  odom->imu = A * T_odom_imu(GLIM), map->odom = A * T_world_odom(GLIM) * A^-1, hence map->imu = A * T_world_imu(GLIM)
  const auto anchor = resolve_anchor(task.T_world_odom * new_frame->T_world_imu);
  if (!anchor) {
    static std::chrono::steady_clock::time_point last_warn;
    if (should_warn(last_warn, 5.0)) {
      logger->warn("anchor_frame=base: waiting for {} -> {} before publishing", imu_frame_id, base_frame_id);
    }
    return;
  }
  const Eigen::Isometry3d& A = *anchor;
  const Eigen::Matrix3d R_A = A.linear();

  const Eigen::Isometry3d T_odom_imu = A * new_frame->T_world_imu;
  const Eigen::Vector3d v_odom_imu = R_A * new_frame->v_world_imu;
  const Eigen::Isometry3d T_lidar_imu = new_frame->T_lidar_imu;
  const Eigen::Isometry3d T_imu_lidar = T_lidar_imu.inverse();

  // "world" below is the published map frame
  const Eigen::Isometry3d T_world_odom = A * task.T_world_odom * A.inverse();
  const Eigen::Isometry3d T_world_imu = T_world_odom * T_odom_imu;

  // Covariances (only available for optimized frames when enabled in GLIM)
  // Covariances: GLIM's when computed (compute_covs), otherwise configurable fallbacks (never all-zero)
  gtsam::Matrix6 C_pose_fallback = gtsam::Matrix6::Zero();
  C_pose_fallback.diagonal() << Eigen::Vector3d::Constant(fallback_rotation_var), Eigen::Vector3d::Constant(fallback_position_var);
  const gtsam::Matrix6* C_pose = new_frame->cov_computed ? &new_frame->pose_cov : &C_pose_fallback;
  const Eigen::Matrix3d C_vel_glim = new_frame->cov_computed ? Eigen::Matrix3d(new_frame->vel_cov) : Eigen::Matrix3d(fallback_velocity_var * Eigen::Matrix3d::Identity());
  const Eigen::Matrix3d C_vel_odom = R_A * C_vel_glim * R_A.transpose();
  const Eigen::Matrix3d* C_vel_odom_ptr = &C_vel_odom;

  if (corrected) {
    latest_state = ImuState{new_frame->stamp, T_odom_imu, v_odom_imu, new_frame->imu_bias, T_lidar_imu};
  }

  // Angular velocity: bias-corrected gyro averaged around the frame stamp (IMU frame)
  const Eigen::Vector3d gyro_bias = new_frame->imu_bias.tail<3>();
  const Eigen::Vector3d w_imu = mean_gyro(new_frame->stamp - 0.5 * twist_angular_window, new_frame->stamp + 0.5 * twist_angular_window) - gyro_bias;

  // Motion during the scan (IMU-rate trajectory, if enabled in the odometry config): scan-end pose
  double imu_end_time = new_frame->stamp;
  Eigen::Isometry3d T_imubegin_imuend = Eigen::Isometry3d::Identity();
  if (new_frame->imu_rate_trajectory.cols() >= 2) {
    const Eigen::Matrix<double, 8, 1> imu_begin = new_frame->imu_rate_trajectory.col(0);
    const Eigen::Matrix<double, 8, 1> imu_end = new_frame->imu_rate_trajectory.col(new_frame->imu_rate_trajectory.cols() - 1);
    T_imubegin_imuend = imu_traj_pose(imu_begin).inverse() * imu_traj_pose(imu_end);
    imu_end_time = imu_end(0);
  }

  // Body frame used for odom*/pose* topics
  std::optional<Eigen::Isometry3d> T_imu_child;
  std::string child_frame_id;
  switch (odom_child_frame) {
    case BodyFrame::IMU:
      T_imu_child = Eigen::Isometry3d::Identity();
      child_frame_id = imu_frame_id;
      break;
    case BodyFrame::LIDAR:
      T_imu_child = T_imu_lidar;
      child_frame_id = lidar_frame_id;
      break;
    case BodyFrame::BASE:
      T_imu_child = lookup_T_imu_base();
      child_frame_id = base_frame_id;
      break;
  }
  if (!T_imu_child) {
    static std::chrono::steady_clock::time_point last_warn;
    if (should_warn(last_warn, 5.0)) {
      logger->warn("{} -> {} unknown; odom/pose topics in the base frame are not published", imu_frame_id, base_frame_id);
    }
  }

  const auto stamp = from_sec(new_frame->stamp);
  const auto tf_stamp = from_sec(new_frame->stamp + tf_time_offset);
  const auto imu_end_stamp = from_sec(imu_end_time);

  // --- TF: published once per scan, from the optimized estimate ---
  if (corrected) {
    geometry_msgs::msg::TransformStamped trans;
    trans.header.stamp = tf_stamp;

    if (publish_tf_odom_base) {
      const auto T_imu_base = lookup_T_imu_base();
      if (T_imu_base) {
        trans.header.frame_id = odom_frame_id;
        trans.child_frame_id = base_frame_id;
        trans.transform = to_transform_msg(T_odom_imu * (*T_imu_base));
        tf_broadcaster->sendTransform(trans);
      }
    }

    if (publish_tf_map_odom && !map_odom_external) {
      trans.header.frame_id = map_frame_id;
      trans.child_frame_id = odom_frame_id;
      trans.transform = to_transform_msg(map_odom_planar ? flatten(T_world_odom) : T_world_odom);
      tf_broadcaster->sendTransform(trans);
    }

    if (map_odom_external) {
      // T_map_odom = T_map_base (GLIM, with loop closure) * T_odom_base(external, same stamp)^-1
      const auto T_imu_base = lookup_T_imu_base();
      if (T_imu_base) {
        try {
          const auto ext = tf_buffer->lookupTransform(odom_frame_id, base_frame_id, from_sec(new_frame->stamp), rclcpp::Duration::from_seconds(external_odom_timeout));
          const auto& t = ext.transform.translation;
          const auto& r = ext.transform.rotation;
          Eigen::Isometry3d T_odom_base = Eigen::Isometry3d::Identity();
          T_odom_base.translation() << t.x, t.y, t.z;
          T_odom_base.linear() = Eigen::Quaterniond(r.w, r.x, r.y, r.z).normalized().toRotationMatrix();

          const Eigen::Isometry3d T_map_base = T_world_imu * (*T_imu_base);
          if (map_odom_planar) {
            // Flatten both poses before composing: flattening the composed 3D transform would rotate the external
            // odom->base translation by GLIM's residual roll/pitch (meters of error far from the odom origin)
            latest_T_map_odom = flatten(T_map_base) * flatten(T_odom_base).inverse();
          } else {
            latest_T_map_odom = T_map_base * T_odom_base.inverse();
          }
          last_map_odom_pub = rclcpp::Time(0, 0, clock->get_clock_type());  // publish right away
        } catch (const tf2::TransformException& e) {
          static std::chrono::steady_clock::time_point last_warn;
          if (should_warn(last_warn, 5.0)) {
            logger->warn("map_odom_mode=external: failed to lookup external {} -> {}: {}", odom_frame_id, base_frame_id, e.what());
          }
        }
      }
    }

    check_imu_lidar_tf();
    if (publish_tf_imu_lidar) {
      // TF parent->child carries the pose of the child in the parent frame, i.e., T_imu_lidar
      trans.header.frame_id = imu_frame_id;
      trans.child_frame_id = lidar_frame_id;
      trans.transform = to_transform_msg(T_imu_lidar);
      tf_broadcaster->sendTransform(trans);
    }
  }

  const auto warn_scanend = [&] {
    if (std::abs(imu_end_time - new_frame->stamp) < 1e-3 && should_warn(last_scanend_warn, 10.0)) {
      logger->warn("Scan end time is too close to the frame time (imu_end_time={}, frame_time={})", imu_end_time, new_frame->stamp);
      logger->warn("Possibly due to the lack of IMU data");
    }
  };

  // --- ~/odometry/local (odom frame, smoother output) and ~/odometry/global (map frame, with loop closure) ---
  if (corrected && T_imu_child && has_subs(odometry_local_pub)) {
    const auto s = make_body_state(T_odom_imu, v_odom_imu, w_imu, C_pose, C_vel_odom_ptr, twist_angular_cov, *T_imu_child);
    odometry_local_pub->publish(to_odom_msg(s, stamp, odom_frame_id, child_frame_id));
  }
  if (corrected && T_imu_child && has_subs(odometry_global_pub)) {
    const Eigen::Vector3d v_world_imu = T_world_odom.linear() * v_odom_imu;
    const Eigen::Matrix3d C_vel_world = T_world_odom.linear() * C_vel_odom * T_world_odom.linear().transpose();
    const auto s = make_body_state(T_world_imu, v_world_imu, w_imu, C_pose, C_vel_odom_ptr ? &C_vel_world : nullptr, twist_angular_cov, *T_imu_child);
    odometry_global_pub->publish(to_odom_msg(s, stamp, map_frame_id, child_frame_id));
  }

  // --- Odometry (odom frame, without loop closure) ---
  auto& odom_pub = !corrected ? this->odom_pub : this->odom_corrected_pub;
  auto& lidar_odom_pub = !corrected ? this->lidar_odom_pub : this->lidar_odom_corrected_pub;
  if (T_imu_child && has_subs(odom_pub)) {
    const auto s = make_body_state(T_odom_imu, v_odom_imu, w_imu, C_pose, C_vel_odom_ptr, twist_angular_cov, *T_imu_child);
    odom_pub->publish(to_odom_msg(s, stamp, odom_frame_id, child_frame_id));
  }
  if (has_subs(lidar_odom_pub)) {
    const auto s = make_body_state(T_odom_imu, v_odom_imu, w_imu, C_pose, C_vel_odom_ptr, twist_angular_cov, T_imu_lidar);
    lidar_odom_pub->publish(to_odom_msg(s, stamp, odom_frame_id, lidar_frame_id));
  }

  auto& odom_scan_end_pub = !corrected ? this->odom_scanend_pub : this->odom_scanend_corrected_pub;
  auto& lidar_odom_scan_end_pub = !corrected ? this->lidar_odom_scanend_pub : this->lidar_odom_scanend_corrected_pub;
  if ((T_imu_child && has_subs(odom_scan_end_pub)) || has_subs(lidar_odom_scan_end_pub)) {
    warn_scanend();
    const Eigen::Isometry3d T_odom_imuend = T_odom_imu * T_imubegin_imuend;
    if (T_imu_child && has_subs(odom_scan_end_pub)) {
      const auto s = make_body_state(T_odom_imuend, v_odom_imu, w_imu, nullptr, nullptr, twist_angular_cov, *T_imu_child);
      odom_scan_end_pub->publish(to_odom_msg(s, imu_end_stamp, odom_frame_id, child_frame_id));
    }
    if (has_subs(lidar_odom_scan_end_pub)) {
      const auto s = make_body_state(T_odom_imuend, v_odom_imu, w_imu, nullptr, nullptr, twist_angular_cov, T_imu_lidar);
      lidar_odom_scan_end_pub->publish(to_odom_msg(s, imu_end_stamp, odom_frame_id, lidar_frame_id));
    }
  }

  // --- Poses (map frame, with loop closure) ---
  auto& pose_pub = !corrected ? this->pose_pub : this->pose_corrected_pub;
  auto& lidar_pose_pub = !corrected ? this->lidar_pose_pub : this->lidar_pose_corrected_pub;
  // Covariance topics exist only for the corrected estimates
  const bool publish_pose_with_cov = corrected && has_subs(pose_corrected_with_cov_pub);
  const bool publish_lidar_pose_with_cov = corrected && has_subs(lidar_pose_corrected_with_cov_pub);

  if (T_imu_child && (has_subs(pose_pub) || publish_pose_with_cov)) {
    const auto s = make_body_state(T_world_imu, Eigen::Vector3d::Zero(), w_imu, C_pose, nullptr, twist_angular_cov, *T_imu_child);
    if (has_subs(pose_pub)) {
      pose_pub->publish(to_pose_stamped(s, stamp, map_frame_id));
    }
    if (publish_pose_with_cov) {
      pose_corrected_with_cov_pub->publish(to_pose_with_cov(s, stamp, map_frame_id));
    }
  }
  if (has_subs(lidar_pose_pub) || publish_lidar_pose_with_cov) {
    const auto s = make_body_state(T_world_imu, Eigen::Vector3d::Zero(), w_imu, C_pose, nullptr, twist_angular_cov, T_imu_lidar);
    if (has_subs(lidar_pose_pub)) {
      lidar_pose_pub->publish(to_pose_stamped(s, stamp, map_frame_id));
    }
    if (publish_lidar_pose_with_cov) {
      lidar_pose_corrected_with_cov_pub->publish(to_pose_with_cov(s, stamp, map_frame_id));
    }
  }

  auto& pose_scan_end_pub = !corrected ? this->pose_scanend_pub : this->pose_scanend_corrected_pub;
  auto& lidar_pose_scan_end_pub = !corrected ? this->lidar_pose_scanend_pub : this->lidar_pose_scanend_corrected_pub;
  const bool publish_pose_scanend_with_cov = corrected && has_subs(pose_scanend_corrected_with_cov_pub);
  const bool publish_lidar_pose_scanend_with_cov = corrected && has_subs(lidar_pose_scanend_corrected_with_cov_pub);
  const bool want_pose_scanend = T_imu_child && (has_subs(pose_scan_end_pub) || publish_pose_scanend_with_cov);
  const bool want_lidar_pose_scanend = has_subs(lidar_pose_scan_end_pub) || publish_lidar_pose_scanend_with_cov;
  if (want_pose_scanend || want_lidar_pose_scanend) {
    warn_scanend();
    const Eigen::Isometry3d T_world_imuend = T_world_imu * T_imubegin_imuend;

    // Transform the covariance to the scan-end pose (T_world_imuend = T_world_imu * T_imubegin_imuend)
    gtsam::Matrix6 C_imuend;
    if (C_pose) {
      const gtsam::Matrix6 Ad_T_imuend_imubegin = gtsam::Pose3(T_imubegin_imuend.inverse().matrix()).AdjointMap();
      C_imuend = Ad_T_imuend_imubegin * (*C_pose) * Ad_T_imuend_imubegin.transpose();
    }
    const gtsam::Matrix6* C_imuend_ptr = C_pose ? &C_imuend : nullptr;

    if (want_pose_scanend) {
      const auto s = make_body_state(T_world_imuend, Eigen::Vector3d::Zero(), w_imu, C_imuend_ptr, nullptr, twist_angular_cov, *T_imu_child);
      if (has_subs(pose_scan_end_pub)) {
        pose_scan_end_pub->publish(to_pose_stamped(s, imu_end_stamp, map_frame_id));
      }
      if (publish_pose_scanend_with_cov) {
        pose_scanend_corrected_with_cov_pub->publish(to_pose_with_cov(s, imu_end_stamp, map_frame_id));
      }
    }
    if (want_lidar_pose_scanend) {
      const auto s = make_body_state(T_world_imuend, Eigen::Vector3d::Zero(), w_imu, C_imuend_ptr, nullptr, twist_angular_cov, T_imu_lidar);
      if (has_subs(lidar_pose_scan_end_pub)) {
        lidar_pose_scan_end_pub->publish(to_pose_stamped(s, imu_end_stamp, map_frame_id));
      }
      if (publish_lidar_pose_scanend_with_cov) {
        lidar_pose_scanend_corrected_with_cov_pub->publish(to_pose_with_cov(s, imu_end_stamp, map_frame_id));
      }
    }
  }

  // --- IMU bias ---
  if (corrected && (has_subs(imu_bias_pub) || has_subs(imu_bias_with_cov_pub))) {
    geometry_msgs::msg::TwistStamped bias;
    bias.header.stamp = stamp;
    bias.header.frame_id = imu_frame_id;
    bias.twist.linear.x = new_frame->imu_bias(0);
    bias.twist.linear.y = new_frame->imu_bias(1);
    bias.twist.linear.z = new_frame->imu_bias(2);
    bias.twist.angular.x = new_frame->imu_bias(3);
    bias.twist.angular.y = new_frame->imu_bias(4);
    bias.twist.angular.z = new_frame->imu_bias(5);

    if (has_subs(imu_bias_pub)) {
      imu_bias_pub->publish(bias);
    }
    if (has_subs(imu_bias_with_cov_pub)) {
      geometry_msgs::msg::TwistWithCovarianceStamped bias_with_cov;
      bias_with_cov.header = bias.header;
      bias_with_cov.twist.twist = bias.twist;
      if (new_frame->cov_computed) {
        const RowMajor6 cov_bias = new_frame->bias_cov;
        std::copy(cov_bias.data(), cov_bias.data() + 36, bias_with_cov.twist.covariance.begin());
      }
      imu_bias_with_cov_pub->publish(bias_with_cov);
    }
  }

  // --- Point clouds ---
  auto& points_pub = !corrected ? this->points_pub : this->points_corrected_pub;
  if (has_subs(points_pub)) {
    // Publish points in their own coordinate frame
    std::string frame_id;
    switch (new_frame->frame_id) {
      case FrameID::LIDAR:
        frame_id = lidar_frame_id;
        break;
      case FrameID::IMU:
        frame_id = imu_frame_id;
        break;
      case FrameID::WORLD:
        frame_id = odom_frame_id;  // GLIM's world frame is the odometry frame
        break;
    }

    if (new_frame->frame_id == FrameID::WORLD && !A.isApprox(Eigen::Isometry3d::Identity())) {
      std::vector<Eigen::Vector4d> transformed(new_frame->frame->size());
      for (int i = 0; i < new_frame->frame->size(); i++) {
        transformed[i] = A * new_frame->frame->points[i];
      }
      gtsam_points::PointCloud frame;
      frame.num_points = new_frame->frame->size();
      frame.points = transformed.data();
      frame.times = new_frame->frame->times;
      frame.intensities = new_frame->frame->intensities;
      points_pub->publish(*frame_to_pointcloud2(frame_id, new_frame->stamp, frame));
    } else {
      auto points = frame_to_pointcloud2(frame_id, new_frame->stamp, *new_frame->frame);
      points_pub->publish(*points);
    }
  }

  auto& aligned_points_pub = !corrected ? this->aligned_points_pub : this->aligned_points_corrected_pub;
  if (has_subs(aligned_points_pub)) {
    // Publish points aligned to the map frame (odom -> map with the loop-closure correction)
    const Eigen::Isometry3d T_map_sensor = T_world_odom * A * new_frame->T_world_sensor();
    std::vector<Eigen::Vector4d> transformed(new_frame->frame->size());
    for (int i = 0; i < new_frame->frame->size(); i++) {
      transformed[i] = T_map_sensor * new_frame->frame->points[i];
    }

    gtsam_points::PointCloud frame;
    frame.num_points = new_frame->frame->size();
    frame.points = transformed.data();
    frame.times = new_frame->frame->times;
    frame.intensities = new_frame->frame->intensities;

    auto points = frame_to_pointcloud2(map_frame_id, new_frame->stamp, frame);
    aligned_points_pub->publish(*points);
  }
}

// Refuse to publish imu->lidar if the LiDAR frame already has another parent (e.g., from the URDF)
void RvizViewer::check_imu_lidar_tf() {
  if (imu_lidar_checked || !publish_tf_imu_lidar) {
    return;
  }
  imu_lidar_checked = true;
  std::string parent;
  if (tf_buffer->_getParent(lidar_frame_id, tf2::TimePointZero, parent) && parent != imu_frame_id) {
    logger->warn("{} already has parent {} in TF (URDF?); not publishing {} -> {}", lidar_frame_id, parent, imu_frame_id, lidar_frame_id);
    imu_lidar_refused = true;
    publish_tf_imu_lidar = false;
  }
}

// ~/odometry/imu_rate: propagate the latest optimized state with the IMU samples received since (strapdown, bias-corrected)
void RvizViewer::publish_imu_rate() {
  if (!imu_rate_active || !latest_state) {
    return;
  }

  std::vector<ImuSample, Eigen::aligned_allocator<ImuSample>> samples;
  {
    std::lock_guard<std::mutex> lock(imu_mutex);
    for (const auto& imu : imu_buffer) {
      if (imu[0] > latest_state->stamp) {
        samples.push_back(imu);
      }
    }
  }
  if (samples.empty() || samples.back()[0] <= last_imu_rate_stamp) {
    return;
  }

  const Eigen::Vector3d gravity(0.0, 0.0, -9.80665);
  const Eigen::Vector3d acc_bias = latest_state->bias.head<3>();
  const Eigen::Vector3d gyro_bias = latest_state->bias.tail<3>();

  Eigen::Matrix3d R = latest_state->T_odom_imu.linear();
  Eigen::Vector3d p = latest_state->T_odom_imu.translation();
  Eigen::Vector3d v = latest_state->v_odom_imu;
  Eigen::Vector3d w = Eigen::Vector3d::Zero();
  double t = latest_state->stamp;
  for (const auto& imu : samples) {
    const double dt = imu[0] - t;
    if (dt <= 0.0) continue;
    const Eigen::Vector3d a_world = R * (imu.segment<3>(1) - acc_bias) + gravity;
    w = imu.segment<3>(4) - gyro_bias;
    p += v * dt + 0.5 * a_world * dt * dt;
    v += a_world * dt;
    const double angle = w.norm() * dt;
    if (angle > 1e-12) {
      R = R * Eigen::AngleAxisd(angle, w.normalized()).toRotationMatrix();
    }
    t = imu[0];
  }
  last_imu_rate_stamp = t;

  std::optional<Eigen::Isometry3d> T_imu_child;
  std::string child_frame_id;
  switch (odom_child_frame) {
    case BodyFrame::IMU:
      T_imu_child = Eigen::Isometry3d::Identity();
      child_frame_id = imu_frame_id;
      break;
    case BodyFrame::LIDAR:
      T_imu_child = latest_state->T_lidar_imu.inverse();
      child_frame_id = lidar_frame_id;
      break;
    case BodyFrame::BASE:
      T_imu_child = lookup_T_imu_base();
      child_frame_id = base_frame_id;
      break;
  }
  if (!T_imu_child) {
    return;
  }

  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = Eigen::Quaterniond(R).normalized().toRotationMatrix();
  T.translation() = p;
  const Eigen::Matrix3d C_vel = fallback_velocity_var * Eigen::Matrix3d::Identity();
  const auto state = make_body_state(T, v, w, nullptr, &C_vel, twist_angular_cov, *T_imu_child);
  odometry_imu_rate_pub->publish(to_odom_msg(state, from_sec(t), odom_frame_id, child_frame_id));
}

rcl_interfaces::msg::SetParametersResult RvizViewer::on_set_parameters(const std::vector<rclcpp::Parameter>& params) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  static const std::set<std::string> restart_required = {
    "imu_frame_id", "lidar_frame_id", "base_frame_id", "odom_frame_id", "map_frame_id", "odom_child_frame", "outputs",
    "map_odom_mode", "anchor_frame", "max_publish_queue", "map_voxel_size", "grid_resolution"};

  bool grid_changed = false;
  for (const auto& p : params) {
    const auto& n = p.get_name();
    if (restart_required.count(n)) {
      result.successful = false;
      result.reason = n + " cannot be changed at runtime (restart required)";
      return result;
    }
  }

  for (const auto& p : params) {
    const auto& n = p.get_name();
    if (n == "publish_tf") {
      publish_tf_map_odom = p.as_bool();
      publish_tf_odom_base = p.as_bool() && !map_odom_external;
      publish_tf_imu_lidar = p.as_bool() && !imu_lidar_refused;
    } else if (n == "publish_tf_map_odom") publish_tf_map_odom = p.as_bool();
    else if (n == "publish_tf_odom_base") publish_tf_odom_base = p.as_bool() && !map_odom_external;
    else if (n == "publish_tf_imu_lidar") publish_tf_imu_lidar = p.as_bool() && !imu_lidar_refused;
    else if (n == "tf_time_offset") tf_time_offset = p.as_double();
    else if (n == "map_odom_planar") map_odom_planar = p.as_bool();
    else if (n == "map_odom_rate") map_odom_rate = p.as_double();
    else if (n == "map_odom_tolerance") map_odom_tolerance = p.as_double();
    else if (n == "external_odom_timeout") external_odom_timeout = p.as_double();
    else if (n == "twist_angular_cov") twist_angular_cov = p.as_double();
    else if (n == "twist_angular_window") twist_angular_window = p.as_double();
    else if (n == "fallback_position_var") fallback_position_var = p.as_double();
    else if (n == "fallback_rotation_var") fallback_rotation_var = p.as_double();
    else if (n == "fallback_velocity_var") fallback_velocity_var = p.as_double();
    else if (n == "map_publish_period") map_publish_period = p.as_double();
    else if (n == "grid_z_min") { grid_z_min = p.as_double(); grid_changed = true; }
    else if (n == "grid_z_max") { grid_z_max = p.as_double(); grid_changed = true; }
    else if (n == "grid_min_points") { grid_min_points = p.as_int(); grid_changed = true; }
    else if (n == "grid_max_range") { grid_max_range = p.as_double(); grid_changed = true; }
    else if (n == "grid_ground_cell") { grid_ground_cell = p.as_double(); grid_changed = true; }
  }

  if (grid_changed) {
    grid_dirty = true;
  }
  return result;
}

void RvizViewer::globalmap_on_update_submaps(const std::vector<SubMap::Ptr>& submaps) {
  if (submaps.empty()) {
    return;
  }
  const SubMap::ConstPtr latest_submap = submaps.back();

  const double stamp_endpoint_R = latest_submap->odom_frames.back()->stamp;
  const Eigen::Isometry3d T_world_endpoint_R = latest_submap->T_world_origin * latest_submap->T_origin_endpoint_R;
  {
    std::lock_guard<std::mutex> lock(trajectory_mutex);
    trajectory->update_anchor(stamp_endpoint_R, T_world_endpoint_R);
  }

  std::vector<SubMap::ConstPtr> all(submaps.begin(), submaps.end());
  std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> poses(submaps.size());
  for (size_t i = 0; i < submaps.size(); i++) {
    poses[i] = submaps[i]->T_world_origin;
  }

  // Cache new submaps (downsampled) and the updated poses in the map thread.
  // on_update_submaps also fires from GlobalMapping::optimize() without new submaps: then only poses change.
  invoke([this, all, poses] {
    std::vector<SubmapCache> new_caches;
    size_t num_cached;
    {
      std::lock_guard<std::mutex> lock(map_mutex);
      num_cached = submap_caches.size();
    }
    for (size_t i = num_cached; i < all.size(); i++) {
      new_caches.push_back(make_submap_cache(all[i]));
    }

    std::lock_guard<std::mutex> lock(map_mutex);
    submap_caches.insert(submap_caches.end(), new_caches.begin(), new_caches.end());
    submap_poses = poses;
    map_dirty = grid_dirty = true;
  });
}

void RvizViewer::invoke(const std::function<void()>& task) {
  {
    std::lock_guard<std::mutex> lock(map_task_mutex);
    invoke_queue.push_back(task);
  }
  map_task_cv.notify_one();
}

void RvizViewer::map_loop() {
  while (!kill_switch) {
    std::vector<std::function<void()>> tasks;
    {
      std::unique_lock<std::mutex> lock(map_task_mutex);
      map_task_cv.wait_for(lock, std::chrono::milliseconds(500), [this] { return kill_switch || !invoke_queue.empty(); });
      if (kill_switch) {
        break;
      }
      tasks.swap(invoke_queue);
    }

    for (const auto& task : tasks) {
      task();
    }

    if (initialized) {
      publish_map_and_grid();
    }
  }
}

RvizViewer::SubmapCache RvizViewer::make_submap_cache(const SubMap::ConstPtr& submap) {
  SubmapCache cache;
  cache.points = gtsam_points::voxelgrid_sampling(submap->frame, map_voxel_size);

  const auto grid_points = gtsam_points::voxelgrid_sampling(submap->frame, grid_resolution);
  cache.grid_points.reserve(grid_points->size());
  for (size_t i = 0; i < grid_points->size(); i++) {
    cache.grid_points.push_back(grid_points->points[i].head<3>());
  }
  return cache;
}

// Requires map_mutex
gtsam_points::PointCloudCPU::Ptr RvizViewer::build_map(const Eigen::Isometry3d& anchor) {
  const size_t n = std::min(submap_caches.size(), submap_poses.size());
  size_t total = 0;
  for (size_t i = 0; i < n; i++) {
    total += submap_caches[i].points->size();
  }

  std::vector<Eigen::Vector4d> points;
  points.reserve(total);
  for (size_t i = 0; i < n; i++) {
    const auto& cloud = submap_caches[i].points;
    const Eigen::Isometry3d T_map_submap = anchor * submap_poses[i];
    for (size_t j = 0; j < cloud->size(); j++) {
      points.push_back(T_map_submap * cloud->points[j]);
    }
  }

  auto merged = std::make_shared<gtsam_points::PointCloudCPU>(points);
  // Remove the overlap between submaps
  return gtsam_points::voxelgrid_sampling(merged, map_voxel_size, 2);
}

// Requires map_mutex
std::shared_ptr<nav_msgs::msg::OccupancyGrid> RvizViewer::build_grid(const Eigen::Isometry3d& anchor) {
  const size_t n = std::min(submap_caches.size(), submap_poses.size());
  if (n == 0) {
    return nullptr;
  }
  // Submap poses in the published map frame (the anchor only rotates about z: heights stay comparable)
  std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> poses(n);
  for (size_t i = 0; i < n; i++) {
    poses[i] = anchor * submap_poses[i];
  }

  // Bounds
  Eigen::Vector2d min_pt = Eigen::Vector2d::Constant(std::numeric_limits<double>::max());
  Eigen::Vector2d max_pt = -min_pt;
  const auto extend = [&](const Eigen::Vector3d& p) {
    min_pt = min_pt.cwiseMin(p.head<2>());
    max_pt = max_pt.cwiseMax(p.head<2>());
  };
  for (size_t i = 0; i < n; i++) {
    const Eigen::Vector3d o = poses[i].translation();
    extend(o);
    for (const auto& p : submap_caches[i].grid_points) {
      const Eigen::Vector3d w = poses[i] * p;
      if ((w - o).head<2>().norm() <= grid_max_range) extend(w);
    }
  }
  min_pt -= Eigen::Vector2d::Constant(2 * grid_resolution);
  max_pt += Eigen::Vector2d::Constant(2 * grid_resolution);

  const int W = static_cast<int>(std::ceil((max_pt.x() - min_pt.x()) / grid_resolution));
  const int H = static_cast<int>(std::ceil((max_pt.y() - min_pt.y()) / grid_resolution));
  if (static_cast<size_t>(W) * H > 50'000'000) {
    logger->warn("occupancy grid too large ({}x{} cells); increase grid_resolution", W, H);
    return nullptr;
  }

  std::vector<uint16_t> hits(static_cast<size_t>(W) * H, 0);
  std::vector<uint16_t> passes(static_cast<size_t>(W) * H, 0);
  const auto to_cell = [&](const Eigen::Vector3d& w) {
    return Eigen::Vector2i(static_cast<int>((w.x() - min_pt.x()) / grid_resolution), static_cast<int>((w.y() - min_pt.y()) / grid_resolution));
  };
  const auto inside = [&](const Eigen::Vector2i& c) { return c.x() >= 0 && c.y() >= 0 && c.x() < W && c.y() < H; };
  const auto sat_inc = [](uint16_t& v) { v = v < std::numeric_limits<uint16_t>::max() ? v + 1 : v; };

  // Bresenham ray from a to b: marks passes, excluding b
  const auto raycast = [&](const Eigen::Vector2i& a, const Eigen::Vector2i& b) {
    int x0 = a.x(), y0 = a.y();
    const int dx = std::abs(b.x() - x0), dy = -std::abs(b.y() - y0);
    const int sx = x0 < b.x() ? 1 : -1, sy = y0 < b.y() ? 1 : -1;
    int err = dx + dy;
    while (x0 != b.x() || y0 != b.y()) {
      sat_inc(passes[static_cast<size_t>(y0) * W + x0]);
      const int e2 = 2 * err;
      if (e2 >= dy) {
        err += dy;
        x0 += sx;
      }
      if (e2 <= dx) {
        err += dx;
        y0 += sy;
      }
    }
  };

  for (size_t i = 0; i < n; i++) {
    const Eigen::Vector3d o = poses[i].translation();
    const Eigen::Vector2i oc = to_cell(o);
    if (!inside(oc)) continue;

    // Local ground: lowest point of THIS submap in each grid_ground_cell column. Estimated per submap because
    // overlapping submaps can be vertically offset by more than the obstacle band (robust to slopes too).
    std::vector<Eigen::Vector3d> pts;
    pts.reserve(submap_caches[i].grid_points.size());
    std::unordered_map<int64_t, float> ground;
    const auto ground_key = [&](const Eigen::Vector3d& w) {
      return (static_cast<int64_t>(std::floor(w.y() / grid_ground_cell)) << 32) ^ (static_cast<int64_t>(std::floor(w.x() / grid_ground_cell)) & 0xffffffff);
    };
    for (const auto& p : submap_caches[i].grid_points) {
      const Eigen::Vector3d w = poses[i] * p;
      if ((w - o).head<2>().norm() > grid_max_range || !inside(to_cell(w))) continue;
      pts.push_back(w);
      auto found = ground.try_emplace(ground_key(w), static_cast<float>(w.z()));
      found.first->second = std::min(found.first->second, static_cast<float>(w.z()));
    }

    std::unordered_map<int64_t, int> obstacle_cells;  // cell -> number of voxels in the band
    std::unordered_set<int64_t> free_cells;
    for (const auto& w : pts) {
      const Eigen::Vector2i c = to_cell(w);
      const double h = w.z() - ground[ground_key(w)];
      const int64_t idx = static_cast<int64_t>(c.y()) * W + c.x();
      if (h >= grid_z_min && h <= grid_z_max) {
        obstacle_cells[idx]++;
      } else if (h < grid_z_min) {
        free_cells.insert(idx);  // ground: observed traversable surface
      }
    }

    for (const auto& [idx, count] : obstacle_cells) {
      raycast(oc, Eigen::Vector2i(idx % W, idx / W));
      for (int k = 0; k < count; k++) sat_inc(hits[idx]);
    }
    for (const auto idx : free_cells) {
      raycast(oc, Eigen::Vector2i(idx % W, idx / W));
      sat_inc(passes[idx]);
    }
  }

  auto grid = std::make_shared<nav_msgs::msg::OccupancyGrid>();
  grid->header.frame_id = map_frame_id;
  grid->header.stamp = clock->now();
  grid->info.map_load_time = grid->header.stamp;
  grid->info.resolution = grid_resolution;
  grid->info.width = W;
  grid->info.height = H;
  grid->info.origin.position.x = min_pt.x();
  grid->info.origin.position.y = min_pt.y();
  grid->info.origin.orientation.w = 1.0;
  grid->data.resize(static_cast<size_t>(W) * H);
  for (size_t k = 0; k < grid->data.size(); k++) {
    grid->data[k] = hits[k] >= grid_min_points ? 100 : (passes[k] > 0 || hits[k] > 0 ? 0 : -1);
  }
  return grid;
}

void RvizViewer::publish_map_and_grid() {
  std::lock_guard<std::mutex> lock(map_mutex);
  if (submap_caches.empty()) {
    return;
  }
  const auto anchor = get_anchor();
  if (!anchor) {
    return;
  }
  const rclcpp::Time now = clock->now();

  // Publish when changed (at most every map_publish_period) or when a new subscriber appears
  const size_t map_subs = map_pub ? map_pub->get_subscription_count() : 0;
  if (map_subs && ((map_dirty && (now - last_globalmap_pub_time).seconds() >= map_publish_period) || map_subs > last_map_subs)) {
    auto map = build_map(*anchor);
    map_pub->publish(*frame_to_pointcloud2(map_frame_id, now.seconds(), *map));
    last_globalmap_pub_time = now;
    map_dirty = false;
    logger->debug("published map ({} points)", map->size());
  }
  last_map_subs = map_subs;

  const size_t grid_subs = grid_pub ? grid_pub->get_subscription_count() : 0;
  if (grid_subs && ((grid_dirty && (now - last_grid_pub_time).seconds() >= map_publish_period) || grid_subs > last_grid_subs)) {
    auto grid = build_grid(*anchor);
    if (grid) {
      grid_pub->publish(*grid);
      logger->debug("published grid ({}x{})", grid->info.width, grid->info.height);
    }
    last_grid_pub_time = now;
    grid_dirty = false;
  }
  last_grid_subs = grid_subs;
}

void RvizViewer::at_exit(const std::string& dump_path) {
  // Submap updates are applied asynchronously by the map thread: wait until everything queued so far
  // (e.g., the last submaps and the final optimization before shutdown) has been applied.
  if (map_thread.joinable() && !kill_switch) {
    auto done = std::make_shared<std::promise<void>>();
    auto applied = done->get_future();
    invoke([done] { done->set_value(); });
    if (applied.wait_for(std::chrono::seconds(60)) != std::future_status::ready) {
      logger->warn("timeout while waiting for pending map updates; saving the current map");
    }
  }

  std::lock_guard<std::mutex> lock(map_mutex);
  if (submap_caches.empty() || !clock) {
    return;
  }
  std::filesystem::create_directories(dump_path);

  std::optional<Eigen::Isometry3d> anchor = get_anchor();
  if (!anchor) {
    logger->warn("anchor_frame=base: the anchor was never resolved ({} -> {} unknown); saving map.pcd/map.pgm in GLIM's world frame", imu_frame_id, base_frame_id);
    anchor = Eigen::Isometry3d::Identity();
  }
  {
    // GLIM's own dump (graph, submaps) stays in GLIM's world frame: record how map.pcd/map.pgm relate to it
    std::ofstream ofs(dump_path + "/map_anchor.txt");
    ofs << "# T_map_world: pose of GLIM's world frame in the frame of map.pcd / map.yaml (p_map = T_map_world * p_world)\n"
        << std::setprecision(12) << anchor->matrix() << "\n";
  }

  // Point cloud map (binary PCD)
  const auto map = build_map(*anchor);
  {
    std::ofstream ofs(dump_path + "/map.pcd", std::ios::binary);
    ofs << "# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
        << "WIDTH " << map->size() << "\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS " << map->size() << "\nDATA binary\n";
    for (size_t i = 0; i < map->size(); i++) {
      const Eigen::Vector3f p = map->points[i].head<3>().cast<float>();
      ofs.write(reinterpret_cast<const char*>(p.data()), sizeof(float) * 3);
    }
  }
  logger->info("saved {} ({} points, {} submaps)", dump_path + "/map.pcd", map->size(), std::min(submap_caches.size(), submap_poses.size()));

  // Occupancy grid (nav2_map_server trinary format)
  const auto grid = build_grid(*anchor);
  if (grid) {
    const int W = grid->info.width, H = grid->info.height;
    std::ofstream pgm(dump_path + "/map.pgm", std::ios::binary);
    pgm << "P5\n# glim_ros occupancy grid\n" << W << " " << H << "\n255\n";
    for (int y = H - 1; y >= 0; y--) {  // image rows go from top (max y) to bottom
      for (int x = 0; x < W; x++) {
        const int8_t v = grid->data[static_cast<size_t>(y) * W + x];
        const uint8_t px = v < 0 ? 205 : (v >= 50 ? 0 : 254);
        pgm.put(static_cast<char>(px));
      }
    }

    std::ofstream yaml(dump_path + "/map.yaml");
    yaml << std::setprecision(9) << "image: map.pgm\nmode: trinary\nresolution: " << grid->info.resolution << "\norigin: [" << grid->info.origin.position.x << ", "
         << grid->info.origin.position.y << ", 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n";
    logger->info("saved {} ({}x{} cells)", dump_path + "/map.yaml", W, H);
  }
}

}  // namespace glim

extern "C" glim::ExtensionModule* create_extension_module() {
  return new glim::RvizViewer();
}
