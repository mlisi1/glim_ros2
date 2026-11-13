#include <glim_ros/rviz_viewer.hpp>

#include <mutex>
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
    s.pose_cov = reorder_cov(Ad_T_X_imu * (*C_pose_imu) * Ad_T_X_imu.transpose());
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
  max_frame_queue = 64;
  map_odom_external = false;
  map_odom_planar = false;
  map_odom_rate = 20.0;
  map_odom_tolerance = 0.1;
  external_odom_timeout = 0.1;

  trajectory.reset(new TrajectoryManager);

  set_callbacks();

  initialized = false;
  kill_switch = false;
  thread = std::thread([this] { worker_loop(); });
}

RvizViewer::~RvizViewer() {
  kill_switch = true;
  queue_cv.notify_all();
  thread.join();
}

std::vector<GenericTopicSubscription::Ptr> RvizViewer::create_subscriptions(rclcpp::Node& node) {
  clock = node.get_clock();
  tf_buffer = std::make_unique<tf2_ros::Buffer>(clock);
  tf_listener = std::make_unique<tf2_ros::TransformListener>(*tf_buffer);
  tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(node);

  // --- ROS parameters (declared on the glim_ros node, so they can be set from launch files) ---
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

  logger->info(
    "TF: map->odom={} ({}{}) odom->base={} imu->lidar={} | odom_child_frame={}",
    publish_tf_map_odom,
    map_odom_mode,
    map_odom_planar ? ", planar" : "",
    publish_tf_odom_base,
    publish_tf_imu_lidar,
    child);

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

  create(map_pub, "map", map_qos);

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

  initialized = true;
  queue_cv.notify_all();

  return {};
}

void RvizViewer::set_callbacks() {
  using std::placeholders::_1;
  OdometryEstimationCallbacks::on_insert_imu.add(
    [this](const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel) { on_insert_imu(stamp, angular_vel); });
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

void RvizViewer::on_insert_imu(double stamp, const Eigen::Vector3d& angular_vel) {
  std::lock_guard<std::mutex> lock(imu_mutex);
  gyro_buffer.emplace_back(stamp, angular_vel.x(), angular_vel.y(), angular_vel.z());
  while (!gyro_buffer.empty() && gyro_buffer.front()[0] < stamp - 5.0) {
    gyro_buffer.pop_front();
  }
}

// Mean gyro reading in [t0, t1]; falls back to the sample closest to the window center
Eigen::Vector3d RvizViewer::mean_gyro(double t0, double t1) {
  std::lock_guard<std::mutex> lock(imu_mutex);
  if (gyro_buffer.empty()) {
    return Eigen::Vector3d::Zero();
  }

  Eigen::Vector3d sum = Eigen::Vector3d::Zero();
  int n = 0;
  const Eigen::Vector4d* closest = &gyro_buffer.front();
  const double tc = 0.5 * (t0 + t1);
  for (const auto& g : gyro_buffer) {
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
    std::vector<std::function<void()>> tasks;
    {
      std::unique_lock<std::mutex> lock(queue_mutex);
      const double wait_sec = map_odom_external ? std::min(0.1, 0.5 / std::max(map_odom_rate, 1.0)) : 0.1;
      queue_cv.wait_for(lock, std::chrono::duration<double>(wait_sec), [this] {
        return kill_switch || (initialized && (!frame_queue.empty() || !invoke_queue.empty()));
      });
      if (kill_switch) {
        break;
      }
      if (!initialized) {
        continue;
      }
      frames.swap(frame_queue);
      tasks.swap(invoke_queue);
    }

    for (const auto& task : tasks) {
      task();
    }
    for (const auto& frame : frames) {
      publish_frame(frame);
    }
    publish_map_odom_periodic();
  }
}

// External mode: republish the latest map->odom at a fixed rate, future-dated so that lookups at "now" succeed
void RvizViewer::publish_map_odom_periodic() {
  if (!map_odom_external || !publish_tf_map_odom || !latest_T_map_odom) {
    return;
  }
  const rclcpp::Time now = clock->now();
  if ((now - last_map_odom_pub).seconds() < 1.0 / std::max(map_odom_rate, 1e-3)) {
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
  if (base_frame_id == imu_frame_id) {
    return Eigen::Isometry3d::Identity();
  }
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

void RvizViewer::publish_frame(const FrameTask& task) {
  const auto& new_frame = task.frame;
  const bool corrected = task.corrected;

  resolve_frame_ids();

  const Eigen::Isometry3d T_odom_imu = new_frame->T_world_imu;
  const Eigen::Vector3d v_odom_imu = new_frame->v_world_imu;
  const Eigen::Isometry3d T_lidar_imu = new_frame->T_lidar_imu;
  const Eigen::Isometry3d T_imu_lidar = T_lidar_imu.inverse();

  const Eigen::Isometry3d T_world_odom = task.T_world_odom;
  const Eigen::Isometry3d T_world_imu = T_world_odom * T_odom_imu;

  // Covariances (only available for optimized frames when enabled in GLIM)
  const gtsam::Matrix6* C_pose = new_frame->cov_computed ? &new_frame->pose_cov : nullptr;
  const Eigen::Matrix3d C_vel_odom = new_frame->vel_cov;
  const Eigen::Matrix3d* C_vel_odom_ptr = new_frame->cov_computed ? &C_vel_odom : nullptr;

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
          const Eigen::Isometry3d T_map_odom = T_map_base * T_odom_base.inverse();
          latest_T_map_odom = map_odom_planar ? flatten(T_map_odom) : T_map_odom;
          last_map_odom_pub = rclcpp::Time(0, 0, clock->get_clock_type());  // publish right away
        } catch (const tf2::TransformException& e) {
          static std::chrono::steady_clock::time_point last_warn;
          if (should_warn(last_warn, 5.0)) {
            logger->warn("map_odom_mode=external: failed to lookup external {} -> {}: {}", odom_frame_id, base_frame_id, e.what());
          }
        }
      }
    }

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

    auto points = frame_to_pointcloud2(frame_id, new_frame->stamp, *new_frame->frame);
    points_pub->publish(*points);
  }

  auto& aligned_points_pub = !corrected ? this->aligned_points_pub : this->aligned_points_corrected_pub;
  if (has_subs(aligned_points_pub)) {
    // Publish points aligned to the map frame (odom -> map with the loop-closure correction)
    const Eigen::Isometry3d T_map_sensor = T_world_odom * new_frame->T_world_sensor();
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

  std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> submap_poses(submaps.size());
  for (int i = 0; i < submaps.size(); i++) {
    submap_poses[i] = submaps[i]->T_world_origin;
  }

  // Invoke a submap concatenation task in the worker thread
  invoke([this, latest_submap, submap_poses] {
    // on_update_submaps also fires from GlobalMapping::optimize(), which runs periodically while
    // the input queues are idle and adds no submap. Only append when a submap was actually added.
    if (this->submaps.size() < submap_poses.size()) {
      this->submaps.push_back(latest_submap->frame);
    }

    if (!has_subs(map_pub)) {
      return;
    }

    // Publish global map every 10 seconds
    const rclcpp::Time now = clock->now();
    if (now - last_globalmap_pub_time < std::chrono::seconds(10)) {
      return;
    }
    last_globalmap_pub_time = now;

    logger->warn("Publishing global map is computationally demanding and not recommended");

    const size_t num_submaps = std::min(this->submaps.size(), submap_poses.size());
    size_t total_num_points = 0;
    for (size_t i = 0; i < num_submaps; i++) {
      total_num_points += this->submaps[i]->size();
    }

    // Concatenate all the submap points
    gtsam_points::PointCloudCPU::Ptr merged(new gtsam_points::PointCloudCPU);
    merged->num_points = total_num_points;
    merged->points_storage.resize(total_num_points);
    merged->points = merged->points_storage.data();

    size_t begin = 0;
    for (size_t i = 0; i < num_submaps; i++) {
      const auto& submap = this->submaps[i];
      std::transform(submap->points, submap->points + submap->size(), merged->points + begin, [&](const Eigen::Vector4d& p) { return submap_poses[i] * p; });
      begin += submap->size();
    }

    auto points_msg = frame_to_pointcloud2(map_frame_id, now.seconds(), *merged);
    map_pub->publish(*points_msg);
  });
}

void RvizViewer::invoke(const std::function<void()>& task) {
  {
    std::lock_guard<std::mutex> lock(queue_mutex);
    invoke_queue.push_back(task);
  }
  queue_cv.notify_one();
}

}  // namespace glim

extern "C" glim::ExtensionModule* create_extension_module() {
  return new glim::RvizViewer();
}
