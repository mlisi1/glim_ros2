#include <glim_ros/glim_ros.hpp>

#define GLIM_ROS2

#include <deque>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <thread>
#include <iostream>
#include <functional>
#include <boost/format.hpp>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <gtsam_points/optimizers/linearization_hook.hpp>
#include <gtsam_points/cuda/nonlinear_factor_set_gpu_create.hpp>

#include <glim/util/debug.hpp>
#include <glim/util/config.hpp>
#include <glim/util/logging.hpp>
#include <glim/util/time_keeper.hpp>
#include <glim/util/ros_cloud_converter.hpp>
#include <glim/util/extension_module.hpp>
#include <glim/util/extension_module_ros2.hpp>
#include <glim/preprocess/cloud_preprocessor.hpp>
#include <glim/odometry/async_odometry_estimation.hpp>
#include <glim/mapping/async_sub_mapping.hpp>
#include <glim/mapping/async_global_mapping.hpp>
#include <glim_ros/ros_compatibility.hpp>
#include <glim_ros/ros_qos.hpp>

namespace glim {

GlimROS::GlimROS(const rclcpp::NodeOptions& options) : Node("glim_ros", options) {
  // Setup logger
  // Reuse the logger if it already exists (e.g., a second instance in the same process)
  auto logger = spdlog::get("glim");
  if (!logger) {
    logger = spdlog::stdout_color_mt("glim");
    logger->sinks().push_back(get_ringbuffer_sink());
  }
  spdlog::set_default_logger(logger);

  bool debug = false;
  this->declare_parameter<bool>("debug", false);
  this->get_parameter<bool>("debug", debug);

  if (debug) {
    spdlog::info("enable debug printing");
    const std::string debug_log_path = this->declare_parameter<std::string>("debug_log_path", "/tmp/glim_log.log");
    auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(debug_log_path, true);
    logger->sinks().push_back(file_sink);
    logger->set_level(spdlog::level::trace);

    print_system_info(logger);
  }

  this->declare_parameter<std::string>("dump_path", "/tmp/dump");

  dump_on_unload = false;
  this->declare_parameter<bool>("dump_on_unload", false);
  this->get_parameter<bool>("dump_on_unload", dump_on_unload);

  if (dump_on_unload) {
    spdlog::info("dump_on_unload={}", dump_on_unload);
  }

  // If true, a stopped extension module (e.g., closing the viewer window) shuts the node down (upstream behavior)
  shutdown_on_extension_exit = this->declare_parameter<bool>("shutdown_on_extension_exit", false);

  std::string config_path;
  this->declare_parameter<std::string>("config_path", "config");
  this->get_parameter<std::string>("config_path", config_path);

  if (config_path[0] != '/') {
    // config_path is relative to the glim directory
    config_path = ament_index_cpp::get_package_share_directory("glim") + "/" + config_path;
  }

  logger->info("config_path: {}", config_path);
  glim::GlobalConfig::instance(config_path);
  glim::Config config_ros(glim::GlobalConfig::get_config_path("config_ros"));

  // config_ros.json values can be overridden by ROS parameters of the same name (the JSON value is the default)
  const auto ros_param = [&](const std::string& name, const auto& json_default) {
    using T = std::decay_t<decltype(json_default)>;
    return this->declare_parameter<T>(name, config_ros.param<T>("glim_ros", name, json_default));
  };

  keep_raw_points = ros_param("keep_raw_points", false);
  imu_time_offset = ros_param("imu_time_offset", 0.0);
  points_time_offset = ros_param("points_time_offset", 0.0);
  acc_scale = ros_param("acc_scale", 0.0);
  ang_scale = ros_param("ang_scale", 1.0);

  glim::Config config_sensors(glim::GlobalConfig::get_config_path("config_sensors"));
  intensity_field = this->declare_parameter<std::string>("intensity_field", config_sensors.param<std::string>("sensors", "intensity_field", "intensity"));
  ring_field = this->declare_parameter<std::string>("ring_field", config_sensors.param<std::string>("sensors", "ring_field", ""));

  // Setup GPU-based linearization
#ifdef BUILD_GTSAM_POINTS_GPU
  gtsam_points::LinearizationHook::register_hook([]() { return gtsam_points::create_nonlinear_factor_set_gpu(); });
#endif

  // Preprocessing
  time_keeper.reset(new glim::TimeKeeper);
  preprocessor.reset(new glim::CloudPreprocessor);

  // Odometry estimation
  glim::Config config_odometry(glim::GlobalConfig::get_config_path("config_odometry"));
  const std::string odometry_estimation_so_name = config_odometry.param<std::string>("odometry_estimation", "so_name", "libodometry_estimation_cpu.so");
  spdlog::info("load {}", odometry_estimation_so_name);

  std::shared_ptr<glim::OdometryEstimationBase> odom = OdometryEstimationBase::load_module(odometry_estimation_so_name);
  if (!odom) {
    spdlog::critical("failed to load odometry estimation module");
    abort();
  }
  odometry_estimation.reset(new glim::AsyncOdometryEstimation(odom, odom->requires_imu()));

  // Sub mapping
  if (ros_param("enable_local_mapping", true)) {
    const std::string sub_mapping_so_name =
      glim::Config(glim::GlobalConfig::get_config_path("config_sub_mapping")).param<std::string>("sub_mapping", "so_name", "libsub_mapping.so");
    if (!sub_mapping_so_name.empty()) {
      spdlog::info("load {}", sub_mapping_so_name);
      auto sub = SubMappingBase::load_module(sub_mapping_so_name);
      if (sub) {
        sub_mapping.reset(new AsyncSubMapping(sub));
      }
    }
  }

  // Global mapping
  if (ros_param("enable_global_mapping", true)) {
    const std::string global_mapping_so_name =
      glim::Config(glim::GlobalConfig::get_config_path("config_global_mapping")).param<std::string>("global_mapping", "so_name", "libglobal_mapping.so");
    if (!global_mapping_so_name.empty()) {
      spdlog::info("load {}", global_mapping_so_name);
      auto global = GlobalMappingBase::load_module(global_mapping_so_name);
      if (global) {
        global_mapping.reset(new AsyncGlobalMapping(global));
      }
    }
  }

  // Extention modules
  const auto extensions = std::make_optional(ros_param("extension_modules", std::vector<std::string>{}));
  if (extensions && !extensions->empty()) {
    for (const auto& extension : *extensions) {
      if (extension.find("viewer") == std::string::npos && extension.find("monitor") == std::string::npos) {
        spdlog::warn("Extension modules are enabled!!");
        spdlog::warn("You must carefully check and follow the licenses of ext modules");

        try {
          const std::string config_ext_path = ament_index_cpp::get_package_share_directory("glim_ext") + "/config";
          spdlog::info("config_ext_path: {}", config_ext_path);
          glim::GlobalConfig::instance()->override_param<std::string>("global", "config_ext", config_ext_path);
        } catch (ament_index_cpp::PackageNotFoundError& e) {
          spdlog::warn("glim_ext package path was not found!!");
        }

        break;
      }
    }

    for (const auto& extension : *extensions) {
      spdlog::info("load {}", extension);
      auto ext_module = ExtensionModule::load_module(extension);
      if (ext_module == nullptr) {
        spdlog::error("failed to load {}", extension);
        continue;
      } else {
        extension_modules.push_back(ext_module);

        auto ext_module_ros = std::dynamic_pointer_cast<ExtensionModuleROS2>(ext_module);
        if (ext_module_ros) {
          const auto subs = ext_module_ros->create_subscriptions(*this);
          extension_subs.insert(extension_subs.end(), subs.begin(), subs.end());
        }
      }
    }
  }

  // ROS-related
  using std::placeholders::_1;
  const std::string imu_topic = ros_param("imu_topic", std::string());
  const std::string points_topic = ros_param("points_topic", std::string());
  const std::string image_topic = ros_param("image_topic", std::string());

  // IMU and points run in separate callback groups (with a multi-threaded executor) so that a slow point cloud
  // conversion/preprocessing never delays IMU input. The timer moving results between stages has its own group.
  imu_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  points_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  timer_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions imu_options;
  imu_options.callback_group = imu_callback_group;
  rclcpp::SubscriptionOptions points_options;
  points_options.callback_group = points_callback_group;

  // Subscribers
  rclcpp::SensorDataQoS default_imu_qos;
  default_imu_qos.get_rmw_qos_profile().depth = 1000;
  auto qos = get_qos_settings(config_ros, "glim_ros", "imu_qos", default_imu_qos);
  imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic, qos, std::bind(&GlimROS::imu_callback, this, _1), imu_options);

  rclcpp::SensorDataQoS default_points_qos;
  default_points_qos.keep_last(10);
  qos = get_qos_settings(config_ros, "glim_ros", "points_qos", default_points_qos);
  points_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(points_topic, qos, std::bind(&GlimROS::points_callback, this, _1), points_options);
#ifdef BUILD_WITH_CV_BRIDGE
  qos = get_qos_settings(config_ros, "glim_ros", "image_qos");
  image_sub = image_transport::create_subscription(this, image_topic, std::bind(&GlimROS::image_callback, this, _1), "raw", qos.get_rmw_qos_profile());
#endif

  for (const auto& sub : this->extension_subscriptions()) {
    spdlog::debug("subscribe to {}", sub->topic);
    sub->create_subscriber(*this);
  }

  // Map saving service
  saving = false;
  map_save_directory = this->declare_parameter<std::string>("map_save_directory", "/tmp/glim_maps");
  if (!map_save_directory.empty() && map_save_directory[0] == '~' && std::getenv("HOME")) {
    map_save_directory = std::string(std::getenv("HOME")) + map_save_directory.substr(1);
  }
  save_map_srv = this->create_service<std_srvs::srv::Trigger>(
    "~/save_map",
    [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
      if (saving.exchange(true)) {
        res->success = false;
        res->message = "a save is already in progress";
        return;
      }

      const std::time_t t = std::time(nullptr);
      std::ostringstream oss;
      oss << map_save_directory << "/" << std::put_time(std::localtime(&t), "%Y%m%d_%H%M%S");
      const std::string path = oss.str();

      // Saving optimizes and serializes the whole graph: never do it in the executor thread (it would stall sensor callbacks)
      if (save_thread.joinable()) {
        save_thread.join();
      }
      save_thread = std::thread([this, path] {
        save(path);
        spdlog::info("map saved to {}", path);
        saving = false;
      });

      res->success = true;
      res->message = path;
    });

  // Start timer
  timer = this->create_wall_timer(std::chrono::milliseconds(10), [this]() { timer_callback(); }, timer_callback_group);

  // Diagnostics (/diagnostics, 1 Hz)
  last_imu_time = last_points_time = last_odom_time = std::chrono::steady_clock::now();
  last_diag_time = std::chrono::steady_clock::now();
  diagnostics = std::make_unique<diagnostic_updater::Updater>(this);
  diagnostics->setHardwareID("glim");
  diagnostics->add("GLIM", this, &GlimROS::diagnostics_callback);

  spdlog::debug("initialized");
}

GlimROS::~GlimROS() {
  spdlog::debug("quit");
  if (save_thread.joinable()) {
    save_thread.join();
  }

  // Flush and save while the extension modules are still alive: GLIM callbacks reference them
  if (dump_on_unload) {
    const std::string dump_path = this->get_parameter("dump_path").as_string();
    wait(true);
    save(dump_path);
  }

  extension_modules.clear();
}

const std::vector<std::shared_ptr<GenericTopicSubscription>>& GlimROS::extension_subscriptions() {
  return extension_subs;
}

void GlimROS::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
  spdlog::trace("IMU: {}.{}", msg->header.stamp.sec, msg->header.stamp.nanosec);
  num_imu++;
  last_imu_time = std::chrono::steady_clock::now();
  if (!imu_frame_detected) {
    std::lock_guard<std::mutex> lock(config_mutex);
    if (!GlobalConfig::instance()->has_param("meta", "imu_frame_id")) {
      spdlog::debug("auto-detecting IMU frame ID: {}", msg->header.frame_id);
      GlobalConfig::instance()->override_param<std::string>("meta", "imu_frame_id", msg->header.frame_id);
    }
    imu_frame_detected = true;
  }

  if (std::abs(acc_scale) < 1e-6) {
    const double norm = Eigen::Vector3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z).norm();
    if (norm > 7.0 && norm < 12.0) {
      acc_scale = 1.0;
      spdlog::debug("assuming [m/s^2] for acceleration unit (acc_scale={}, norm={})", acc_scale, norm);
    } else if (norm > 0.8 && norm < 1.2) {
      acc_scale = 9.80665;
      spdlog::debug("assuming [g] for acceleration unit (acc_scale={}, norm={})", acc_scale, norm);
    } else {
      acc_scale = 1.0;
      spdlog::warn("unexpected acceleration norm {}. assuming [m/s^2] for acceleration unit (acc_scale={})", norm, acc_scale);
    }
  }

  const double imu_stamp = msg->header.stamp.sec + msg->header.stamp.nanosec / 1e9 + imu_time_offset;
  const Eigen::Vector3d linear_acc = acc_scale * Eigen::Vector3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
  const Eigen::Vector3d angular_vel = ang_scale * Eigen::Vector3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

  bool valid_stamp;
  {
    std::lock_guard<std::mutex> lock(time_keeper_mutex);
    valid_stamp = time_keeper->validate_imu_stamp(imu_stamp);
  }
  if (!valid_stamp) {
    spdlog::warn("skip an invalid IMU data (stamp={})", imu_stamp);
    return;
  }

  odometry_estimation->insert_imu(imu_stamp, linear_acc, angular_vel);
  if (sub_mapping) {
    sub_mapping->insert_imu(imu_stamp, linear_acc, angular_vel);
  }
  if (global_mapping) {
    global_mapping->insert_imu(imu_stamp, linear_acc, angular_vel);
  }
}

#ifdef BUILD_WITH_CV_BRIDGE
void GlimROS::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
  spdlog::trace("image: {}.{}", msg->header.stamp.sec, msg->header.stamp.nanosec);
  {
    std::lock_guard<std::mutex> lock(config_mutex);
    if (!GlobalConfig::instance()->has_param("meta", "image_frame")) {
      spdlog::debug("auto-detecting image frame ID: {}", msg->header.frame_id);
      GlobalConfig::instance()->override_param<std::string>("meta", "image_frame", msg->header.frame_id);
    }
  }

  auto cv_image = cv_bridge::toCvCopy(msg, "bgr8");

  const double stamp = msg->header.stamp.sec + msg->header.stamp.nanosec / 1e9;
  odometry_estimation->insert_image(stamp, cv_image->image);
  if (sub_mapping) {
    sub_mapping->insert_image(stamp, cv_image->image);
  }
  if (global_mapping) {
    global_mapping->insert_image(stamp, cv_image->image);
  }
}
#endif

size_t GlimROS::points_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
  spdlog::trace("points: {}.{}", msg->header.stamp.sec, msg->header.stamp.nanosec);
  num_points++;
  last_points_time = std::chrono::steady_clock::now();
  if (!lidar_frame_detected) {
    std::lock_guard<std::mutex> lock(config_mutex);
    if (!GlobalConfig::instance()->has_param("meta", "lidar_frame_id")) {
      spdlog::debug("auto-detecting LiDAR frame ID: {}", msg->header.frame_id);
      GlobalConfig::instance()->override_param<std::string>("meta", "lidar_frame_id", msg->header.frame_id);
    }
    lidar_frame_detected = true;
  }

  auto raw_points = glim::extract_raw_points(*msg, intensity_field, ring_field);
  if (raw_points == nullptr) {
    spdlog::warn("failed to extract points from message");
    return 0;
  }

  raw_points->stamp += points_time_offset;
  bool valid_points;
  {
    std::lock_guard<std::mutex> lock(time_keeper_mutex);
    valid_points = time_keeper->process(raw_points);
  }
  if (!valid_points) {
    num_points_rejected++;
    spdlog::warn("skip an invalid point cloud (stamp={})", raw_points->stamp);
    return 0;
  }
  auto preprocessed = preprocessor->preprocess(raw_points);

  if (keep_raw_points) {
    // note: Raw points are used only in extension modules for visualization purposes.
    //       If you need to reduce the memory footprint, you can safely comment out the following line.
    preprocessed->raw_points = raw_points;
  }

  odometry_estimation->insert_frame(preprocessed);

  const size_t workload = odometry_estimation->workload();
  last_workload = workload;
  spdlog::debug("workload={}", workload);

  return workload;
}

bool GlimROS::needs_wait() {
  for (const auto& ext_module : extension_modules) {
    if (ext_module->needs_wait()) {
      return true;
    }
  }

  return false;
}

void GlimROS::timer_callback() {
  for (const auto& ext_module : extension_modules) {
    if (ext_module->ok()) {
      continue;
    }
    if (shutdown_on_extension_exit) {
      spdlog::info("an extension module requested termination; shutting down");
      rclcpp::shutdown();
      return;
    }
    // e.g., a GUI viewer window was closed: keep SLAM running
    if (exited_extension_modules.insert(ext_module.get()).second) {
      spdlog::warn("an extension module stopped (e.g., viewer closed); SLAM keeps running (shutdown_on_extension_exit=false)");
    }
  }

  std::vector<glim::EstimationFrame::ConstPtr> estimation_frames;
  std::vector<glim::EstimationFrame::ConstPtr> marginalized_frames;
  odometry_estimation->get_results(estimation_frames, marginalized_frames);
  if (!estimation_frames.empty()) {
    num_odom += estimation_frames.size();
    last_odom_time = std::chrono::steady_clock::now();
  }

  if (sub_mapping) {
    for (const auto& frame : marginalized_frames) {
      sub_mapping->insert_frame(frame);
    }

    auto submaps = sub_mapping->get_results();
    if (global_mapping) {
      for (const auto& submap : submaps) {
        global_mapping->insert_submap(submap);
      }
    }
  }
}

void GlimROS::wait(bool auto_quit) {
  spdlog::info("waiting for odometry estimation");
  odometry_estimation->join();

  if (sub_mapping) {
    std::vector<glim::EstimationFrame::ConstPtr> estimation_results;
    std::vector<glim::EstimationFrame::ConstPtr> marginalized_frames;
    odometry_estimation->get_results(estimation_results, marginalized_frames);
    for (const auto& marginalized_frame : marginalized_frames) {
      sub_mapping->insert_frame(marginalized_frame);
    }

    spdlog::info("waiting for local mapping");
    sub_mapping->join();

    const auto submaps = sub_mapping->get_results();
    if (global_mapping) {
      for (const auto& submap : submaps) {
        global_mapping->insert_submap(submap);
      }
      spdlog::info("waiting for global mapping");
      global_mapping->join();
    }
  }

  if (!auto_quit) {
    bool terminate = false;
    while (!terminate && rclcpp::ok()) {
      for (const auto& ext_module : extension_modules) {
        terminate |= (!ext_module->ok());
      }
    }
  }
}

void GlimROS::diagnostics_callback(diagnostic_updater::DiagnosticStatusWrapper& stat) {
  const auto now = std::chrono::steady_clock::now();
  const double dt = std::chrono::duration<double>(now - last_diag_time).count();
  last_diag_time = now;
  const auto age = [&](const std::chrono::steady_clock::time_point& t) { return std::chrono::duration<double>(now - t).count(); };

  const size_t imu = num_imu.exchange(0), points = num_points.exchange(0), rejected = num_points_rejected.exchange(0), odom = num_odom.exchange(0);
  stat.add("imu_rate [Hz]", dt > 0 ? imu / dt : 0.0);
  stat.add("points_rate [Hz]", dt > 0 ? points / dt : 0.0);
  stat.add("odometry_rate [Hz]", dt > 0 ? odom / dt : 0.0);
  stat.add("points_rejected", rejected);
  stat.add("odometry_workload", static_cast<size_t>(last_workload));
  stat.add("time_since_last_odometry [s]", age(last_odom_time));

  if (age(last_points_time) > 2.0) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "no point clouds received");
  } else if (age(last_odom_time) > 3.0) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "point clouds arriving but no odometry output");
  } else if (age(last_imu_time) > 1.0) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "no IMU data received");
  } else if (last_workload > 10) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "odometry is falling behind");
  } else {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "running");
  }
}

void GlimROS::save(const std::string& path) {
  if (global_mapping) {
    global_mapping->save(path);
  }
  for (auto& module : extension_modules) {
    module->at_exit(path);
  }
}

}  // namespace glim

RCLCPP_COMPONENTS_REGISTER_NODE(glim::GlimROS);