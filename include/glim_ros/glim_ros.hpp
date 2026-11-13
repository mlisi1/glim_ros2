#pragma once

#include <any>
#include <deque>
#include <memory>
#include <unordered_set>
#include <atomic>
#include <thread>
#include <mutex>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>

#include <std_srvs/srv/trigger.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#ifdef BUILD_WITH_CV_BRIDGE
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>
#endif

namespace glim {
class TimeKeeper;
class CloudPreprocessor;
class AsyncOdometryEstimation;
class AsyncSubMapping;
class AsyncGlobalMapping;

class ExtensionModule;
class GenericTopicSubscription;

class GlimROS : public rclcpp::Node {
public:
  GlimROS(const rclcpp::NodeOptions& options);
  ~GlimROS();

  bool needs_wait();
  void timer_callback();

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
#ifdef BUILD_WITH_CV_BRIDGE
  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
#endif
  size_t points_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

  void diagnostics_callback(diagnostic_updater::DiagnosticStatusWrapper& stat);

  void wait(bool auto_quit = false);
  void save(const std::string& path);

  const std::vector<std::shared_ptr<GenericTopicSubscription>>& extension_subscriptions();

private:
  std::unique_ptr<glim::TimeKeeper> time_keeper;
  std::unique_ptr<glim::CloudPreprocessor> preprocessor;

  std::shared_ptr<glim::AsyncOdometryEstimation> odometry_estimation;
  std::unique_ptr<glim::AsyncSubMapping> sub_mapping;
  std::unique_ptr<glim::AsyncGlobalMapping> global_mapping;

  bool keep_raw_points;
  double imu_time_offset;
  double points_time_offset;
  double acc_scale;
  double ang_scale;
  bool dump_on_unload;
  bool shutdown_on_extension_exit;
  std::unordered_set<const ExtensionModule*> exited_extension_modules;

  std::string intensity_field, ring_field;

  // Extension modulles
  std::vector<std::shared_ptr<ExtensionModule>> extension_modules;
  std::vector<std::shared_ptr<GenericTopicSubscription>> extension_subs;

  // Thread safety (IMU / points / timer callbacks may run concurrently)
  std::mutex time_keeper_mutex;
  std::mutex config_mutex;
  std::atomic_bool imu_frame_detected{false};
  std::atomic_bool lidar_frame_detected{false};

  // Diagnostics
  std::unique_ptr<diagnostic_updater::Updater> diagnostics;
  std::atomic<size_t> num_imu{0}, num_points{0}, num_points_rejected{0}, num_odom{0}, last_workload{0};
  std::atomic<std::chrono::steady_clock::time_point> last_imu_time, last_points_time, last_odom_time;
  std::chrono::steady_clock::time_point last_diag_time;

  // ROS-related
  rclcpp::CallbackGroup::SharedPtr imu_callback_group;
  rclcpp::CallbackGroup::SharedPtr points_callback_group;
  rclcpp::CallbackGroup::SharedPtr timer_callback_group;
  rclcpp::TimerBase::SharedPtr timer;

  // ~/save_map: saves GLIM dump + map.pcd + map.pgm/yaml in the background
  std::string map_save_directory;
  std::atomic_bool saving;
  std::thread save_thread;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_map_srv;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_sub;
#ifdef BUILD_WITH_CV_BRIDGE
  image_transport::Subscriber image_sub;
#endif
};

}  // namespace glim
