#include <iostream>
#include <spdlog/spdlog.h>
#include <rclcpp/rclcpp.hpp>

#include <glim_ros/glim_ros.hpp>
#include <glim/util/config.hpp>
#include <glim/util/extension_module_ros2.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto glim = std::make_shared<glim::GlimROS>(options);

  // IMU, points and the result-forwarding timer use separate callback groups
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 4);
  exec.add_node(glim);
  exec.spin();
  rclcpp::shutdown();

  const std::string dump_path = glim->get_parameter("dump_path").as_string();

  glim->wait();
  glim->save(dump_path);

  return 0;
}