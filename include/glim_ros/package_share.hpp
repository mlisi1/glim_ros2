#pragma once

#include <string>

// get_package_share_directory() is deprecated on newer distros (Lyrical) in favor of get_package_share_path(),
// which older ones (Humble) do not have.
#if __has_include(<ament_index_cpp/get_package_share_path.hpp>)
#include <ament_index_cpp/get_package_share_path.hpp>
#else
#include <ament_index_cpp/get_package_share_directory.hpp>
#endif

namespace glim {

/// @brief Share directory of a ROS package (throws ament_index_cpp::PackageNotFoundError if not found)
inline std::string package_share_directory(const std::string& package_name) {
#if __has_include(<ament_index_cpp/get_package_share_path.hpp>)
  return ament_index_cpp::get_package_share_path(package_name).string();
#else
  return ament_index_cpp::get_package_share_directory(package_name);
#endif
}

}  // namespace glim
