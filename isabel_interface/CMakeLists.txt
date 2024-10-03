cmake_minimum_required(VERSION 3.5)
project(isabel_interface)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(bosdyn_api_msgs REQUIRED)
find_package(bosdyn_spot_api_msgs REQUIRED)
find_package(bosdyn_spot_cam_api_msgs REQUIRED)
find_package(common_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(custom_msg_srv REQUIRED)


# Include Cpp "include" directory
include_directories(include)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

set(dependencies
  rclcpp
  geometry_msgs
#  nav2_costmap_2d
#  pluginlib
#  nav_msgs
  std_msgs
#  nav2_util
#  nav2_core
  sensor_msgs
  tf2
  tf2_geometry_msgs
  custom_msg_srv
  pcl_ros
  pcl_conversions
  tf2_ros
  tf2_sensor_msgs
)

# Create Cpp executable
add_executable(cpp_executable src/pointcloud.cpp)
ament_target_dependencies(cpp_executable ${dependencies})

add_executable(2cpp_executable src/2_pointcloud.cpp)
ament_target_dependencies(2cpp_executable ${dependencies})

#set(msg_files
#	"msg/A.msg"
#)
#rosidl_generate_interfaces(${PROJECT_NAME}
#  "msg/Trajectory.msg"
#  DEPENDENCIES
#    bosdyn_api_msgs
#    bosdyn_spot_api_msgs
#    bosdyn_spot_cam_api_msgs
#    builtin_interfaces
#    common_interfaces
#    geometry_msgs
#    std_msgs
#    sensor_msgs
#    custom_msg_srv
#)
#ament_export_dependencies(rosidl_default_runtime)

# Install Cpp executables
install(TARGETS
  cpp_executable
  2cpp_executable
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
#ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/vel2.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
