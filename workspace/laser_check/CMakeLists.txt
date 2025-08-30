cmake_minimum_required(VERSION 3.5)
project(laser_check)

# C++ settings:
set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -g)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
 
add_executable(laser_data laser_script.cpp)
ament_target_dependencies(laser_data rclcpp sensor_msgs tf2)

install(TARGETS
  laser_data
  DESTINATION lib/${PROJECT_NAME})

ament_package()