cmake_minimum_required(VERSION 3.5)
project(precision_landing)

# Default to C99
if(NOT CMAKE_C_STANDARD)
	set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(custom_msgs REQUIRED)

#################
# Setup targets #
#################

include_directories(include)

# examples/listeners/sensor_combined_listener
#add_executable(sensor_combined_listener src/examples/listeners/sensor_combined_listener.cpp)
#ament_target_dependencies(sensor_combined_listener rclcpp px4_msgs)
#install(TARGETS sensor_combined_listener DESTINATION lib/${PROJECT_NAME})

add_executable(flight_controller src/flight_controller.cpp)
ament_target_dependencies(flight_controller rclcpp px4_msgs sensor_msgs custom_msgs)
install(TARGETS flight_controller DESTINATION lib/${PROJECT_NAME})

############
# Install ##
############

# Export information to downstream packages
ament_export_dependencies(ament_cmake rclcpp rosidl_default_runtime eigen3_cmake_module Eigen3 px4_msgs geometry_msgs sensor_msgs custom_msgs)

ament_export_include_directories(include)


# Install header files
install(DIRECTORY include/ DESTINATION include/)

# Install launch files.
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

###########
# Python ##
###########

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/camera.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
