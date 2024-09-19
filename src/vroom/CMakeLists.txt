# Specify the minimum version of CMake required
cmake_minimum_required(VERSION 3.22)

# Define the project name and version
project(vroom VERSION 1.0)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros_pkgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)

set(dependencies
  "geometry_msgs"
  "nav_msgs"
  "rclcpp"
  "sensor_msgs"
  "tf2"
)

# Add source files
set(source_
    "src/main.cpp"
    "src/lidar.cpp"
    "src/motordrive.cpp"
    "src/CameraReader.cpp"
    )

# Include directories
include_directories(include)

# Add executable
add_executable(mantis ${source_})
ament_target_dependencies(mantis ${dependencies})

# Install target
install(TARGETS
  mantis
  DESTINATION lib/${PROJECT_NAME})

# Handle testing and linting
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Mark the project as an ament package
ament_package()
