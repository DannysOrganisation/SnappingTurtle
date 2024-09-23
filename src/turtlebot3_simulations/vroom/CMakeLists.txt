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
find_package(visualization_msgs REQUIRED)

set(dependencies
  "geometry_msgs"
  "nav_msgs"
  "rclcpp"
  "visualization_msgs"
  "sensor_msgs"
  "tf2"
)

set(source_
    "src/main.cpp"
    "src/lidar.cpp"
    "src/motordrive.cpp"
    "src/odom.cpp"
    "src/camera.cpp"
    "src/fsm.cpp"
    )

# Include directories
include_directories(include)

add_executable(lidar src/lidar.cpp)
add_executable(odom src/odom.cpp)
add_executable(motordrive src/motordrive.cpp)
add_executable(fsm src/fsm.cpp src/lidar.cpp src/odom.cpp)
add_executable(camerareader src/camera.cpp)

target_compile_definitions(odom PRIVATE ODOM_MAIN)
target_compile_definitions(lidar PRIVATE LIDAR_MAIN)
target_compile_definitions(fsm PRIVATE FSM_MAIN)


ament_target_dependencies(lidar ${dependencies})
ament_target_dependencies(odom ${dependencies})
ament_target_dependencies(motordrive ${dependencies})
ament_target_dependencies(fsm ${dependencies})
ament_target_dependencies(camerareader ${dependencies})

install(TARGETS
  lidar
  odom
  motordrive
  fsm
  camerareader
  DESTINATION lib/${PROJECT_NAME})

# Handle testing and linting
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  
  
  # The following line skips the linter which checks for copyrights
  # Comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # The following line skips cpplint (only works in a git repo)
  # Comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
