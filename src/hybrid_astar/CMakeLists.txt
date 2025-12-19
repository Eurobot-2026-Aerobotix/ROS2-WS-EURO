cmake_minimum_required(VERSION 3.8)
project(hybrid_astar)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate messages and services
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Obstacle.msg"
  "msg/ObstacleArray.msg"
  "msg/ObstacleIDArray.msg"
  "msg/ColorCommand.msg"
  "msg/BatchMember.msg"
  "msg/Batch.msg"
  "msg/Track.msg"
  "msg/ArucoMapperState.msg"
  "srv/ComputePath.srv"
  "srv/GetBatchColorPattern.srv"
  DEPENDENCIES std_msgs geometry_msgs nav_msgs builtin_interfaces
)

# Install Python modules
install(DIRECTORY hybrid_astar/
  DESTINATION lib/python3.12/site-packages/${PROJECT_NAME}
  PATTERN "*.pyc" EXCLUDE
  PATTERN "__pycache__" EXCLUDE
)

# Install Python executables
install(PROGRAMS
  hybrid_astar/hybrid_astar_node.py
  hybrid_astar/json_to_ros.py
  hybrid_astar/yaml_to_ros.py
  hybrid_astar/generate_path.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install config and launch files
install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
