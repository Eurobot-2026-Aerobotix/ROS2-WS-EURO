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
find_package(aruco_mapper_msgs REQUIRED)

# Generate messages and services
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Obstacle.msg"
  "msg/ObstacleArray.msg"
  "msg/ObstacleIDArray.msg"
  "msg/ColorCommand.msg"
  "srv/ComputePath.srv"
  "srv/GetBatchColorPattern.srv"
  "srv/DeleteBatch.srv"
  DEPENDENCIES std_msgs geometry_msgs nav_msgs builtin_interfaces aruco_mapper_msgs
)

# This macro ensures that Python scripts can import the generated messages
ament_export_dependencies(rosidl_default_runtime)

# Install Python source modules into the package's site-packages
install(DIRECTORY hybrid_astar/
  DESTINATION lib/python3.12/site-packages/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py"
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

# Install Python executables to standard ROS2 location
# Note: Using FILES instead of PROGRAMS to ensure they are copied, not symlinked
install(FILES
  hybrid_astar/hybrid_astar_node.py
  hybrid_astar/json_to_ros.py
  hybrid_astar/yaml_to_ros.py
  hybrid_astar/generate_path.py
  hybrid_astar/web_handler.py
  DESTINATION lib/${PROJECT_NAME}
  PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
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
