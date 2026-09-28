cmake_minimum_required(VERSION 3.8)
project(multi_action_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(base_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Add include directory
include_directories(
  include
  ${behaviortree_cpp_v3_INCLUDE_DIRS}
)

# 新增内部逻辑库
add_library(arm_task_core STATIC
  src/arm_motion_executor.cpp
  src/arm_task_orchestrator.cpp
)

# 给内部逻辑库补依赖
ament_target_dependencies(arm_task_core
  rclcpp
  rclcpp_action
  geometry_msgs
  moveit_msgs
  shape_msgs
  std_msgs
  tf2_ros
  base_interfaces
)

# Create main library (BT plugin nodes)
add_library(${PROJECT_NAME} SHARED
  src/multi_action_node.cpp
  src/nav_arm_action_bt_node.cpp
  src/align_to_wall_action_bt_node.cpp
)

target_compile_definitions(${PROJECT_NAME} PRIVATE BT_PLUGIN_EXPORT)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_action
  base_interfaces
  behaviortree_cpp_v3
  nav2_behavior_tree
  geometry_msgs
)

# 新增 Action Server 可执行文件
add_executable(nav_arm_action_server
  src/nav_arm_action_server.cpp
)

target_link_libraries(nav_arm_action_server arm_task_core)

# 给可执行文件补依赖
ament_target_dependencies(nav_arm_action_server
  rclcpp
  rclcpp_action
  base_interfaces
)

# Install library
install(TARGETS
  ${PROJECT_NAME}
  arm_task_core
  nav_arm_action_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include/
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

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME} arm_task_core)
ament_export_dependencies(
  rclcpp
  rclcpp_action
  behaviortree_cpp_v3
  base_interfaces
  geometry_msgs
  moveit_msgs
  shape_msgs
  std_msgs
  tf2_ros
)

ament_package()