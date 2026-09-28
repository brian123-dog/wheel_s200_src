cmake_minimum_required(VERSION 3.8)
project(waypoint_action_publisher)

# 查找所有需要的依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(base_interfaces REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# 如果你已安装系统级的 nlohmann-json3-dev，可以这样找到它：
find_package(nlohmann_json REQUIRED)

# 包含自定义头文件目录
include_directories(include)

# 第一个可执行文件：json_nav_action_publisher.cpp
add_executable(json_waypoint_navigator src/json_nav_action_publisher.cpp)
ament_target_dependencies(json_waypoint_navigator
  rclcpp
  rclcpp_action
  geometry_msgs
  nav2_msgs
  nav_msgs
  std_srvs
  base_interfaces
  ament_index_cpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  # 如果使用了 nlohmann_json，也可以添加（通常只需链接目标名）
  # nlohmann_json::nlohmann_json
)

# 第二个可执行文件：position_recorder.cpp
add_executable(position_recorder src/position_recorder.cpp)
ament_target_dependencies(position_recorder
  rclcpp
  rclcpp_action
  geometry_msgs
  nav2_msgs
  nav_msgs
  std_srvs
  base_interfaces
  ament_index_cpp
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# 安装可执行文件
install(TARGETS
  json_waypoint_navigator
  position_recorder
  DESTINATION lib/${PROJECT_NAME}
)

# 如有需要，安装自定义头文件
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# 测试设置（可选）
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# 最后打包
ament_package()