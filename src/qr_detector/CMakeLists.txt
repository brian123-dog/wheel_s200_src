cmake_minimum_required(VERSION 3.8)
project(qr_detector)

# C++17 标准 (std::filesystem 等)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 编译选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

# ---- 查找依赖 ----
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# BT.CPP v3 (ROS2 Humble 默认安装版本)
find_package(behaviortree_cpp_v3 REQUIRED)

find_package(nlohmann_json REQUIRED)

# ZBar 通过 pkg-config 查找
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZBAR REQUIRED zbar)

# ---- 编译共享库 (BT 插件需要动态注册，必须 SHARED) ----
add_library(${PROJECT_NAME} SHARED
  src/scan_qr_code.cpp
  src/qr_detector_plugins.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${ZBAR_INCLUDE_DIRS}
)

# 链接 ZBar + zlib（OpenCV 已由 cv_bridge 间接链接，此处不重复）
target_link_libraries(${PROJECT_NAME}
  ${ZBAR_LIBRARIES}
  nlohmann_json::nlohmann_json
  -lz
)

# ROS2 依赖通过 ament_target_dependencies 处理
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
  cv_bridge
  behaviortree_cpp_v3
)

# ---- 测试可执行文件 (独立程序，不依赖 BT.CPP) ----
add_executable(test_qr_main src/test_qr_main.cpp)
target_link_libraries(test_qr_main ${OpenCV_LIBS} ${ZBAR_LIBRARIES} nlohmann_json::nlohmann_json -lz)
target_include_directories(test_qr_main PRIVATE ${ZBAR_INCLUDE_DIRS})
ament_target_dependencies(test_qr_main
  rclcpp
  sensor_msgs
  cv_bridge
)

# ---- 安装 ----
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# test_qr_main 必须装在 lib/${PROJECT_NAME} 才能被 ros2 run 发现
install(TARGETS test_qr_main
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(FILES resource/qr_detector DESTINATION share/ament_index/resource_index/packages)

# ---- 导出非 ROS2 依赖（供其他包使用 qr_detector 头文件时能找到 nlohmann_json） ----
ament_export_dependencies(
  nlohmann_json
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
