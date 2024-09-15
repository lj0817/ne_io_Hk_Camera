cmake_minimum_required(VERSION 3.10)
project(hikcamera_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 设置政策
cmake_policy(SET CMP0012 NEW)  # 处理布尔常量和数字
cmake_policy(SET CMP0057 NEW)  # 支持 IN_LIST 操作符


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

include_directories(
    include
    /opt/MVS/include
    ${OpenCV_INCLUDE_DIRS}
)

set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_INSTALL_RPATH /opt/MVS/lib/64)


add_executable(camera_node src/hik_camera_io.cpp src/hik_camera_node.cpp)

target_link_libraries(
    camera_node
    /opt/MVS/lib/64/libMvCameraControl.so
    ${OpenCV_LIBS}
)

ament_target_dependencies(camera_node rclcpp sensor_msgs std_msgs cv_bridge image_transport)

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

install(TARGETS
  camera_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()