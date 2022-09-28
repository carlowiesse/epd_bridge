cmake_minimum_required(VERSION 3.5)
project(epd_bridge)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

find_package(std_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(emd_msgs REQUIRED)
find_package(epd_msgs REQUIRED)
find_package(robot_control_interface_msgs REQUIRED)

# add header files
include_directories(include/${PROJECT_NAME})

add_definitions(-DPATH_TO_PKG="${CMAKE_CURRENT_LIST_DIR}")

# add executables
add_executable(box_vision src/box_node.cpp src/utils.cpp)
ament_target_dependencies(box_vision
  rclcpp
  OpenCV
  cv_bridge
  std_msgs
  shape_msgs
  geometry_msgs
  emd_msgs
  epd_msgs
  robot_control_interface_msgs
)

add_executable(label_vision src/label_node.cpp src/utils.cpp)
ament_target_dependencies(label_vision
  rclcpp
  OpenCV
  cv_bridge
  std_msgs
  shape_msgs
  geometry_msgs
  emd_msgs
  epd_msgs
  robot_control_interface_msgs
)

add_executable(tray_vision src/tray_node.cpp src/utils.cpp)
ament_target_dependencies(tray_vision
  rclcpp
  OpenCV
  cv_bridge
  std_msgs
  shape_msgs
  geometry_msgs
  emd_msgs
  epd_msgs
  robot_control_interface_msgs
)

add_executable(human_vision src/human_node.cpp src/utils.cpp)
ament_target_dependencies(human_vision
  rclcpp
  OpenCV
  cv_bridge
  std_msgs
  shape_msgs
  geometry_msgs
  emd_msgs
  epd_msgs
  robot_control_interface_msgs
)

add_executable(box_tray_vision src/box_tray_node.cpp src/utils.cpp)
ament_target_dependencies(box_tray_vision
  rclcpp
  OpenCV
  cv_bridge
  std_msgs
  shape_msgs
  geometry_msgs
  emd_msgs
  epd_msgs
  robot_control_interface_msgs
)

install(TARGETS
  box_vision
  label_vision
  tray_vision
  human_vision
  box_tray_vision
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
