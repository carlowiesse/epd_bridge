cmake_minimum_required(VERSION 3.5)
project(robot_control_interface_msgs)

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
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(visualization_msgs REQUIRED)

rosidl_generate_interfaces( ${PROJECT_NAME}
  "msg/SystemMode.msg"
  "msg/SafetyMode.msg"
  "msg/Object.msg"
  "msg/Box.msg"
  "msg/GraspExecutionOptions.msg"
  "srv/ChangeMode.srv"
  "srv/CheckOn.srv"
  "srv/InitNode.srv"
  "srv/PowerOff.srv"
  "srv/ResetNode.srv"
  "srv/SafetyInterrupt.srv"
  "srv/StopNode.srv"
  "srv/RobotCommandGrasp.srv"
  "srv/GetPose.srv"
  "srv/DetectObjects.srv"
  "srv/DetectBox.srv"
  "srv/Grasp.srv"
  "srv/RemoveObject.srv"
  "action/RobotCommandDetectBox.action"
  "action/RobotCommandDetectObject.action"
  "action/RobotCommandMove.action"
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
  shape_msgs
  visualization_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
