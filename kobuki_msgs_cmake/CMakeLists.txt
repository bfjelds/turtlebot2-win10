cmake_minimum_required(VERSION 3.5)

project(kobuki_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # we dont use add_compile_options with pedantic in message packages
  # because the Python C extensions dont comply with it
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(kobuki_msgs
  "../kobuki_msgs/msg/BumperEvent.msg"
  "../kobuki_msgs/msg/ButtonEvent.msg"
  "../kobuki_msgs/msg/CliffEvent.msg"
#  "../kobuki_msgs/msg/ControllerInfo.msg"
  "../kobuki_msgs/msg/DigitalInputEvent.msg"
#  "../kobuki_msgs/msg/DigitalOutput.msg"
#  "../kobuki_msgs/msg/DockInfraRed.msg"
#  "../kobuki_msgs/msg/ExternalPower.msg"
#  "../kobuki_msgs/msg/KeyboardInput.msg"
  "../kobuki_msgs/msg/Led.msg"
#  "../kobuki_msgs/msg/MotorPower.msg"
  "../kobuki_msgs/msg/PowerSystemEvent.msg"
  "../kobuki_msgs/msg/RobotStateEvent.msg"
#  "../kobuki_msgs/msg/ScanAngle.msg"
#  "../kobuki_msgs/msg/SensorState.msg"
#  "../kobuki_msgs/msg/Sound.msg"
#  "../kobuki_msgs/msg/VersionInfo.msg"
  "../kobuki_msgs/msg/WheelDropEvent.msg"
  DEPENDENCIES builtin_interfaces
)

ament_package()
