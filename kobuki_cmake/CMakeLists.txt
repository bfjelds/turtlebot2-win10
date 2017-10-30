cmake_minimum_required(VERSION 3.5)

project(kobuki)

find_package(ament_cmake REQUIRED)

find_package(rcl REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # we dont use add_compile_options with pedantic in message packages
  # because the Python C extensions dont comply with it
  # TODO(mikaelarguedas) change to add_compile_options
  # once this is not a message package anymore
  # https://github.com/ros2/system_tests/issues/191
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

include_directories(include
  ${rcl_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${rcutils_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ../ecl_core/ecl_eigen/include/eigen3.1.2
  ../sophus
  ../ecl_core/ecl_devices/include
  ../ecl_lite/ecl_time_lite/include
  ../ecl_navigation/ecl_mobile_robot/include
  ../ecl_core/ecl_time/include
  ../ecl_core/ecl_threads/include
  ../ecl_core/ecl_sigslots/include
  ../ecl_core/ecl_utilities/include
  ../ecl_core/ecl_concepts/include
  ../ecl_core/ecl_containers/include
  ../ecl_core/ecl_converters/include
  ../ecl_lite/ecl_errors/include
  ../ecl_core/ecl_exceptions/include
  ../ecl_core/ecl_formatters/include
  ../ecl_core/ecl_math/include
  ../ecl_core/ecl_mpl/include
  ../ecl_core/ecl_type_traits/include
  ../ecl_core/ecl_linear_algebra/include
  ../ecl_lite/ecl_config/include
  ../ecl_core/ecl_geometry/include
  ../kobuki_core/kobuki_driver/include
)

set(${PROJECT_NAME}_SRCS
  ../ecl_core/ecl_devices/src/lib/serial_w32.cpp
  ../ecl_core/ecl_exceptions/src/lib/standard_exception.cpp
  ../ecl_core/ecl_geometry/src/lib/angle.cpp
  ../ecl_core/ecl_threads/src/lib/mutex_w32.cpp
  ../ecl_core/ecl_threads/src/lib/thread_win.cpp
  ../ecl_core/ecl_time/src/lib/sleep_win.cpp
  ../ecl_core/ecl_time/src/lib/timestamp_base.cpp
  ../ecl_core/ecl_time/src/lib/timestamp_win.cpp
  ../ecl_lite/ecl_time_lite/src/lib/functions_win.cpp
  ../ecl_navigation/ecl_mobile_robot/src/lib/kinematics/differential_drive.cpp
  ../kobuki_core/kobuki_driver/src/driver/battery.cpp
  ../kobuki_core/kobuki_driver/src/driver/command.cpp
  ../kobuki_core/kobuki_driver/src/driver/core_sensors.cpp
  ../kobuki_core/kobuki_driver/src/driver/diff_drive.cpp
  ../kobuki_core/kobuki_driver/src/driver/event_manager.cpp
  ../kobuki_core/kobuki_driver/src/driver/kobuki.cpp
  ../kobuki_core/kobuki_driver/src/driver/packet_finder.cpp
)

add_executable(${PROJECT_NAME}_node 
  ../turtlebot2_demo/turtlebot2_drivers/src/kobuki_node.cpp
  ${${PROJECT_NAME}_SRCS}
)
target_link_libraries(${PROJECT_NAME}_node 
  ${std_msgs_LIBRARIES}
  ${rcl_LIBRARIES}
  ${rclcpp_LIBRARIES}
  ${rcutils_LIBRARIES}
  ${tf2_LIBRARIES}
  ${tf2_ros_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${nav_msgs_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION bin)

ament_export_libraries(${PROJECT_NAME}_node)
ament_package()
