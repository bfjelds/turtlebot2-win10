cmake_minimum_required(VERSION 3.5)

project(avoid_obstacles)

find_package(ament_cmake REQUIRED)

find_package(rcl REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(kobuki_msgs REQUIRED)

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
  ${geometry_msgs_INCLUDE_DIRS}
  ${kobuki_msgs_INCLUDE_DIRS}
)

set(${PROJECT_NAME}_node_SRCS
)

add_executable(${PROJECT_NAME}_node 
  ../avoid_obstacles_node/src/avoid_obstacles_node.cpp
  ${${PROJECT_NAME}_node_SRCS}
)
target_link_libraries(${PROJECT_NAME}_node 
  ${std_msgs_LIBRARIES}
  ${rcl_LIBRARIES}
  ${rclcpp_LIBRARIES}
  ${rcutils_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  ${kobuki_msgs_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}_node DESTINATION bin)
ament_export_libraries(${PROJECT_NAME}_node)
ament_package()
