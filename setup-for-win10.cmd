@echo off

set TURTLEBOT2_WIN10_PATH=%~dp0
set ROS_PATH=%TURTLEBOT2_WIN10_PATH%..\ros
set ROS2_PATH=%TURTLEBOT2_WIN10_PATH%..\ros2


echo Modifying the AMENT_IGNORE in projects that Turtlebot2 doesn't need: %~1"

call:AddOrRemoveAmentIgnore %~1 %TURTLEBOT2_WIN10_PATH%\ecl_core
call:AddOrRemoveAmentIgnore %~1 %TURTLEBOT2_WIN10_PATH%\ecl_lite
call:AddOrRemoveAmentIgnore %~1 %TURTLEBOT2_WIN10_PATH%\ecl_navigation
call:AddOrRemoveAmentIgnore %~1 %TURTLEBOT2_WIN10_PATH%\kobuki
call:AddOrRemoveAmentIgnore %~1 %TURTLEBOT2_WIN10_PATH%\kobuki_core
call:AddOrRemoveAmentIgnore %~1 %TURTLEBOT2_WIN10_PATH%\kobuki_msgs
call:AddOrRemoveAmentIgnore %~1 %TURTLEBOT2_WIN10_PATH%\sophus
call:AddOrRemoveAmentIgnore %~1 %TURTLEBOT2_WIN10_PATH%\navigation\amcl

call:AddOrRemoveAmentIgnore %~1 %ROS_PATH%\class_loader
rem call:AddOrRemoveAmentIgnore %~1 %ROS_PATH%\console_bridge

call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\poco_vendor
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\common_interfaces\actionlib_msgs
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\common_interfaces\diagnostic_msgs
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\common_interfaces\shape_msgs
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\common_interfaces\std_srvs
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\common_interfaces\stereo_msgs
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\common_interfaces\trajectory_msgs
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\common_interfaces\visualization_msgs
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\demos\composition
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\demos\demo_nodes_cpp
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\demos\demo_nodes_cpp_native
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\demos\demo_nodes_py
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\demos\dummy_robot\dummy_map_server
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\demos\dummy_robot\dummy_robot_bringup
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\demos\dummy_robot\dummy_sensors
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\demos\image_tools
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\demos\intra_process_demo
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\demos\lifecycle
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\demos\pendulum_control
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\demos\pendulum_msgs
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\demos\topic_monitor
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\examples
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\examples\rclcpp\minimal_client
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\examples\rclcpp\minimal_composition
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\examples\rclcpp\minimal_publisher
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\examples\rclcpp\minimal_service
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\examples\rclcpp\minimal_subscriber
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\examples\rclcpp\minimal_timer
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\example_interfaces
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\geometry2\geometry2
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\geometry2\test_tf2
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\geometry2\tf2_eigen
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\geometry2\tf2_geometry_msgs
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\orocos_kinematics_dynamics\orocos_kdl
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\orocos_kinematics_dynamics\orocos_kdl\doc
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\orocos_kinematics_dynamics\orocos_kdl\examples
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\orocos_kinematics_dynamics\orocos_kdl\models
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\orocos_kinematics_dynamics\orocos_kdl\src
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\orocos_kinematics_dynamics\orocos_kdl\tests
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\realtime_support\rttest
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\realtime_support\rttest\examples
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\realtime_support\tlsf_cpp
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\robot_model\kdl_parser
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\robot_model\urdf
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\robot_state_publisher
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\ros1_bridge
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\ros2cli\ros2cli
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\ros2cli\ros2msg
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\ros2cli\ros2node
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\ros2cli\ros2pkg
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\ros2cli\ros2run
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\ros2cli\ros2srv
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\sros2
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\system_tests\test_communication
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\system_tests\test_rclcpp
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\tlsf\tlsf
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\urdfdom
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\urdfdom\urdf_parser
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\urdfdom\urdf_parser\test
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\urdfdom\urdf_parser\test\gtest
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\urdfdom_headers
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\urdfdom_headers\urdf_exception
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\urdfdom_headers\urdf_model
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\urdfdom_headers\urdf_model_state
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\urdfdom_headers\urdf_sensor
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\urdfdom_headers\urdf_world
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\vision_opencv\cv_bridge\python
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\vision_opencv\cv_bridge\src
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\vision_opencv\cv_bridge\test
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\vision_opencv\image_geometry
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\vision_opencv\image_geometry\test



goto:EOF


rem
rem Other projects to consider ignoring...
rem
call:AddOrRemoveAmentIgnore %~1 %ROS_PATH%\console_bridge
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\rcl\rcl
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\rcl\rcl\test
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\rcl\rcl\test\memory_tools
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\rcl\rcl_lifecycle
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\rclcpp\rclcpp
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\rclcpp\rclcpp_lifecycle
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\rclpy\rclpy
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\rcl_interfaces\lifecycle_msgs
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\rcl_interfaces\rcl_interfaces
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\rcl_interfaces\test_msgs
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\rcutils
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\rcutils\test\memory_tools
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\rmw\rmw
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\rmw\rmw\test
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\rmw\rmw_implementation_cmake
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\rmw_connext\connext_cmake_module
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\rmw_connext\rmw_connext_cpp
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\rmw_connext\rmw_connext_dynamic_cpp
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\rmw_connext\rmw_connext_shared_cpp
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\rmw_connext\rosidl_typesupport_connext_c
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\rmw_connext\rosidl_typesupport_connext_cpp
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\rmw_fastrtps\fastrtps_cmake_module
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\rmw_fastrtps\rmw_fastrtps_cpp
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\rmw_implementation\rmw_implementation
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\rmw_opensplice\opensplice_cmake_module
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\rmw_opensplice\rmw_opensplice_cpp
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\rmw_opensplice\rosidl_typesupport_opensplice_c
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\rmw_opensplice\rosidl_typesupport_opensplice_cpp
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\ros1_bridge
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\rosidl\python_cmake_module
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\rosidl\rosidl_cmake
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\rosidl\rosidl_generator_c
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\rosidl\rosidl_generator_cpp
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\rosidl\rosidl_generator_py
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\rosidl\rosidl_parser
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\rosidl\rosidl_typesupport_interface
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\rosidl\rosidl_typesupport_introspection_c
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\rosidl\rosidl_typesupport_introspection_cpp
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\rosidl_dds\rosidl_generator_dds_idl
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\rosidl_typesupport\rosidl_default_generators
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\rosidl_typesupport\rosidl_default_runtime
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\rosidl_typesupport\rosidl_typesupport_c
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\rosidl_typesupport\rosidl_typesupport_cpp
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\tinyxml_vendor
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\tlsf\tlsf


:AddOrRemoveAmentIgnore 

echo In AddOrRemoveAmentIgnore %~1 %~2
set IGNORE_FILE=%~2\AMENT_IGNORE

if "%~1" == "filter" (

    if EXIST "%IGNORE_FILE%" ( 
        echo %IGNORE_FILE% already exists!!
        goto:EOF
    )
    
    type NUL > %IGNORE_FILE%
    goto:EOF

)
if "%~1" == "reset" (

    del /Q %IGNORE_FILE%
    
)

goto:EOF
