@echo off

set TURTLEBOT2_WIN10_PATH=%~dp0
set ROS_PATH=%TURTLEBOT2_WIN10_PATH%..\ros
set ROS2_PATH=%TURTLEBOT2_WIN10_PATH%..\ros2
set ADV_PATH=%TURTLEBOT2_WIN10_PATH%..\advanced-win10


echo Modifying the AMENT_IGNORE in projects that Turtlebot2 doesn't need: ARG1="%~1" ARG2="%~2"
echo   ARG1, specify "filter" to add AMENT_IGNORE
echo   ARG1, specify "reset" to remove AMENT_IGNORE
echo   ARG2, specify "turtlebot2" to configure only TURTLEBOT2 nodes (kobuki, etc)
echo   ARG2, specify "advanced" to configure TURTLEBOT2 and ADVANCED nodes (laser_filters, etc)

call:AddOrRemoveAmentIgnore %~1 %TURTLEBOT2_WIN10_PATH%\eigen3

if "%~2" == "turtlebot2" (
call:AddOrRemoveAmentIgnore %~1 %ROS_PATH%\class_loader
call:AddOrRemoveAmentIgnore %~1 %ROS_PATH%\console_bridge
)

if "%~2" == "turtlebot2" (
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\poco_vendor
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\common_interfaces\actionlib_msgs
)
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\common_interfaces\diagnostic_msgs
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\common_interfaces\shape_msgs
if "%~2" == "turtlebot2" (
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\common_interfaces\std_srvs
)
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
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\geometry2
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\kdl_parser
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
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\rviz
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\sros2
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\system_tests\test_communication
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\system_tests\test_rclcpp
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\tlsf\tlsf
call:AddOrRemoveAmentIgnore %~1 %ROS2_PATH%\urdf
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
