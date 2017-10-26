@echo off


set TURTLEBOT2_WIN10_PATH=%~dp0
set ROS_PATH=%TURTLEBOT2_WIN10_PATH%..\ros
set ROS2_PATH=%TURTLEBOT2_WIN10_PATH%..\ros2

type NUL > %TURTLEBOT2_WIN10_PATH%\ecl_core\AMENT_IGNORE
type NUL > %TURTLEBOT2_WIN10_PATH%\ecl_lite\AMENT_IGNORE
type NUL > %TURTLEBOT2_WIN10_PATH%\ecl_navigation\AMENT_IGNORE
type NUL > %TURTLEBOT2_WIN10_PATH%\kobuki\AMENT_IGNORE
type NUL > %TURTLEBOT2_WIN10_PATH%\kobuki_core\AMENT_IGNORE
type NUL > %TURTLEBOT2_WIN10_PATH%\sophus\AMENT_IGNORE

type NUL > %ROS_PATH%\class_loader\AMENT_IGNORE
type NUL > %ROS_PATH%\console_bridge\AMENT_IGNORE
rem type NUL > %ROS2_PATH%\cartographer_ros\AMENT_IGNORE
rem type NUL > %ROS2_PATH%\cartographer_ros\cartographer_rviz\AMENT_IGNORE
rem type NUL > %ROS2_PATH%\cartographer_ros\docs\AMENT_IGNORE
rem type NUL > %ROS2_PATH%\demos\build\composition\AMENT_IGNORE
type NUL > %ROS2_PATH%\demos\composition\AMENT_IGNORE
type NUL > %ROS2_PATH%\demos\dummy_robot\AMENT_IGNORE
type NUL > %ROS2_PATH%\demos\image_tools\AMENT_IGNORE
type NUL > %ROS2_PATH%\demos\intra_process_demo\AMENT_IGNORE
type NUL > %ROS2_PATH%\demos\pendulum_control\AMENT_IGNORE
type NUL > %ROS2_PATH%\demos\pendulum_msgs\AMENT_IGNORE
type NUL > %ROS2_PATH%\demos\topic_monitor\AMENT_IGNORE
rem type NUL > %ROS2_PATH%\depthimage_to_laserscan\AMENT_IGNORE
type NUL > %ROS2_PATH%\examples\AMENT_IGNORE
type NUL > %ROS2_PATH%\geometry2\geometry2\AMENT_IGNORE
type NUL > %ROS2_PATH%\geometry2\geometry_experimental\AMENT_IGNORE
type NUL > %ROS2_PATH%\geometry2\test_tf2\AMENT_IGNORE
type NUL > %ROS2_PATH%\geometry2\tf2_bullet\AMENT_IGNORE
type NUL > %ROS2_PATH%\geometry2\tf2_kdl\AMENT_IGNORE
type NUL > %ROS2_PATH%\geometry2\tf2_py\AMENT_IGNORE
type NUL > %ROS2_PATH%\geometry2\tf2_sensor_msgs\AMENT_IGNORE
type NUL > %ROS2_PATH%\geometry2\tf2_tools\AMENT_IGNORE
rem type NUL > %ROS2_PATH%\joystick_drivers\AMENT_IGNORE
rem type NUL > %ROS2_PATH%\joystick_drivers\joystick_drivers\AMENT_IGNORE
rem type NUL > %ROS2_PATH%\joystick_drivers\ps3joy\AMENT_IGNORE
rem type NUL > %ROS2_PATH%\joystick_drivers\spacenav_node\AMENT_IGNORE
rem type NUL > %ROS2_PATH%\joystick_drivers\wiimote\AMENT_IGNORE
rem type NUL > %ROS2_PATH%\navigation\amcl\AMENT_IGNORE
rem type NUL > %ROS2_PATH%\navigation\base_local_planner\AMENT_IGNORE
rem type NUL > %ROS2_PATH%\navigation\carrot_planner\AMENT_IGNORE
rem type NUL > %ROS2_PATH%\navigation\clear_costmap_recovery\AMENT_IGNORE
rem type NUL > %ROS2_PATH%\navigation\costmap_2d\AMENT_IGNORE
rem type NUL > %ROS2_PATH%\navigation\dwa_local_planner\AMENT_IGNORE
rem type NUL > %ROS2_PATH%\navigation\fake_localization\AMENT_IGNORE
rem type NUL > %ROS2_PATH%\navigation\global_planner\AMENT_IGNORE
rem type NUL > %ROS2_PATH%\navigation\map_server\AMENT_IGNORE
rem type NUL > %ROS2_PATH%\navigation\move_base\AMENT_IGNORE
rem type NUL > %ROS2_PATH%\navigation\move_slow_and_clear\AMENT_IGNORE
rem type NUL > %ROS2_PATH%\navigation\navfn\AMENT_IGNORE
rem type NUL > %ROS2_PATH%\navigation\navigation\AMENT_IGNORE
rem type NUL > %ROS2_PATH%\navigation\nav_core\AMENT_IGNORE
rem type NUL > %ROS2_PATH%\navigation\robot_pose_ekf\AMENT_IGNORE
rem type NUL > %ROS2_PATH%\navigation\rotate_recovery\AMENT_IGNORE
rem type NUL > %ROS2_PATH%\navigation\voxel_grid\AMENT_IGNORE
type NUL > %ROS2_PATH%\orocos_kinematics_dynamics\python_orocos_kdl\AMENT_IGNORE
rem type NUL > %ROS2_PATH%\pcl_conversions\AMENT_IGNORE
type NUL > %ROS2_PATH%\poco_vendor\AMENT_IGNORE
rem type NUL > %ROS2_PATH%\ps3joy\AMENT_IGNORE
type NUL > %ROS2_PATH%\rmw_connext\rmw_connext_dynamic_cpp\AMENT_IGNORE
type NUL > %ROS2_PATH%\robot_model\collada_parser\AMENT_IGNORE
type NUL > %ROS2_PATH%\robot_model\collada_urdf\AMENT_IGNORE
type NUL > %ROS2_PATH%\robot_model\joint_state_publisher\AMENT_IGNORE
type NUL > %ROS2_PATH%\robot_model\kdl_parser_py\AMENT_IGNORE
type NUL > %ROS2_PATH%\robot_model\urdf_parser_plugin\AMENT_IGNORE
rem type NUL > %ROS2_PATH%\ros_astra_camera\AMENT_IGNORE
rem type NUL > %ROS2_PATH%\spacenav_node\AMENT_IGNORE
rem type NUL > %ROS2_PATH%\teleop_twist_joy\build\teleop_twist_joy\AMENT_IGNORE
rem type NUL > %ROS2_PATH%\turtlebot2_demo\AMENT_IGNORE
rem type NUL > %ROS2_PATH%\turtlebot2_demo\build\depthimage_to_pointcloud2\AMENT_IGNORE
rem type NUL > %ROS2_PATH%\turtlebot2_demo\build\turtlebot2_drivers\AMENT_IGNORE
rem type NUL > %ROS2_PATH%\turtlebot2_demo\depthimage_to_pointcloud2\AMENT_IGNORE
rem type NUL > %ROS2_PATH%\turtlebot2_demo\turtlebot2_drivers\build\turtlebot2_drivers\AMENT_IGNORE
type NUL > %ROS2_PATH%\vision_opencv\cv_bridge\AMENT_IGNORE
type NUL > %ROS2_PATH%\vision_opencv\opencv_tests\AMENT_IGNORE
type NUL > %ROS2_PATH%\vision_opencv\vision_opencv\AMENT_IGNORE
rem type NUL > %ROS2_PATH%\wiimote\AMENT_IGNORE
rem D:\git\ros2-onecore\src\vendor\cartographer\AMENT_IGNORE