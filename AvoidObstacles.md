# Simple autonomous "avoid obstacles" robot for Windows 10 IoT Core

## Introduction

## Build

1. See [README.md](./README.md)

## Setup (assuming development folder of `c:\dev\ros2`):

1. Install X64 Windows IoT Core on your MBM

1. Configure Kobuki driver **TODO: get these instructions correct ... this is from memory**

     1. Copy `c:\dev\ros2`\src\turtlebot2-win10\kobuki.inf to your device

     1. Install kobuki.inf using devcon tool from SSH:

          ```
          devcon dp_add kobuki.inf
          ```
     1. Configure kobuki to use COM1 from SSH:

          ```
          reg add "hklm\system\controlset001\enum\ftdibus\VID_0403+PID_6001+kobuki_AH02B8WIA\0000\Device Parameters" /v PortName /t REG_SZ /d COM1 /f
          ```
     1. Restart kobuki driver using devcon tool from SSH:

          ```
          devcon restart *usb*vid_0403*pid_6001*
          ```
1. Create C:\data\ROS2 on your MBM

1. Copy contents of `c:\dev\ros2`\install to your MBM's C:\data\ROS2

1. Create C:\data\ROS2\AvoidObstacles.cmd file to start the required AvoidObstacles nodes

     ```
     set Path=c:\data\ros2\Scripts;c:\data\ros2\bin;%path%
     start C:\data\ros2\bin\kobuki_node.exe
     start C:\Data\ros2\bin\avoid_obstacles_node.exe
     ```
1. Create the following task on your MBM using SSH:

     ```
     schtasks /create /tn AvoidObstacles /f /sc onstart /ru system /tr "C:\Data\ros2\AvoidObstacles.bat"
     ```
1. If you do not have a network configured, you need to enable loopback by following [these steps](./NoNetwork.md)

1. Restart device.

