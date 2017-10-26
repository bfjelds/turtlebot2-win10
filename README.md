# Turtlebot2 for Windows 10 IoT Core

## Introduction

## Download and Build

1. Follow setup instructions (up to, **BUT NOT INCLUDING**, Building the ROS 2 Code) for ROS2 found [here](https://github.com/ros2/ros2/wiki/Windows-Development-Setup)
1. Navigate to the development folder you set up in the first step (they call out: C:\dev\ros2)
1. Run the following commands to get the Turtlebot2 source code:
    curl -sk https://raw.githubusercontent.com/bfjelds/turtlebot2-win10/master/turtlebot2_win10.repos -o turtlebot2_win10.repos
    vcs import src < turtlebot2_win10.repos
1. Run the following command to build the Turtlebot2 binaries from a VS2015 x64 Native Tools Command Prompt:
    src\turtlebot2-win10\build.cmd
1. Validate that the following files exist:
    teleop_node.exe
    kobuki_node.exe
    GamepadNodeUwp.appx

## Setup Device

###Required Hardware:
1. MinnowboardMax as x64 Windows IoT Core
1. Turtlebot2
1. Xbox 360 Controller
1. Microsoft Xbox 360 Wireless Receiver for Windows
    
1. Install X64 Windows IoT Core on your MBM
1. Configure Kobuki driver **TODO: get these instructions correct ... this is from memory**
    1. Copy kobuki.inf to your device
    1. Install kobuki.inf
    1. Configure kobuki to use COM1
    1. Restart kobuki driver
1. Install GamepadNodeUwp
1. Create C:\data\ROS2 on your MBM
1. Copy contents of your development folders <install> folder to your MBM's C:\data\ROS2
1. Create the following tasks on your MBM using SSH:
    schtasks /create /tn Teleop /f /sc onstart /ru system /tr "C:\Data\ros2\Lib\teleop_twist_joy\teleop_node.exe" **TODO: FIX THIS PATH**
    schtasks /create /tn Kobuki /f /sc onstart /ru system /tr "C:\data\ros2\bin\kobuki_node.exe"
1. Configure GamepadNodeUwp to be the startup app on your MBM by using SSH:
    iotstartup add headed GamepadNodeUwp
1. **TODO: fix this** Make sure network connection is present.  Can be network cable plugged in or wireless Wifi dongle (with required Wifi profile configured)
1. Restart device.

