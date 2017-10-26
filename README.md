# Turtlebot2 for Windows 10 IoT Core

## Introduction

## Download and Build

1. Follow setup instructions (up to, **BUT NOT INCLUDING**, Building the ROS 2 Code) for ROS2 found [here](https://github.com/ros2/ros2/wiki/Windows-Development-Setup)

1. Navigate to the development folder you set up in the first step (we will assume, as ROS2 suggests in their instructions, C:\dev\ros2)

1. Run the following commands to get the Turtlebot2 source code:

     curl -sk https://raw.githubusercontent.com/bfjelds/turtlebot2-win10/master/turtlebot2_win10.repos -o turtlebot2_win10.repos
     vcs import src < turtlebot2_win10.repos

1. Run the following command to build the Turtlebot2 binaries from a VS2015 x64 Native Tools Command Prompt (assuming development folder of C:\dev\ros2):

     C:\dev\ros2\src\turtlebot2-win10\build.cmd

1. Validate that the following files exist (assuming development folder of C:\dev\ros2):

     C:\dev\ros2\install\lib\teleop_twist_joy\teleop_node.exe
     C:\dev\ros2\bin\kobuki_node.exe
     GamepadNodeUwp.appx

## Setup Device

### Required Hardware:

1. MinnowboardMax as x64 Windows IoT Core

1. Turtlebot2

1. Xbox 360 Controller

1. Microsoft Xbox 360 Wireless Receiver for Windows

### Setup (assuming development folder of C:\dev\ros2):

1. Install X64 Windows IoT Core on your MBM

1. Configure Kobuki driver **TODO: get these instructions correct ... this is from memory**

     1. Copy C:\dev\ros2\src\turtlebot2-win10\kobuki.inf to your device

     1. Install kobuki.inf using devcon tool from SSH:

          devcon dp_add kobuki.inf

     1. Configure kobuki to use COM1 from SSH:

          reg add "hklm\system\controlset001\enum\ftdibus\VID_0403+PID_6001+kobuki_AH02B8WIA\0000\Device Parameters" /v PortName /t REG_SZ /d COM1 /f

     1. Restart kobuki driver using devcon tool from SSH:

          devcon restart *usb*vid_0403*pid_6001*

1. Install GamepadNodeUwp. This can be done in several ways: **TODO: make the following instructions true**

    1. From Visual Studio. To do this, open C:\dev\ros2\src\turtlebot2-win10\GamepadNodeUwp\GamepadNodeUwp.vcxproj using Visual Studio.  Specify your MBM as target and F5 deploy it.

    1. From the Windows Device Portal. To do this, open your MBM's Device Portal by opening <IP>:8080 in a browser.  Navigate to the Apps tab and deploy your APPX.

    1. From SSH.  Copy your APPX, dependency APPXs, and CER files.  Use the deployappx tool to install.

1. Create C:\data\ROS2 on your MBM

1. Copy contents of C:\dev\ros2\install to your MBM's C:\data\ROS2

1. Create C:\data\ROS2\turtlebot2.cmd file to start the required Turtlebot2 nodes

     set Path=c:\data\ros2\Scripts;c:\data\ros2\bin;%path%
     start C:\data\ros2\bin\kobuki_node.exe
     start C:\Data\ros2\Lib\teleop_twist_joy\teleop_node.exe

1. Create the following task on your MBM using SSH:

     schtasks /create /tn Turtlebot2 /f /sc onstart /ru system /tr "C:\Data\ros2\turtlebot2.bat"

1. Configure GamepadNodeUwp to be the startup app on your MBM by using SSH:

     iotstartup add headed GamepadNodeUwp

1. **TODO: fix this** Make sure network connection is present.  Can be network cable plugged in or wireless Wifi dongle (with required Wifi profile configured)

1. Restart device.

