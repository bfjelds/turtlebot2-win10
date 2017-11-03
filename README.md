# Turtlebot2 for Windows 10 IoT Core

## Introduction

## Requirements

1. Visual Studio 2017

1. MinnowboardMax as x64 Windows IoT Core (with fix for Gamepad API)

1. Turtlebot2

1. Xbox 360 Controller

1. Microsoft Xbox 360 Wireless Receiver for Windows

## Download and Build

```
NOTE: to build this specifically for Window 10 IoT Core, I have changed the way CMAKE works so that
by default it only links to onecoreuap.lib.  This is not required because of the API forwarders.  To
do this, change C:\Program Files\CMake\share\cmake-3.9\Modules\Platform\Windows-MSVC.cmake 
on the line below marked with ****

	  if(_MSVC_C_ARCHITECTURE_FAMILY STREQUAL "ARM" OR _MSVC_CXX_ARCHITECTURE_FAMILY STREQUAL "ARM")
	    set(CMAKE_C_STANDARD_LIBRARIES_INIT "kernel32.lib user32.lib")
	  elseif(MSVC_VERSION GREATER 1310)
	    if(CMAKE_VS_PLATFORM_TOOLSET MATCHES "(v[0-9]+_clang_.*|LLVM-vs[0-9]+.*)")
	      # Clang/C2 in MSVC14 Update 1 seems to not support -fsantinize (yet?)
	      # set(_RTC1 "-fsantinize=memory,safe-stack")
	      set(_FLAGS_CXX " -frtti -fexceptions")
	    else()
	      set(_RTC1 "/RTC1")
	      set(_FLAGS_CXX " /GR /EHsc")
	    endif()
****	    set(CMAKE_C_STANDARD_LIBRARIES_INIT "onecoreuap.lib")
	  else()
	    set(_RTC1 "/GZ")
	    set(_FLAGS_CXX " /GR /GX")
	    set(CMAKE_C_STANDARD_LIBRARIES_INIT "kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib")
	  endif()
	

Also, to use OpenCV and VS2017, I had to modify <OpenCV_DIR>\OpenCVConfig.cmake from:

  elseif(MSVC_VERSION EQUAL 1910)
    set(OpenCV_RUNTIME vc15)

to this:

  else()
    set(OpenCV_RUNTIME vc15)


To build debug binaries, follow the "Extra stuff for Debug mode" instructions found here 
https://github.com/ros2/ros2/wiki/Windows-Development-Setup.  Then build using the following 
command:

     python src\ament\ament_tools\scripts\ament.py build --cmake-args -DCMAKE_BUILD_TYPE=Debug
    
     
```

1. Using 'x64 Native Tools Command Prompt for VS **2017**' (not **2015**), follow setup instructions (up to, **BUT NOT INCLUDING**, Building the ROS 2 Code) for ROS2 found 
[here](https://github.com/ros2/ros2/wiki/Windows-Development-Setup)

1. Navigate to the development folder you set up in the first step (we will assume, as ROS2 suggests in their 
instructions, `c:\dev\ros2`)

1. Run the following commands to get the Turtlebot2 source code:

     ```
     curl -sk https://raw.githubusercontent.com/bfjelds/turtlebot2-win10/master/turtlebot2_win10.repos -o turtlebot2_win10.repos
     vcs import src < turtlebot2_win10.repos
     ```
1. Run the following command to build the Turtlebot2 binaries from a VS2015 x64 Native Tools Command Prompt (assuming 
development folder of `c:\dev\ros2`):

     ```
     C:\dev\ros2\src\turtlebot2-win10\build.cmd
     ```
1. Validate that the following files exist (assuming development folder of `c:\dev\ros2`):

     ```
     C:\dev\ros2\install\lib\teleop_twist_joy\teleop_node.exe
     C:\dev\ros2\bin\kobuki_node.exe
     GamepadNodeUwp.appx
     ```
     
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
1. Install GamepadNodeUwp. This can be done in several ways: **TODO: make the following instructions true**

    1. From Visual Studio. To do this, open `c:\dev\ros2`\src\turtlebot2-win10\GamepadNodeUwp\GamepadNodeUwp.vcxproj using 
    Visual Studio.  Specify your MBM as target and F5 deploy it.

    1. From the Windows Device Portal. To do this, open your MBM's Device Portal by opening <IP>:8080 in a browser.  Navigate 
    to the Apps tab and deploy your APPX.

    1. From SSH.  Copy your APPX, dependency APPXs, and CER files.  Use the deployappx tool to install.

1. Create C:\data\ROS2 on your MBM

1. Copy contents of `c:\dev\ros2`\install to your MBM's C:\data\ROS2

1. Create C:\data\ROS2\turtlebot2.cmd file to start the required Turtlebot2 nodes

     ```
     set Path=c:\data\ros2\Scripts;c:\data\ros2\bin;%path%
     start C:\data\ros2\bin\kobuki_node.exe
     start C:\Data\ros2\Lib\teleop_twist_joy\teleop_node.exe
     ```
1. Create the following task on your MBM using SSH:

     ```
     schtasks /create /tn Turtlebot2 /f /sc onstart /ru system /tr "C:\Data\ros2\turtlebot2.bat"
     ```
1. Configure GamepadNodeUwp to be the startup app on your MBM by using SSH:

     ```
     iotstartup add headed GamepadNodeUwp
     ```
1. The UWP Gamepad app requires a valid network connection to communicate with the other nodes.
If you create a set of nodes that don't involve UWP, you can enable loopback without a network
connection by following [these steps](.\NoNetwork.md)

1. Restart device.

