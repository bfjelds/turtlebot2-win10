@echo off

set TURTLEBOT2_WIN10_PATH=%~dp0
set SRC_PATH=%TURTLEBOT2_WIN10_PATH%\..
set EIGEN3_PATH=%SRC_PATH%\ros2-devel\eigen3
set ROOT_PATH=%SRC_PATH%\..

echo .
echo Build Eigen3
echo .
mkdir %EIGEN3_PATH%\build
mkdir %ROOT_PATH%\install
pushd %EIGEN3_PATH%\build
cmake -DCMAKE_INSTALL_PREFIX=%ROOT_PATH%\install ..
msbuild INSTALL.vcxproj
popd


pushd %ROOT_PATH%


echo .
echo Configure the build to skip AMENT projects that aren't relevant
echo .
if "%~1" == "advanced" (
call %TURTLEBOT2_WIN10_PATH%\setup-for-win10.cmd filter advanced
goto:NextStep
)
call %TURTLEBOT2_WIN10_PATH%\setup-for-win10.cmd filter turtlebot2


:NextStep

echo .
echo Build ROS2 binaries for Windows 10 Iot Core
echo .
python src\ament\ament_tools\scripts\ament.py build


echo .
echo Build Gamepad UWP binary for Windows 10 Iot Core
echo .
msbuild %TURTLEBOT2_WIN10_PATH%\GamepadNodeUwp\GamepadNodeUwp.vcxproj /p:Configuration=Release /p:Platform=x64 /t:clean;build


popd