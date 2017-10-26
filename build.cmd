
set TURTLEBOT2_WIN10_PATH=%~dp0
set SRC_PATH=%TURTLEBOT2_WIN10_PATH%\..
set ROOT_PATH=%SRC_PATH%\..

pushd %ROOT_PATH%

python src\ament\ament_tools\scripts\ament.py build

msbuild %TURTLEBOT2_WIN10_PATH%\kobuki.vcxproj /p:Configuration=Release /p:Platform=x64 /t:clean;build
msbuild %TURTLEBOT2_WIN10_PATH%\kobuki-node.vcxproj /p:Configuration=Release /p:Platform=x64 /t:clean;build
msbuild %TURTLEBOT2_WIN10_PATH%\GamepadNodeUwp\GamepadNodeUwp.vcxproj /p:Configuration=Release /p:Platform=x64 /t:clean;build


popd