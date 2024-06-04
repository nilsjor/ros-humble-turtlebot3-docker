@echo off

docker run ^
   --rm ^
   --privileged ^
   --interactive --tty ^
   --net=host ^
   --env ROS_MASTER_URI=http://192.168.0.230:11311 ^
   --env ROS_HOSTNAME=192.168.0.230 ^
   --name turtlebot-kinetic ^
   turtlebot-kinetic

if %ERRORLEVEL% NEQ 0 (
   pause
)
