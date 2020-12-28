#!/bin/sh

# Create a test_slam.sh shell script that launches these files
# turtlebot_world.launch: to deploy a turtlebot in your environment
# gmapping_demo.launch or slam_gmapping: perform SLAM
# view_navigation.launch: observe the map in rviz
# keyboard_teleop.launch: manually control the robot with keyboard commands

# Catkin Workspace Path
PATH_CATKIN_WS="/home/robond/workspace2/Robotics-ND/P5-HomeServiceRobot/catkin_ws"

export

#echo ">>> USING WORLD FILE: ${TURTLEBOT_GAZEBO_WORLD_FILE}"
echo " "

# Open workspace, source setup, launch turtlebot_world.launch
echo  " cd ${PATH_CATKIN_WS} && source devel/setup.bash && roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=${PATH_CATKIN_WS}/src/map/newworld.world " 
echo " "
xterm -title world -bg olive -e " cd ${PATH_CATKIN_WS} && source devel/setup.bash && roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=${PATH_CATKIN_WS}/src/map/newworld.world " & 

sleep 10

#
# Open the workspace, source and launch gmapping_demo.launch
echo " cd ${PATH_CATKIN_WS} && source devel/setup.bash && roslaunch turtlebot_gazebo gmapping_demo.launch" &
echo " "
xterm -title gmapping -bg teal -e " cd ${PATH_CATKIN_WS} && source devel/setup.bash && roslaunch turtlebot_gazebo gmapping_demo.launch" &

sleep 10

# Open the workspace, source and launch view_navigation.launch
echo  "cd ${PATH_CATKIN_WS} && source devel/setup.bash && roslaunch turtlebot_rviz_launchers view_navigation.launch" &
echo " "
xterm -title navigation -e " cd ${PATH_CATKIN_WS} && source devel/setup.bash && roslaunch turtlebot_rviz_launchers view_navigation.launch " &

sleep 10

# Open the workspace, source and launch keyboard_teleop.launch
echo "cd ${PATH_CATKIN_WS} && source devel/setup.bash && roslaunch turtlebot_teleop keyboard_teleop.launch"
echo " "
xterm -title Teleop -bg navy -e " cd ${PATH_CATKIN_WS} && source devel/setup.bash && roslaunch turtlebot_teleop keyboard_teleop.launch " &

