#!/bin/sh

echo ""
echo "Launching pick objects..."

# Catkin Workspace Path
PATH_CATKIN_WS="/home/robond/workspace2/Robotics-ND/P5-HomeServiceRobot/catkin_ws"


# 1 - World Launch
# Open workspace, source setup, launch turtlebot_world.launch
echo "Launching Turtlebot in World..."
echo  "cd ${PATH_CATKIN_WS} && source devel/setup.bash && roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=${PATH_CATKIN_WS}/src/map/newworld.world " 
echo " "
xterm -title world -bg olive -e " cd ${PATH_CATKIN_WS} && source devel/setup.bash && roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=${PATH_CATKIN_WS}/src/map/newworld.world " & 

sleep 7

# 2 - View Navigation Launch
# Open the workspace, source and launch view_navigation.launch
echo "Launching View Navigation..."
echo  "cd ${PATH_CATKIN_WS} && source devel/setup.bash && roslaunch turtlebot_rviz_launchers view_navigation.launch" &
echo " "
xterm -title navigation -e " cd ${PATH_CATKIN_WS} && source devel/setup.bash && roslaunch turtlebot_rviz_launchers view_navigation.launch " &

sleep 7

#3 - AMCL launchin
echo "Launching AMCL Demo..."
echo " cd ${PATH_CATKIN_WS} && source devel/setup.bash && roslaunch turtlebot_gazebo amcl_demo.launch map_file:=${PATH_CATKIN_WS}/src/map/map.yaml " & 
echo " "
xterm -title AMCL -e " cd ${PATH_CATKIN_WS} && source devel/setup.bash && roslaunch turtlebot_gazebo amcl_demo.launch map_file:=${PATH_CATKIN_WS}/src/map/map.yaml " & 

#4 - Pick Objects Launch
#xterm -bg darkblue -geom 80x40 -e " roslaunch pick_objects pick_objects.launch " &
