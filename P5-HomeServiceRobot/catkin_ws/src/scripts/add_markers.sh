#!/bin/sh

echo ""
echo "Launching add markers..."

# Catkin Workspace Path
PATH_CATKIN_WS="/home/robond/workspace2/Robotics-ND/P5-HomeServiceRobot/catkin_ws"


# 1 - World Launch
# Open workspace, source setup, launch turtlebot_world.launch
echo "Launching Turtlebot in World..."
echo  "cd ${PATH_CATKIN_WS} && source devel/setup.bash && roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=${PATH_CATKIN_WS}/src/map/newworld.world " 
echo " "
xterm -title world -bg olive -e " cd ${PATH_CATKIN_WS} && source devel/setup.bash && roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=${PATH_CATKIN_WS}/src/map/newworld.world " & 

sleep 10

# 2 - View Navigation Launch
# Open the workspace, source and launch view_navigation.launch
echo "Launching View Navigation..."
#echo  " roslaunch rviz rviz -d src/rvizConfig/homeServiceConfig.rviz" 
echo " cd ${PATH_CATKIN_WS} && source devel/setup.bash && roslaunch turtlebot_rviz_launchers view_navigation.launch rviz_path:=${PATH_CATKIN_WS}/src/rvizConfig/homeServiceConfig.rviz"
echo " "
xterm -title navigation -bg navy -e " cd ${PATH_CATKIN_WS} && source devel/setup.bash && roslaunch turtlebot_rviz_launchers view_navigation.launch rviz_path:=${PATH_CATKIN_WS}/src/rvizConfig/homeServiceConfig.rviz" &
#xterm -title navigation -e " cd ${PATH_CATKIN_WS} && source devel/setup.bash && roslaunch rviz rviz -d src/rvizConfig/homeServiceConfig.rviz" &

sleep 10

#3 - AMCL launch
echo "Launching AMCL Demo..."
echo " cd ${PATH_CATKIN_WS} && source devel/setup.bash && roslaunch turtlebot_gazebo amcl_demo.launch map_file:=${PATH_CATKIN_WS}/src/map/map.yaml " & 
echo " "
xterm -title AMCL -e " cd ${PATH_CATKIN_WS} && source devel/setup.bash && roslaunch turtlebot_gazebo amcl_demo.launch map_file:=${PATH_CATKIN_WS}/src/map/map.yaml " & 

sleep 10

#4 - Add Markers Timed launch
echo "Adding Markers (timed)...."
echo " roslaunch add_markers add_markers_timed.launch " 
xterm -bg darkblue -geom 80x40 -e " roslaunch add_markers add_markers_timed.launch " &
