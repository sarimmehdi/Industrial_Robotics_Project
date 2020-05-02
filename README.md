# Industrial_Robotics_Project
Mapping and Navigation in the ROS simulation environment

# How to execute mapping
roslaunch turtlebot3_gazebo turtlebot3_house.launch

roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping

rosrun ros_project mapping.py

# How to execute navigation
Make sure your goal file follows the same format as provided here (have a look at goals.txt) and it needs to be in your catkin_ws folder (ROS workspace folder). Your Map pgm and yaml files need to be in $HOME

roslaunch turtlebot3_gazebo turtlebot3_house.launch

roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml

rosservice call /global_localization "{}"

rosrun ros_project navigation.py
