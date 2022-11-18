# multibot-startup-example
Utiliziing ROS, Gazebo, and Turtlebot3 demonstrate how the multibot startup works. The startup sample script is imported as a submodule from ```https://github.com/terminator0117/multibot-paramater-startup```  

## Setup
Clone or download repository into catkin_ws (or youre own preferred workspace).

Required Packages:
* rospy
* tf
* robot_state_publisher
* gazebo_ros
* move_base
* turtlebot3
* turtlebot3_gazebo
* turtlebot3_slam
* turtlebot3_navigation
* multirobot_map_merge
Required message packages
* nav_msgs
* geometry_msgs
* move_base_msgs

## Startup
### Option 1
In different terminal windows run each command:
```
roslaunch multibot_move_control spawn_3_bots.launch
```
```
roslaunch multibot_move_control start_move_base.launch bot_namespace:="robot_0"
```
```
roslaunch multibot_move_control start_move_base.launch bot_namespace:="robot_1"
```
```
roslaunch multibot_move_control start_move_base.launch bot_namespace:="robot_2"
```
```
roslaunch multibot_move_control run_example.launch
```

The first command spawns all of the robots. All of the other launches start up the move_base files for each robot. You need the same number of the move_base launches as you do spawned robots. You just need to update the bot_namespace arguement. The number for the namespace incrementally goes up by 1 from 0. 

### Option 2
In different terminal windows run each command:
```
roslaunch multibot_move_control spawn_3_nav_bots.launch
```
```
roslaunch multibot_move_control run_example.launch
```

The first command spawns all of the robots along with their respective move_base files. 

### In addition 
There is other spawn launch files with a different number. These just spawn a different number of robots  files. 
