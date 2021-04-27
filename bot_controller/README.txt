------ Project 3 : Phase 3 ----------
ENPM661				
Charu Sharma			John-Edward Draganov
117555448			113764875
charu107@umd.edu		jdragano@umd.edu
-------------------------------------
python: 3.0
IDE used : Spyder

The code runs on A star algorithm and executes the path for the robot to travel through the obstacles to reach 
the goal position.
The start and the goal positions are taken as inputs so that we can execute different positions with the same 
file.

Libraries used: geometry_msgs, sensor_msgs, tf.transformations, velocity_publisher, actionlib, move_base_msgs,
rospy, sys, math- These libraries are used to move the turtlebot.
from matplotlib, numpy, pygame, heapq, time-These libraries are used to calculate the optimum path.

We have implemented the already used A star algorithm and added the movement function. This new movement
function is based on different wheel velocity of the robot.

Have implemented the the movement in gazebo using ROS. Publishers and Subscribers are used to for the 
transversing through the plane. Lidar sensor is used to detects the surroundings.

Process to run the code:
Attaching the whole package so that it gets easier to work with.

1. Open the workspace and source the file, then catkin_make or catkin_build.
2. In one terminal roscore
3. Run rosalunch turtlebot3_gazebo turtlebot3_empty_world.launch
4. Run rosrun bot_controller temp 1 1 goal

IMP: Please uncomment direct position controller at the bottom if the differential drive doesnt work.