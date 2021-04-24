#!/usr/bin/env python
"""
Created on Thu Apr  15 09:53:16 2021

@author: sharm
"""

import rospy
import math
import sys
import tf
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from velocity_publisher import computation
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal



rospy.init_node("move_robot")
pub = rospy.Publisher("cmd_vel", Twist, queue_size=5)
velocity_msg = Twist()
rate = rospy.Rate(4)
tf_listener = tf.TransformListener()
parent_frame = 'odom'
child_frame = 'base_footprint'
k_h_gain = 1
k_v_gain = 1



try:
    tf_listener.waitForTransform(parent_frame, child_frame, rospy.Time(), rospy.Duration(1.0))
except (tf.Exception, tf.ConnectivityException, tf.LookupException):
    rospy.loginfo("Cannot find transform between {p} and {c}".format(p=parent_frame, c=child_frame))
    rospy.signal_shutdown("tf Exception")


def get_odom_data():
    try:
        (trans, rot) = tf_listener.lookupTransform(parent_frame, child_frame, rospy.Time(0))

        rotation = euler_from_quaternion(rot)
    except (tf.Exception, tf.ConnectivityException, tf.LookupException):
        rospy.loginfo("TF Exception")
        return

    return Point(*trans), rotation[2]


def sensor_callback(msg):
    front = msg.ranges[0]
    left = msg.ranges[90]
    right = msg.ranges[270]

    rospy.loginfo("Distance from obstacle (front): {f}".format(f=front))
    rospy.loginfo("Distance from obstacle (left): {l}".format(l=left))
    rospy.loginfo("Distance from obstacle (right): {r}".format(r=right))
    rospy.loginfo("--" * 20)

    rospy.init_node('laser_data') #to initiate the node 'laser data'
    sub = rospy.Subscriber('scan', LaserScan, sensor_callback) #create Subscriber. Do we need it??
    rospy.spin() 
    
    if front < 1.0:
        #velocity_msg.linear.x = 0.5 # move is from the publisher. should check into that
        if right > 1.0:
            velocity_msg.angular.z = -1.5  #move right
        else:
            velocity_msg.angular.z = 1.5  #move left
    if front < 0.2: #should check when the robot hits the obstacle
        velocity_msg.linear.x = -0.5  

    rospy.init_node('obstacle_avoidance')   
    sub = rospy.Subscriber('scan', LaserScan, sensor_callback)
    pub = rospy.Publisher('/cmd_vel', Twist) # Creating publisher. But is it needed?
    #move = Twist()
    
    rospy.spin()


def go_to_goal(goal_x, goal_y):
    (position, rotation) = get_odom_data()
    last_rotation = 0

    # compute the distance from the current position to the goal
    distance_to_goal = computation.compute_distance(position.x, position.y, goal_x, goal_y)

    while distance_to_goal > 0.05:
        (position, rotation) = get_odom_data()
        x_start = position.x
        y_start = position.y
        rospy.loginfo("x = {0}, y = {1}".format(x_start, y_start))
        angle_to_goal = math.atan2(goal_y - y_start, goal_x - x_start)

        if angle_to_goal < -math.pi/4 or angle_to_goal > math.pi/4:
            if 0 > goal_y > y_start:
                angle_to_goal = -2 * math.pi + angle_to_goal
            elif 0 <= goal_y < y_start:
                angle_to_goal = 2 * math.pi + angle_to_goal
        if last_rotation > math.pi - 0.1 and rotation <= 0:
            rotation = 2 * math.pi + rotation
        elif last_rotation < -math.pi + 0.1 and rotation > 0:
            rotation = -2 * math.pi + rotation

        velocity_msg.angular.z = k_v_gain * angle_to_goal-rotation
        distance_to_goal = computation.compute_distance(position.x, position.y, goal_x, goal_y)

        velocity_msg.linear.x = min(k_h_gain * distance_to_goal, 0.5)


        if velocity_msg.angular.z > 0:
            velocity_msg.angular.z = min(velocity_msg.angular.z, 1.5)
        else:
            velocity_msg.angular.z = max(velocity_msg.angular.z, -1.5)

        # update the new rotation for the next loop
        last_rotation = rotation
        pub.publish(velocity_msg)
        rate.sleep()

    # force the robot to stop by setting linear and angular velocities to 0
    velocity_msg.linear.x = 0.0
    velocity_msg.angular.z = 0.0
    # publish the new message on /cmd_vel topic
    pub.publish(velocity_msg)

# rosrun 
if __name__ == "__main__":
    action = ""
    if len(sys.argv) == 2:
        action = str(sys.argv[1])
    else:
        sys.exit('Unknown arguments passed to the command line')
        
    if action == 1:
        while not rospy.is_shutdown():
            go_to_goal(3,3)
            
    else:
        sys.exit('Unknown argument')

        
    
    