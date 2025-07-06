#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import time
from geometry_msgs.msg import Twist

def move_car():
    print("move_car")

    rospy.init_node('car_control')
    print("rospy.init_node")

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    print("pub")

    rospy.sleep(0.5)
    print("rospy.sleep")

    msg = Twist()
    msg.linear.x = 0.05
    msg.linear.y = 0.0
    msg.linear.z = 0.0
    msg.angular.x = 0.0
    msg.angular.y = 0.0
    msg.angular.z = 0.5
    
    start = time.time()
    rate = rospy.Rate(10)
    
    while time.time() - start < 1.0 and not rospy.is_shutdown():
        pub.publish(msg)
        rate.sleep()
    
    stop_msg = Twist()
    pub.publish(stop_msg)

if __name__ == "__main__":
    move_car()