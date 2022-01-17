#!/usr/bin/env python

import math
from math import sin, cos, pi

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

rospy.init_node('odometry_publisher')

odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)

x = 0.0
y = 0.0
th = 0.0

vx = 0.00000001
vy = 0.0
vth = 0.0000001

current_time = rospy.Time.now()
last_time = rospy.Time.now()

r = rospy.Rate(20)
while not rospy.is_shutdown():
    current_time = rospy.Time.now()

    # compute odometry in a typical way given the velocities of the robot
    dt = (current_time - last_time).to_sec()
    delta_x = (vx * cos(th) - vy * sin(th)) * dt
    delta_y = (vx * sin(th) + vy * cos(th)) * dt
    delta_th = vth * dt

    x += delta_x
    y += delta_y
    th += delta_th

    # since all odometry is 6DOF we'll need a quaternion created from yaw
    odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

    # next, we'll publish the odometry message over ROS
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom_combined"

    # set the position
    odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))
    for i in range(6):
        odom.pose.covariance[i + (i * 6)] = 1
        odom.twist.covariance[i + (i * 6)] = 1

    # set the velocity
    odom.child_frame_id = "base_footprint"
    odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

    # publish the message
    odom_pub.publish(odom)

    last_time = current_time
    r.sleep()
