#!usr/bin/env python

import rospy
import math

from std_msgs.msg import Header

from geometry_msgs.msg import Vector3

from sbg_driver.msg import SbgImuData
from sbg_driver.msg import SbgEkfEuler
from sbg_driver.msg import SbgEkfNav
from sbg_driver.msg import SbgImuStatus
from sbg_driver.msg import SbgEkfStatus

import tf

class odom_publisher:

    def __init__(self):
        pass

    def ekf_nav_callback(self, ekf_nav_msg):
        pass

    def ekf_euler_callback(self, ekf_euler_msg):
        pass

    def ekf_listener(self):
        # Initialize the odometry publisher node
        rospy.init_node("tf_odom_publisher", anonymous=True)

        # Create subsribers to listen to the spb topics output by the sensor
        rospy.Subscriber("ekf_euler", SbgEkfEuler, self.ekf_euler_callback, queue_size=1)
        rospy.Subscriber("ekf_nav", SbgEkfNav, self.ekf_nav_callback, queue_size=1)

        # Spin and wait for new messages
        rospy.spin()

if __name__ == "__main__":
    try:
        odom_publisher()
    except rospy.ROSInterruptException:
        rospy.loginfo("Odometry Transform Interrupted")