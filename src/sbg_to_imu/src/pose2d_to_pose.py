#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Pose2D, PoseWithCovarianceStamped, Quaternion
from tf.transformations import quaternion_from_euler
from nav_msgs.msg import Odometry

class pose2d_to_pose:

    def __init__(self):

        self.pose_with_covariance = PoseWithCovarianceStamped()

        self.publisher = rospy.Publisher("odom", PoseWithCovarianceStamped, queue_size=100)

        self.odom = Odometry()

        self.msg_seq = 0

        self.pose2D_listener()

    def pose2D_callback(self, pose2d_msg):

        self.odom.pose.pose.position.x = pose2d_msg.x
        self.odom.pose.pose.position.y = pose2d_msg.y

        self.odom.pose.pose.orientation = Quaternion(
            *(quaternion_from_euler(0, 0, pose2d_msg.theta, axes='sxyz')))

        self.pose.header.frame_id = "odom_combined"
        self.pose.header.stamp = rospy.Time.now()

        for i in range(36):
            self.pose_with_covariance.pose.covariance[i] = 0.5

        self.publisher.publish(self.pose_with_covariance)

    def calculate_velocities(self, pose2d_msg):
        pass

    def pose2D_listener(self):
        rospy.init_node("scanmatcher_pose", anonymous=True)

        rospy.Subscriber("pose2D", Pose2D, self.pose2D_callback, queue_size=1)

        rospy.spin()

if __name__ == "__main__":
    try:
        pose2d_to_pose()
    except rospy.ROSInterruptException:
        rospy.loginfo("Scan Pose Data Interrupted.")
