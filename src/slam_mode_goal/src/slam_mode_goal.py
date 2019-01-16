#!/usr/bin/env python

import rospy
import math
import actionlib
import numpy as np

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from tf.transformations import quaternion_from_euler

"""
This version of the planner takes into account
the laserscan messages instead of the costmap.

The main difference for this version of the 
unknown area planner is that it takes into
account the current data instead of the data
that is already in the costmap

The sparse matrix that arises from using the costmap
slows down computation and is much harder to perform
calculations on due to its grid-like style rather than
using distances and the angle at which those distances
were taken.
"""

# Weight constants for calculating the vectors
K = 0.005
FORWARD_WEIGHT_D = 0.05

# Safety tolerance, the kart cannot be this close to
# another obstacle (meters)
SAFETY_TOLERANCE = 0.1

class MoveBaseSequence():

    def __init__(self):
        rospy.init_node("move_base_sequence")

        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.client.wait_for_server()

        self.goal_running = False

        self.goal = MoveBaseGoal()

        self.laserscan_listener()


    def callback(self, laserscan):
        if not self.goal_running:
            self.goal_running = True
            vec_x, vec_y = self.compute_vector_field(laserscan)
            self.goal.target_pose.header.frame_id = "map"
            self.goal.target_pose.header.stamp = rospy.Time.now()
            self.goal.target_pose.pose.position.x = vec_x
            self.goal.target_pose.pose.position.y = vec_y
            self.goal.target_pose.pose.orientation = Quaternion(*(quaternion_from_euler(0, 0, math.atan2(vec_y, vec_x), axes='sxyz')))
            self.client.send_goal(self.goal)
            print(vec_x, vec_y)
            print(math.atan2(vec_y, vec_x))
            print(self.goal)
            self.wait = self.client.wait_for_result()

            if not wait:
                rospy.logerr("Unable to wait for result.")
                rospy.signal_shutdown("Unable to wait for result.")

        if self.client.get_state() == actionlib.SimpleGoalState.DONE:
            self.goal_running = False

    """
    This should be the main function running, as it receives
    laserscan messages it should interrupt and perform the operations
    on the vector field.
    """
    def laserscan_listener(self):
        rospy.Subscriber("/top/scan", LaserScan, self.callback)
        rospy.spin()

    """
    This function computes the vector field that is created
    by the objects in the distance and the resultant vector
    components that will be safe to act upon when moving the kart
    """
    def compute_vector_field(self, laserscan):
        self.angle_min = laserscan.angle_min
        self.angle_max = laserscan.angle_max
        self.angle_incr = laserscan.angle_increment

        self.ranges = np.array(laserscan.ranges)

        # Create an array to hold the angles
        theta_arr = np.arange(start=round(self.angle_max, 6), stop=round(self.angle_min + (2 * math.pi), 6), step=round(self.angle_incr, 6), dtype=np.float)

        # Calculate the x & y components for the given information
        x_vector_comp = np.multiply(np.divide(K, np.square(self.ranges)), np.cos(theta_arr))
        y_vector_comp = np.multiply(np.divide(K, np.square(self.ranges)), np.sin(theta_arr))

        # Sum the vectors
        x_vec = np.sum(x_vector_comp)
        y_vec = np.sum(y_vector_comp)

        # Add a weight for the kart to move forward
        forward_weight_mag = (K / (FORWARD_WEIGHT_D ** 2))
        x_vec += forward_weight_mag * math.cos(0)
        y_vec += forward_weight_mag * math.sin(0)

        # verify the vector is safe and move if so, otherwise give zero vector
        if self.verify_vector(laserscan, (x_vec, y_vec)):
            # return the vectors as a zipped tuple
            return (x_vec, y_vec)
        else:
            return (0, 0)

    """
    The importance of this function is that it verifies that once
    the vector is generated, nothing is in the way of the kart
    and that the vector is safe to act upon.

    To be extra safe, every range finding is checked to make sure
    that the kart will not be hitting the possible object in the distance
    """
    def verify_vector(self, laserscan, mov_vec):
        # Create an array to hold the angles
        theta_arr = np.arange(start=round(self.angle_min, 6), stop=round(self.angle_max, 6),
                              step=round(self.angle_incr, 6), dtype=np.float)

        # Unpack the given movement vector
        vec_x, vec_y = mov_vec

        # Calculate the magnitude and angle of the given vector
        vec_mag = math.sqrt((vec_x ** 2) + (vec_y ** 2))
        vec_theta = math.atan2(vec_y, vec_x)

        # Check each range at each angle, to actually test the distance a few
        # things will be checked. The first of which is if the vector is through
        # an object. The second is if the kart is too close to the obstacle
        # For the first part this can be done by creating a line segment perpendicular
        # to the movement vector of size SAFETY_TOLERANCE and checking for intersection
        # for dist in np.nditer(theta_arr):
        #   pass
        # TODO: Evaluate need for verification of vector if nav stack provides support
        return True


if __name__ == "__main__":
    try:
        MoveBaseSequence()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation Complete.")
