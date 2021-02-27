"""This node is respondible for goal setting and sending those goals to the ROS Navigation Stack.

The goal setting algorithm is needed to navigate around the track without a 
existing map (during lap 1). This node takes 2D laser scans (/top/scan) then 
constructs a goal message and creates an ActionClient to sends the goal as 
ROS actions to SimpleActionServer on move_base.

Note: /move_base_simple/goal is a non-action topic that move_base subsribes 
to in case users don't want to use the ActionServer but all of our goals 
go through the action server. When looking at an rqt_graph or visualizing 
the connections of nodes, you can think of slam_mode_goal publishing to 
/move_base_simple/goal which move_base subscribes to. But know that in 
actuality, slam_mode_goal node is sending ROS Actions to the move_base 
ActionAPI to send move_base new goals to pursue (move_base will get these 
from move_base/goal Action Topic). 
For more info see http://wiki.ros.org/move_base#Action_API.
"""
#!/usr/bin/env python

import rospy
import math
import actionlib
import numpy as np
import time

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Vector3
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from tf.transformations import quaternion_from_euler



# Weight constants for calculating the vectors
K = 0.0005
FORWARD_WEIGHT_D = 0.02

# Safety tolerance, the kart cannot be this close to
# another obstacle (meters)
SAFETY_TOLERANCE = 0.1

# Update rate for the goal, this should be dependent
# on the incoming messages. (in seconds)
UPDATE_RATE = 0.5

# Debug flag for use in displaying extra information
# to rviz. If true debugging information will be shown.
DEBUG_FLAG = False

# Scale factor for the vectors in rviz
SCALE_FACTOR = 2000

KART_WIDTH = 0.4
KART_LENGTH = 0.6


def visualize_field(x_vec_arr, y_vec_arr, ranges, theta_arr):
  """
  This function displays the vector field into a pose array
  that can be viewed in RVIZ. This should only be used for debugging
  purposes as it can be costly to run RVIZ while the kart is actually
  running.
  """
  # Create a publisher for the rostopic 'vector_field'
  # this will publish the marker array messages
  publisher = rospy.Publisher('vector_field', MarkerArray)

  vec_field = MarkerArray()

  # Variable used to ID each marker in the array
  i = 0

  # Iterate through the following arrays and add markers to the marker array
  # message to be sent
  for r, theta, x_or, y_or in np.nditer([ranges, theta_arr, x_vec_arr, y_vec_arr]):
    # Initialize a marker message
    _marker = Marker()

    # Populate the pose field of the marker message
    _marker.pose.position.x = r * math.cos(theta + math.pi)
    _marker.pose.position.y = r * math.sin(theta + math.pi)
    _marker.pose.position.z = 0

    _marker.pose.orientation = Quaternion(*(quaternion_from_euler(0, 0, math.atan2(y_or, x_or) + math.pi, 'sxyz')))

    # Populate the scale field of the marker message
    _marker.scale.x = SCALE_FACTOR * x_or
    _marker.scale.y = SCALE_FACTOR * y_or
    _marker.scale.z = 0.0001

    # Populate the color field of the marker message -> teal
    _marker.color.r = 0
    _marker.color.g = 128
    _marker.color.b = 128
    _marker.color.a = 255

    # Populate the header of the marker message
    _marker.header.stamp.secs = rospy.Time.now().secs
    _marker.header.stamp.nsecs = rospy.Time.now().nsecs
    _marker.header.frame_id = "base_link"
    _marker.header.seq = i

    _marker.ns = "cur_marker"
    _marker.id = i

    i += 1

    _marker.type = 0
    _marker.lifetime.secs = UPDATE_RATE
    _marker.frame_locked = False

    # Append the marker message to the list of markers
    vec_field.markers.append(_marker)

  # Publish the marker array
  publisher.publish(vec_field)


def verify_vector(laserscan, mov_vec):
  """
  The importance of this function is that it verifies that once
  the vector is generated, nothing is in the way of the kart
  and that the vector is safe to act upon.

  To be extra safe, every range finding is checked to make sure
  that the kart will not be hitting the possible object in the distance
  """
  angle_min = laserscan.angle_min
  angle_max = laserscan.angle_max
  angle_incr = laserscan.angle_increment

  ranges = np.array(laserscan.ranges)

  # Create an array to hold the angles
  theta_arr = np.arange(start=round(angle_min, 6), stop=round(angle_max, 6),
                        step=round(angle_incr, 6), dtype=np.float)

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
  # TODO: Evaluate need for verification of vector if nav stack provides support
  return True


def weighted_resultant_vector(laserscan):
  """
  Calculates the goal vector relative to the karts position based on wall shape 
  in the front 180 deg view frame.

  The goal vector is calculated by taking the weighted average from a set a 
  set of 90 angles. Each angle is calculated by taking the tangent of the resultant 
  vector created from the sum of the positive and its corresponding negative 
  lazerscans in the set of lazerscans (e.g. the sum of the pi/4 and -pi/4 angle 
  vectors). The vector created from the average theta is a unit vector. When the 
  kart gets to close to a wall, the x and y are scaled so that it is able to take
  a sharper turn while minimizing the chance that the goal is place into a wall.
  As of now, each of these thetas added to 'theta_tot' has the same 
  weighting (but this may be adjusted in the future). 
  """
  # Grab the min and max angles of the 2D lazerscan (radians)
  angle_min = laserscan.angle_min
  angle_max = laserscan.angle_max

  # A list containing all of the distances from -pi/4 to pi/4 rad
  ranges = laserscan.ranges

  # Parallel list where each element is the angle corresponding to the 
  # lazer scan distance at the same index in 'ranges'.
  theta_arr = np.linspace(start=round(angle_min, 6), 
                           stop=round(angle_max, 6), num=len(ranges))

  # Iterate through left half of the lazerscans and grab corresponding
  # right lazerscan for each left one.
  theta_tot = 0
  for i in range(len(ranges) / 2):
    l_dist = ranges[i]
    l_ang = theta_arr[i]
    r_dist = ranges[-1 - i]
    r_ang = theta_arr[-1 - i]

    # Calculate x and y components of the vector that is sum of 
    # left and right lazerscan vectors
    x = l_dist * math.cos(l_ang) + r_dist * math.cos(r_ang)
    y = l_dist * math.sin(l_ang) + r_dist * math.sin(r_ang)

    # Add the theta of res vector
    theta_tot += math.atan(y / x)

  theta = theta_tot / (len(ranges) / 2)

  # If the vector (x_pos, y_pos) is too close to a wall, then 
  # decrease magnitude of vector and give more weight to the y position.
  x_pos = math.cos(theta)
  y_pos = math.sin(theta)

  ## Experimentally determining this based on simulation. Will want to get do this more thoroughly later
  thresh_dist = 2.1 * KART_LENGTH
  front_dist_from_wall = ranges[len(ranges) / 2] - KART_LENGTH

  if front_dist_from_wall < thresh_dist:
    x_pos *= 0.35
    y_pos *= 0.85

  print "theta: {}, x: {}, y: {}".format(theta, x_pos, y_pos) 

  # Orientation of kart at goal should be the resultant vect angle (may not be the best)
  orient = math.atan(y_pos / x_pos)

  return x_pos, y_pos, orient


def callback(laserscan):
  """
  This function goes into effect when a new laserscan message is taken
  from the ROS system. With each new message this function sends a new
  movement goal based on potential field avoidance and obstacle collision.
  """
  # Create the action client that sends messages to move base
  client = actionlib.SimpleActionClient("move_base", MoveBaseAction)

  # Wait for initialization of the action client
  client.wait_for_server()

  # Initialize the move base goal message to be sent
  goal = MoveBaseGoal()
  
  x_pos, y_pos, orient = weighted_resultant_vector(laserscan)

  # Fill in the message with header information and the movement vectors
  goal.target_pose.header.frame_id = "base_link"
  goal.target_pose.header.stamp = rospy.Time.now()
  goal.target_pose.pose.position.x = x_pos
  goal.target_pose.pose.position.y = y_pos
  goal.target_pose.pose.orientation = Quaternion(*(quaternion_from_euler(0, 0, orient, 
                                                                         axes='sxyz')))

  # Send the goal and sleep while the goal is followed
  # The sleep prevents a "stop and go" behavior and instead
  # calculates the vectors as the kart is moving
  client.send_goal(goal)
  time.sleep(UPDATE_RATE)


"""
This should be the main function running, as it receives
laserscan messages it should interrupt and perform the operations
on the vector field.
"""
def laserscan_listener():
    try:
      rospy.init_node("move_base_sequence", anonymous=True)
      rospy.Subscriber("top/scan", LaserScan, callback, queue_size=1)
      rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation Complete.")


'''
if __name__ == "__main__":
    laserscan_listener()
'''
