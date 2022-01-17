#!/usr/bin/env python

import rospy

from sbg_driver.msg import SbgImuData
from sbg_driver.msg import SbgEkfEuler
from sbg_driver.msg import SbgMag

from sensor_msgs.msg import Imu, MagneticField

from geometry_msgs.msg import Quaternion, Vector3

from tf.transformations import quaternion_from_euler

import numpy as np

class sbg_to_imu:

    def __init__(self):
        self.orientation_status = False
        self.inertial_status = False
        self.mag_status = False

        self.orientation = Quaternion()

        self.imu_msg = Imu()
        self.mag_msg = MagneticField()

        self.imu_publisher = rospy.Publisher("imu/data_raw", Imu, queue_size=50)
        self.mag_publisher = rospy.Publisher("imu/mag", MagneticField, queue_size=50)

        self.msg_seq = 0

        self.msg_listener()

    def gravity_filter(self, accel_vector):
        pure_quat = np.array([accel_vector.x, accel_vector.y, accel_vector.z, 0])
        orientation_quat = np.array([self.orientation.x, self.orientation.y, self.orientation.z, self.orientation.w])
        inv_orientation_quat = np.array([orientation_quat[0] * -1, orientation_quat[1] * -1, orientation_quat[2] * -1, orientation_quat[3]])
        rotated_vector = np.multiply(inv_orientation_quat, pure_quat, orientation_quat)

        rotated_vector[2] += 9.81
        filtered_acceleration = Vector3()

        filtered_acceleration.x = rotated_vector[0]
        filtered_acceleration.y = rotated_vector[1]
        filtered_acceleration.z = rotated_vector[2]

        print(filtered_acceleration)

        return filtered_acceleration

    def ekf_euler_callback(self, ekf_euler_msg):
        #self.imu_msg.orientation = Quaternion(*(quaternion_from_euler(0, 0, ekf_euler_msg.angle.z, axes='sxyz')))
        #self.orientation = Quaternion(*(quaternion_from_euler(ekf_euler_msg.angle.x, ekf_euler_msg.angle.y, ekf_euler_msg.angle.z, axes='sxyz')))

        self.orientation_status = True

        if self.orientation_status and self.inertial_status and self.mag_status:
            self.send_message()

    def imu_data_callback(self, imu_data_msg):
        self.imu_msg.linear_acceleration = imu_data_msg.accel
        self.imu_msg.angular_velocity = imu_data_msg.gyro

        self.inertial_status = True

        if self.orientation_status and self.inertial_status and self.mag_status:
            self.send_message()

    def mag_callback(self, mag_msg):
        self.mag_msg.magnetic_field = mag_msg.mag

        self.mag_status = True

        if self.orientation_status and self.inertial_status and self.mag_status:
            self.send_message()

    def send_message(self):
        self.imu_msg.header.seq = self.msg_seq
        self.imu_msg.header.stamp = rospy.Time.now()
        self.imu_msg.header.frame_id = "imu_link"

        self.mag_msg.header.seq = self.msg_seq
        self.mag_msg.header.stamp = rospy.Time.now()
        self.mag_msg.header.frame_id = "imu_link"

        #self.imu_msg.linear_acceleration = self.gravity_filter(self.imu_msg.linear_acceleration)

        for i in range(3):
            #self.imu_msg.orientation_covariance[i + (i * 3)] = 0.01
            self.imu_msg.angular_velocity_covariance[i + (i * 3)] = 0.01
            self.imu_msg.linear_acceleration_covariance[i + (i * 3)] = 0.01
            self.mag_msg.magnetic_field_covariance[i + (i * 3)] = 0.01

        self.imu_publisher.publish(self.imu_msg)
        self.mag_publisher.publish(self.mag_msg)

        self.orientation_status = False
        self.inertial_status = False
        self.mag_status = False

    def msg_listener(self):
        # Initialize the imu publisher node
        rospy.init_node("sbg_to_imu_publisher", anonymous=True)

        # Create subscribers to the two message types
        rospy.Subscriber("ekf_euler", SbgEkfEuler, self.ekf_euler_callback, queue_size=1)
        rospy.Subscriber("imu_data", SbgImuData, self.imu_data_callback, queue_size=1)
        rospy.Subscriber("mag", SbgMag, self.mag_callback, queue_size=1)

        # Spin and wait for new messages
        rospy.spin()

if __name__ == "__main__":
    try:
        sbg_to_imu()
    except rospy.ROSInterruptException:
        rospy.loginfo("IMU Data Interrupted.")
