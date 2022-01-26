import rospy
import time
import serial
import math
import ctypes
import sys
import struct

import subprocess

from geometry_msgs.msg import Twist, Vector3

"""
Constants concering the configuration of the serial port
"""

# Linux Serial Port, May be Something different on Windows (i.e. COM3)
LINUX_SERIAL_PORT = '/dev/ttyUSB0'

# Baudrate for sending / receiving
BAUDRATE = 4800

# Bytesize of Data
BYTESIZE = serial.EIGHTBITS

# No Parity Checking
PARITY = serial.PARITY_NONE

# One Stop Bit
STOPBITS = serial.STOPBITS_ONE

# Timeout for Reading Bytes (seconds)
READ_TIMEOUT = 10

class ti_comm():
    def __init__(self, ser):

        """
        Enumeration values for use in message construction
        """

        # Create Flags for Message Choice to be used by data_to_msg()
        self.AMP_SERIAL_MAX_PKT_SIZE = 0xFF

        self.AMP_SERIAL_START_PKT = 0x02
        self.AMP_SERIAL_STOP_PKT = 0x03

        self.AMP_SERIAL_CONTROL = 0x00
        self.AMP_SERIAL_DAC_CONTROL = 0x01
        self.AMP_SERIAL_PWM_CONTROL = 0x02
        self.AMP_SERIAL_KILL_KART = 0xFF

        self.serial_port = ser


        # Start the transmission
        self.cmd_vel_listener()


    """
    Format the given data value into either a DAC message
    or a PWM Message using the given 'd' or 'f' chars at the end
    of the message.
    
    If the given parameter is 
    
    IMPORTANT: The Serial Message is being parsed by a /r carriage
    return. Must end the serial message with the carriage return.
    
    The Input message is assumed to be in a type int
    """
    def control_msg(self, vel, ang):
        # Verify the msg sent as input is a type of int


        # CRC Calculation
        crc_calc = ctypes.c_ubyte(0)

        # Declare and Intialize Output Variable
        msg_out = []

        # Format the Message Start
        msg_out.append(self.AMP_SERIAL_START_PKT)

        # Message ID
        msg_out.append(self.AMP_SERIAL_CONTROL)

        crc_calc.value = (crc_calc.value + self.AMP_SERIAL_CONTROL) & 0xFF

        # Message Size
        msg_out.append(0x08)

        crc_calc.value = (crc_calc.value + 0x08) & 0xFF

        # Add Data to the Message Packet
        msg_out.append(vel)
        msg_out.append(ang)

        crc_calc.value = self.calculate_crc_data([vel, ang], crc_calc.value)

        # Add CRC Byte
        msg_out.append(crc_calc.value)

        # Add ETX Byte
        msg_out.append(self.AMP_SERIAL_STOP_PKT)

        return msg_out

    """
    Send the message over UART through the specified port
    """
    def send_UART_msg(self, ser, msg):
        try:
            bytes_written = ser.write(msg.encode('utf-8'))
        except serial.SerialTimeoutException as err:
            print "Serial Write has Timed Out."
            raise

        # Check the number of bytes written to the serial port
        if bytes_written != len(msg):
            print "Bytes Dropped in send_UART_msg:" + msg

    """
    Define the callback function for the cmd_vel topic messages.
    """
    def cmd_vel_callback(self, cmd):
        # Parse cmd_vel values into the correct objects
        translational_velocity = ctypes.c_float(cmd.linear.x)
        drive_angle = ctypes.c_float(cmd.angular.z)

        # Format the Values into the Correct Message
        translational_msg = self.control_msg(translational_velocity, drive_angle)

        # Send each of the messages through the Serial Port
        for byte in translational_msg:
            self.send_UART_msg(self.serial_port, byte)

    """
    Create a rospy subscriber to read in the cmd_vel values that are output
    through ROS & move_base.
    """
    def cmd_vel_listener(self):
        rospy.init_node("cmd_vel_listener", anonymous=True)
        rospy.Subscriber("cmd_vel", Twist, self.cmd_vel_callback)
        rospy.spin()


    def calculate_crc_data(self, data, crc):
        for val in data:
            ba = bytearray(struct.pack('f', val))
            for byte in ba:
                crc = (crc + byte) & 0xFF

        return crc


"""
Initialize the Serial Port with the Values
Indicated on the 'Defines' Above
"""
def init_port():
    if not verify_port():
        print "Stopping Script.\n"

    try:
        ser = serial.Serial(
            port=LINUX_SERIAL_PORT,
            baudrate=BAUDRATE,
            bytesize=BYTESIZE,
            parity=PARITY,
            stopbits=STOPBITS,
            timeout=READ_TIMEOUT
        )
    except ValueError as v_err:
        print("Current Parameters are out of range. Please re-init with different values.\n")
        raise
    except serial.SerialException as s_err:
        print("Unable to Configure Serial Port.\n")
        raise

    # Verify the Serial Port is open
    if (ser.isOpen() == False):
        ser.open()

    # Return the Serial Object if Successful
    return ser

"""
Verify that a serial port exists on the system
"""
def verify_port():
    bash_cmd = "ls " + LINUX_SERIAL_PORT
    try:
        subprocess.check_call(bash_cmd, shell=True)
    except subprocess.CalledProcessError:
        print "Unable to Find Specified Serial Port.\n"
        return False

    return True

if __name__ == "__main__":
    ser = init_port()

    try:
        ti_comm(ser)
    except rospy.ROSInterruptException:
        rospy.loginfo("Serial Communication Interrupted")


