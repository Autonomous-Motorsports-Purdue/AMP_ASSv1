/*
 * amp_serial_jetson.cpp
 * 
 * Created on: Apr 18, 2019
 *     Author: David Pimley
 */

// Standard Defines
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>

// ROS Defines
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

// External Libraries
#include "libserialport-0.1.0/libserialport.h"

// User Defined Libraries / Headers
#include "amp_err.h"
#include "amp_serial_jetson.h"


using namespace std;

// Global Variables Regarding the Serial Port
const char* s_port_name = "/dev/ttyUSB0";                  // Name of the Serial Port
amp_serial_state_t s_port_state = AMP_SERIAL_STATE_IDLE;    // Current State of the Serial Port
struct sp_port * s_port = NULL;                             // Serial Port Handle
struct sp_port_config s_config;                             // Configuration of the Serial Port

// Global Variables Concerning State of Control
amp_control_state_t amp_control_state = AMP_CONTROL_AUTONOMOUS;


int main(int argc, char** argv) {
    ofstream file1 ("rx.txt", ios::out | ios::app | ios::text);
    ofstream file2 ("tx.txt", ios::out | ios::app | ios::text);
    // Initialize the Serial Port
    amp_serial_jetson_initialize();

    //amp_serial_jetson_enable_default();

    // Set the kart to the enable state
    amp_serial_jetson_enable_kart();

    // Set the kart to the drive state
    //amp_serial_jetson_enable_drive();

    while(true) {
      // Declare & Initialize Local Variables
      amp_serial_pkt_t s_pkt;                                 // Full Serial Packet
      amp_serial_pkt_control_t c_pkt;                         // Control Data Packet

      // Create Control Packet
      c_pkt.v_speed = 10; //msg->linear.x;
      c_pkt.v_angle = 5; //msg->angular.z;

      // Create Full Serial Packet
      s_pkt.id = AMP_SERIAL_CONTROL;
      s_pkt.size = 0xE2;

      // Copy From the Control Packet to the Serial Packet
      memcpy(s_pkt.msg, &c_pkt, sizeof(amp_serial_pkt_control_t));

      // Send the Packet
      amp_serial_jetson_tx_pkt(&s_pkt);

      printf("Sending Packet...\n");
    }
    file.close();
    return EXIT_SUCCESS;
}

void key_cmd_callback(const geometry_msgs::Twist::ConstPtr& msg) {
    // Declare & Initialize Local Variables
    amp_serial_pkt_t s_pkt;                                 // Full Serial Packet
    amp_serial_pkt_control_t c_pkt;                         // Control Data Packet

    // Create Control Packet
    c_pkt.v_speed = (unsigned char)msg->linear.x;
    c_pkt.v_angle = (unsigned char)msg->angular.z;

    // Create Full Serial Packet
    s_pkt.id = AMP_SERIAL_CONTROL;
    s_pkt.size = sizeof(amp_serial_pkt_control_t);

    // Copy From the Control Packet to the Serial Packet
    memcpy(s_pkt.msg, &c_pkt, sizeof(amp_serial_pkt_control_t));

    // Send the Packet
    amp_serial_jetson_tx_pkt(&s_pkt);

    printf("Sending Packet...\n");

    return;
}

void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg) {
    // Declare & Initialize Local Variables
    uint8_t translational_velocity = msg->linear.x;           // Translational Velocity Command
    uint8_t drive_angle = msg->angular.z;                     // Steering Angle Command
    amp_serial_pkt_t s_pkt;                                 // Full Serial Packet
    amp_serial_pkt_control_t c_pkt;                         // Control Data Format

    // Check Current Status of the Car's Control (RC / Autonomous)
    if (AMP_CONTROL_REMOTE == amp_control_state) {
        return;
    }

    // Create Control Packet
    c_pkt.v_speed = translational_velocity;
    c_pkt.v_angle = drive_angle;

    // Create Full Serial Packet
    s_pkt.id = AMP_SERIAL_CONTROL;
    s_pkt.size = sizeof(amp_serial_pkt_control_t);

    // Copy From the Control Packet to the Serial Packet
    memcpy(s_pkt.msg, &c_pkt, sizeof(amp_serial_pkt_control_t));

    // Send the Packet
    amp_serial_jetson_tx_pkt(&s_pkt);

    return;
}

/*
 * FUNCTION: 
 * 
 * amp_err_code_t amp_serial_jetson_initialize()
 *
 * initializes the port defined in amp_serial_jetson.h
 */
 amp_err_code_t amp_serial_jetson_initialize() {
    // Declare & Initialize Local Variables

    printf("Initializing Serial Protocol...\n");

    // Get the Port Handle by the Passed Name
    if (SP_OK != (sp_get_port_by_name(s_port_name, &s_port))) {
        printf("ERROR: Could not find specified serial port!\n");
        return AMP_SERIAL_ERROR_INIT;
    }

    // Open the Serial Port based on the Previous Handle
    if (SP_OK != (sp_open(s_port, (enum sp_mode)(SP_MODE_READ | SP_MODE_WRITE)))) {
        return AMP_SERIAL_ERROR_INIT;
    }

    // Set the Configuration Parameters
    s_config.baudrate   =  AMP_SERIAL_CONFIG_BAUD;
    s_config.bits       =  AMP_SERIAL_CONFIG_BITS;
    s_config.parity     =  AMP_SERIAL_CONFIG_PARY;
    s_config.stopbits   =  AMP_SERIAL_CONFIG_STOP;
    s_config.cts        =  AMP_SERIAL_CONFIG_CTS;
    s_config.dsr        =  AMP_SERIAL_CONFIG_DSR;
    s_config.dtr        =  AMP_SERIAL_CONFIG_DTR;
    s_config.rts        =  AMP_SERIAL_CONFIG_RTS;
    s_config.xon_xoff   =  AMP_SERIAL_CONFIG_XST;

    if (SP_OK != (sp_set_config(s_port, &s_config))) {
        return AMP_SERIAL_ERROR_INIT;
    }

    printf("Successfully opened port (%s) at %d\n", s_port_name , AMP_SERIAL_CONFIG_BAUD);

    return AMP_ERROR_NONE;
}

/*
 * FUNCTION:
 *
 * amp_err_code_t amp_serial_jetson_tx_pkt(amp_serial_pkt_t * pkt)
 * 
 * takes in an arbitrary packet and sends the data from the 
 * initialized serial port.
 */
amp_err_code_t amp_serial_jetson_tx_pkt(amp_serial_pkt_t * pkt) {
    // Declare & Initialize Local Variables
    uint8_t s_data[AMP_SERIAL_MAX_PKT_SIZE];                // Contains Raw Data of Packet
    uint8_t s_pos = 0;                                      // Current Position of Data Array
    sp_return err_ret;                                      // return code from sp functions
    uint8_t c_crc = 0;                                      // Used to calculate the current CRC
    int i;                                                  // Iteration Variable

    if (pkt == NULL) {
        printf("ERROR: Unable to fill out packet with NULL pointer\n");
        return AMP_SERIAL_ERROR_TX;
    }

    // Start Sending Data By Sending STX
    s_data[s_pos++] = (uint8_t)AMP_SERIAL_START_PKT;

    // Send the ID of the Packet
    s_data[s_pos++] = (uint8_t)pkt->id;
    c_crc += pkt->id & 0xFF;

    // Send the Size of the Packet
    s_data[s_pos++] = (uint8_t)pkt->size;
    c_crc += pkt->size & 0xFF;

    // Sned the Data of the Packet
    for (i = 0; i < pkt->size; i++) {
        s_data[s_pos++] = (uint8_t)pkt->msg[i];
        c_crc += pkt->msg[i] & 0xFF;
    }

    // Send the CRC of the Previous Packet
    pkt->crc = c_crc;
    s_data[s_pos++] = (uint8_t)pkt->crc;

    // Finish off by sending the ETX    
    s_data[s_pos++] = (uint8_t)AMP_SERIAL_STOP_PKT;

    // Wait for data to be sent out of the port
    while (s_port_state != AMP_SERIAL_STATE_IDLE || sp_output_waiting(s_port) > 0) {
    }

    // Indicate that the Serial Port is now Transmitting
    s_port_state = AMP_SERIAL_STATE_TX;

    // Send the Packet
    if (s_pos != (err_ret = sp_nonblocking_write(s_port, (const void *)s_data, s_pos * sizeof(uint8_t)))) {
        printf("ERROR: Unable to write data to serial port in amp_serial_jetson_tx_pkt, error code: %d\n", (int)err_ret);
        return AMP_SERIAL_ERROR_TX;
    }

    // Indicate that the Serial Port is now Free
    s_port_state = AMP_SERIAL_STATE_IDLE;

    return AMP_ERROR_NONE;
}

amp_err_code_t amp_serial_jetson_rx_pkt(amp_serial_pkt_t * pkt) {
    // Declare & Initialize Local Variables
    uint8_t s_pos = 0;                                      // Current Position of Data Array
    uint8_t c_crc = 0;                                      // Used to calculate the current CRC
    uint8_t s_buf = 0;                                      // Used for single byte reads i.e. STX, ETX

    sp_return err_ret;                                      // return code from sp functions
    
    int i;                                                  // Iteration Variable

    // Verify Input Parameters and the Serial Port
    if (pkt == NULL) {
        printf("ERROR: Unable to fill out packet with NULL pointer\n");
        return AMP_SERIAL_ERROR_RX;
    }

    // Verify the Current State of the Serial Port
    while (s_port_state != AMP_SERIAL_STATE_IDLE) {
    }

    // Verify that there is Data to read
    if (sp_input_waiting(s_port) == 0) {
        printf("ERROR: No Incoming Data\n");
        return AMP_ERROR_NONE;
    }

    // Only Change the Serial Port's State if an STX has been received
    if (sizeof(uint8) != (err_ret = sp_nonblocking_read(s_port, (void *)s_buf, sizeof(uint8)))) {
        printf("ERROR: Unable to chage port's serial state, error code: %d\n", (int)err_ret);
        return AMP_SERIAL_ERROR_RX;
    }

    if (AMP_SERIAL_START_PKT == s_buf) {
        s_port_state = AMP_SERIAL_STATE_RX;
    } else {
        return AMP_ERROR_NONE;
    }

    // Receive the ID from the Serial Port
    if (sizeof(uint8) != (err_ret = sp_nonblocking_read(s_port, (void *)(pkt->id), sizeof(uint8)))) {
        printf("ERROR: Unable to recieve packet id: %d\n", (int)err_ret);
        return AMP_SERIAL_ERROR_RX;
    }
    
    c_crc += pkt->id & 0xFF;

    // Receive the Size from the Serial Port
    if (sizeof(uint8) != (err_ret = sp_nonblocking_read(s_port, (void *)pkt->size, sizeof(uint8)))) {
        printf("ERROR: Unable to recieve packet size: %d\n", (int)err_ret);
        return AMP_SERIAL_ERROR_RX;
    }

    c_crc += pkt->size & 0xFF;

    // Receive the Data from the Incoming Packet
    if (pkt->size != (err_ret = sp_nonblocking_read(s_port, (void *)pkt->msg, pkt->size))) {
        printf("ERROR: Unable to recieve packet message: %d\n", (int)err_ret);
        return AMP_SERIAL_ERROR_RX;
    }

    for (i = 0; i < pkt->size; i++) {
        c_crc += pkt->msg[i] & 0xFF;
    }

    // Receive the CRC from the Incoming Packet
    if (sizeof(uint8) != (err_ret = sp_nonblocking_read(s_port, (void *)pkt->crc, sizeof(uint8)))) {
        printf("ERROR: Unable to recieve packet crc: %d\n", (int)err_ret);
        return AMP_SERIAL_ERROR_RX;
    }

    if (pkt->crc != c_crc) {
        printf("ERROR: Packet crc does not match\n");
        return AMP_SERIAL_ERROR_CRC;
    }

    // Verify that we have received the ETX before changing serial port state
    if (sizeof(uint8) != (err_ret = sp_nonblocking_read(s_port, (void *)s_buf, sizeof(uint8)))) {
        printf("ERROR: Unable to recieve ETX for serial state exit: %d\n", (int)err_ret);
        return AMP_SERIAL_ERROR_RX;
    }

    s_port_state = AMP_SERIAL_STATE_IDLE;

    if (s_buf != AMP_SERIAL_STOP_PKT) {
        printf("ERROR: No stop instruction\n");
        return AMP_SERIAL_ERROR_RX_NO_STOP;
    }

    return AMP_ERROR_NONE;
}

void amp_serial_jetson_enable_kart() {
    // Declare & Initialize Local Variables
    amp_serial_pkt_t t_pkt;

    printf("Enabling Kart through Packet ID\n");

    // Enable the kart
    t_pkt.id = AMP_SERIAL_ENABLE;
    t_pkt.size = 1;
    t_pkt.msg[0] = 0xFF;

    amp_serial_jetson_tx_pkt(&t_pkt);
}

void amp_serial_jetson_enable_drive() {
    // Declare & Initialize Local Variables
    amp_serial_pkt_t t_pkt;

    // Put into drive mode
    printf("Putting Kart in Drive Mode\n");

    t_pkt.id = AMP_SERIAL_DRIVE;
    t_pkt.size = 1;
    t_pkt.msg[0] = 0xFF;

    amp_serial_jetson_tx_pkt(&t_pkt);
}

void amp_serial_jetson_enable_default() {
    // Declare & Initialize Local Variables
    amp_serial_pkt_t t_pkt;

    // Put into default mode
    printf("Putting Kart in Default Mode\n");

    t_pkt.id = AMP_SERIAL_DEFAULT;
    t_pkt.size = 1;
    t_pkt.msg[0] = 0xFF;

    amp_serial_jetson_tx_pkt(&t_pkt);
}

void print_rx_packet(amp_serial_pkt_t * pkt, ofstream file)
{
    int i;
    if(file.is_open())
    {
        file << "Packet\n"
        file << pkt.id;
        file << "\n";
        file << pkt.size;
        file << "\n";
        for (i = 0; i < pkt.size; i++)
        {
            file << pkt.msg[i];
            file << "\n";
        }
        file << pkt.crc;
        file << "\n\n";
    }
}

void print_tx_packet(uint8_t * s_data, uint8_t size, ofstream file)
{
    int i;
    if(file.is_open())
    {
        file << "Packet\n"
        for(i = 0; i < size; i++)
        {
            file << s_data[i];
            file << "\n"
        }
        file << "\n"
    }
}
