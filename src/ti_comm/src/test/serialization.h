/*
 * serialization.h
 * Used for accessing functions used in amp_serial_jetson files
 */

#ifndef __SERIALIZATION_H_
#define __SERIALIZATION_H_

// Standard Defines
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <iostream>
#include <queue>
#include <vector>
#include <fstream>
#include <tuple>


// ROS Defines
//#include "ros/ros.h"
//#include "geometry_msgs/Twist.h"

// External Libraries
#include <libserialport.h>

// User Defined Libraries / Headers
#include "amp_err.h"
#include "amp_serial_jetson.h"

class serialization {
	
	public:
		/*
		 * FUNCTION:
		 *
		 * amp_err_code_t amp_serial_jetson_tx_pkt(amp_serial_pkt_t * pkt, int * size)
		 * 
		 * takes in an arbitrary packet and sends the data from the 
		 * initialized serial port.
		 */
		amp_err_code_t amp_serial_jetson_tx_pkt(amp_serial_pkt_t * pkt, int * size); 
		
		/*
		 * FUNCTION:
		 *
		 * void amp_serial_jetson_build_packet(amp_serial_pkt_t * pkt, uint8_t * s_data)
		 * 
		 * builds the data buffer from an arbitrary packet for transmission
		 */
		void amp_serial_jetson_build_packet(amp_serial_pkt_t * pkt, uint8_t * s_data);

		/*
		 * FUNCTION:
		 *
		 * amp_err_code_t amp_serial_jetson_tx_byte(uint8_t * s_byte)
		 * 
		 * byte by byte transmission for debugging
		 */
		amp_err_code_t amp_serial_jetson_tx_byte(uint8_t * s_byte);

		/*
		 * FUNCTION:
		 *
		 * amp_err_code_t amp_serial_jetson_rx_pkt(amp_serial_pkt_t * pkt, int bytes)
		 * 
		 * receiver for packets from arduino 
		 */
		amp_err_code_t amp_serial_jetson_rx_pkt(amp_serial_pkt_t * pkt, int bytes);

		/*
		 * FUNCTION:
		 *
		 * uint8_t amp_serial_jetson_rebuild_packet(amp_serial_pkt_t * pkt, uint8_t * s_buf)
		 * 
		 * rebuilds packet from received buffer and returns crc for confirmation
		 */
		uint8_t amp_serial_jetson_rebuild_packet(amp_serial_pkt_t * pkt, uint8_t * s_buf);

		/*
		 * FUNCTION:
		 *
		 * amp_err_code_t amp_serial_jetson_rx_byte(uint8_t * s_byte)
		 * 
		 * byte by byte reciever for debugging
		 */
		amp_err_code_t amp_serial_jetson_rx_byte(uint8_t * s_byte);

		// HELPER FUNCTIONS

		/*
		 * FUNCTION:
		 *
		 * const char *parity_name(enum sp_parity parity)
		 * 
		 * returns the parity state of the buffer
		 */
		const char *parity_name(enum sp_parity parity);

		/*
		 * FUNCTION:
		 *
		 * void end_program(amp_err_code_t amp_err)
		 * 
		 * clean termination program for handling serial comm errors
		 */
		void end_program(amp_err_code_t amp_err);

		/*
		 * FUNCTION:
		 *
		 * int check(enum sp_return result, amp_err_code_t amp_err)
		 * 
		 * error handling function for serial communication
		 */
		int check(enum sp_return result, amp_err_code_t amp_err);

		void amp_serial_jetson_enable_kart();

		void amp_serial_jetson_enable_drive();

		void amp_serial_jetson_enable_default();

		/*
		 * FUNCTION:
		 *
		 * void argparse(int &argc, char *argv[], string &filename, float &delay) {
		 * 
		 * Parse input args
		 */
		void argparse(int &argc, char *argv[], string &filename, float &delay);

};

#endif
