/*
 * serialization.cpp
 * Used for accessing functions used in amp_serial_jetson files
 */

#ifndef __SERIAL_H_
#define __SERIAL_H_

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

class serial {

	public:
		/*
		 * FUNCTION: 
		 * 
		 * amp_err_code_t amp_serial_jetson_initialize(sp_port * _port)
		 *
		 * initializes any given port
		 */
		 amp_err_code_t amp_serial_jetson_initialize();

		/*
		 * FUNCTION:
		 * 
		 * void amp_serial_jetson_config_port(sp_port * _port, sp_port_config _config)
		 *
		 * writes the configurations for any given port
		 */
		void amp_serial_jetson_config_port(sp_port * _port, sp_port_config _config);

		/*
		 * FUNCTION:
		 * 
		 * vvoid amp_serial_jetson_check_port(sp_port * _port, sp_port_config _config)
		 *
		 * checks the configurations for any given port
		 */
		void amp_serial_jetson_check_port(sp_port * _port, sp_port_config _config);

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
