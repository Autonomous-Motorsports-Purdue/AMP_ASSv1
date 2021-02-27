/*
 * amp_serial_jetson.cpp
 * 
 * Created on: Apr 18, 2019
 *     Author: David Pimley
 */

// Standard Defines
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>

// ROS Defines
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

// External Libraries
#include <libserialport.h>

// User Defined Libraries / Headers
#include "amp_err.h"
#include "amp_serial_jetson.h"

using namespace std;

// Global Variables Regarding the Serial Port
const char* port_name = "/dev/ttyUSB0";                  // Name of the Serial Port
amp_serial_state_t port_state = AMP_SERIAL_STATE_IDLE;    // Current State of the Serial Port
struct sp_port * port = NULL;                             // Serial Port Handle
struct sp_port_config config;                             // Configuration of the Serial Port

// Global Variables Concerning State of Control
amp_control_state_t amp_control_state = AMP_CONTROL_AUTONOMOUS;

//Debug Intialization
#if defined(DEBUG) || defined(DEBUG_TX) || defined(DEBUG_RX)
FILE * fptr1 = fopen("debug.txt", "w");
#endif

#ifdef DEBUG_TX
FILE * fptr2 = fopen("debug_tx.txt", "w");
#endif

#ifdef DEBUG_RX
FILE * fptr3 = fopen("debug_rx.txt", "w");
#endif

void dummy_cmd_callback(const geometry_msgs::Twist::ConstPtr& msg) {
		ROS_INFO("cmd_vel speed in x dir: [%d]", (int)(msg->linear.x));
}

int main(int argc, char** argv) {

    // Global Configuration Parameters
    config.baudrate   =  AMP_SERIAL_CONFIG_BAUD;
    config.bits       =  AMP_SERIAL_CONFIG_BITS;
    config.parity     =  AMP_SERIAL_CONFIG_PARY;
    config.stopbits   =  AMP_SERIAL_CONFIG_STOP;
    config.cts        =  AMP_SERIAL_CONFIG_CTS;
    config.dsr        =  AMP_SERIAL_CONFIG_DSR;
    config.dtr        =  AMP_SERIAL_CONFIG_DTR;
    config.rts        =  AMP_SERIAL_CONFIG_RTS;
    config.xon_xoff   =  AMP_SERIAL_CONFIG_XST; 

    // Initialize the Serial Port
    amp_serial_jetson_initialize(port);

    //amp_serial_jetson_enable_default();

    // Set the kart to the enable state
    //amp_serial_jetson_enable_kart();

    // Set the kart to the drive state
    //amp_serial_jetson_enable_drive();


    // Start the ROS Node
    ros::init(argc, argv, "cmd_vel_listener");

    // Create a Handle and have it Subscribe to the Command Vel Messages
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("cmd_vel", 1000, cmd_vel_callback);
    //ros::Subscriber sub = n.subscribe("cmd_vel", 10, key_cmd_callback);
		// TODO(ihagedo): Replace call to the dummy callback with the real one once
		//                testing with MCU is complete.
    //ros::Subscriber dummy_sub = n.subscribe("cmd_vel", 10, dummy_cmd_callback);

    // Spin as new Messages come in
    ros::spin();

    return EXIT_SUCCESS;
}

void key_cmd_callback(const geometry_msgs::Twist::ConstPtr& msg) {
    // Declare & Initialize Local Variables
    amp_serial_pkt_t s_pkt;                                 // Full Serial Packet
    amp_serial_pkt_control_t c_pkt;                         // Control Data Packet
    int size;

    // Create Control Packet
    c_pkt.v_speed = float_to_int(AMP_MAX_VEL, AMP_MIN_VEL, msg->linear.x);
    c_pkt.v_angle = float_to_int(AMP_MAX_ANG, AMP_MIN_ANG, msg->angular.z);

    // Create Full Serial Packet
    s_pkt.id = AMP_SERIAL_CONTROL;
    s_pkt.size = sizeof(amp_serial_pkt_control_t);

    // Copy From the Control Packet to the Serial Packet
    memcpy(s_pkt.msg, &c_pkt, sizeof(amp_serial_pkt_control_t));

    // Send the Packet
    amp_serial_jetson_tx_pkt(&s_pkt, &size);

    printf("Sending Packet...\n");

    return;
}

void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg) {
    // Declare & Initialize Local Variables
    float translational_velocity = msg->linear.x;           // Translational Velocity Command
    float drive_angle = msg->angular.z;                     // Steering Angle Command
    amp_serial_pkt_t s_pkt;                                 // Full Serial Packet
    amp_serial_pkt_control_t c_pkt;                         // Control Data Format
    int size;

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
    amp_serial_jetson_tx_pkt(&s_pkt, &size);

    return;
}

/*
 * FUNCTION: 
 * 
 * amp_err_code_t amp_serial_jetson_initialize(sp_port * _port)
 *
 * initializes any given port
 */
 amp_err_code_t amp_serial_jetson_initialize(sp_port * _port) {
    // Declare & Initialize Local Variables

    #ifdef DEBUG
    fprintf(fptr1, "Initializing Serial Protocol...\n");
    fprintf(fptr1, "Looking for port %s.\n", port_name);
    #endif    
    // Get the Port Handle by the Passed Name
    check(sp_get_port_by_name(port_name, &port), AMP_SERIAL_ERROR_INIT);

    // Display some basic information about the port.
    #ifdef DEBUG
    fprintf(fptr1, "Port name: %s\n", sp_get_port_name(port));
    fprintf(fptr1, "Description: %s\n", sp_get_port_description(port));

    // Open the Serial Port based on the Previous Handle
    fprintf(fptr1, "Opening port...\n");
    #endif
    check(sp_open(port, SP_MODE_READ_WRITE), AMP_SERIAL_ERROR_INIT);

    #ifdef DEBUG
    fprintf(fptr1, "Setting configurations...\n");
    #endif
    amp_serial_jetson_config_port(port, config);

    #ifdef DEBUG    
    fprintf(fptr1, "Checking configurations...\n");
    #endif
    amp_serial_jetson_check_port(port, config);

    return AMP_ERROR_NONE;
}

/*
 * FUNCTION: 
 * 
 * void amp_serial_jetson_config_port(sp_port * _port, sp_port_config _config)
 *
 * writes the configurations for any given port
 */
void amp_serial_jetson_config_port(sp_port * _port, sp_port_config _config) {
    
    // Set the Configuration Parameters
    _config.baudrate   =  AMP_SERIAL_CONFIG_BAUD;
    _config.bits       =  AMP_SERIAL_CONFIG_BITS;
    _config.parity     =  AMP_SERIAL_CONFIG_PARY;
    _config.stopbits   =  AMP_SERIAL_CONFIG_STOP;
    _config.cts        =  AMP_SERIAL_CONFIG_CTS;
    _config.dsr        =  AMP_SERIAL_CONFIG_DSR;
    _config.dtr        =  AMP_SERIAL_CONFIG_DTR;
    _config.rts        =  AMP_SERIAL_CONFIG_RTS;
    _config.xon_xoff   =  AMP_SERIAL_CONFIG_XST;

    check(sp_set_config(_port, &_config), AMP_SERIAL_ERROR_INIT);
    
}

/*
 * FUNCTION: 
 * 
 * vvoid amp_serial_jetson_check_port(sp_port * _port, sp_port_config _config)
 *
 * checks the configurations for any given port
 */
void amp_serial_jetson_check_port(sp_port * _port, sp_port_config _config) {
    
    int baudrate;
    int bits;
    int stopbits;
    struct sp_port_config * check_config;
    enum sp_parity parity;
    
    check(sp_new_config(&check_config), AMP_SERIAL_ERROR_INIT);
    check(sp_get_config(_port, check_config), AMP_SERIAL_ERROR_INIT);
    
    check(sp_get_config_baudrate(check_config, &baudrate), AMP_SERIAL_ERROR_INIT);
    check(sp_get_config_bits(check_config, &bits), AMP_SERIAL_ERROR_INIT);
    check(sp_get_config_stopbits(check_config, &stopbits), AMP_SERIAL_ERROR_INIT);
    check(sp_get_config_parity(check_config, &parity), AMP_SERIAL_ERROR_INIT);
    sp_free_config(check_config);

    #ifdef DEBUG
    fprintf(fptr1, "Baudrate: %d, data bits: %d, parity: %s, stop bits: %d\n", baudrate, bits, parity_name(parity), stopbits);
    fprintf(fptr1, "Successfully opened port (%s) at %d\n", port_name , baudrate);

    if(_config.baudrate != baudrate)
    {
	fprintf(fptr1, "Baudrate mismatch\n");
    }
    if(_config.bits != bits)
    {
	fprintf(fptr1, "Bits mismatch\n");
    }
    if(_config.parity != parity)
    {
	fprintf(fptr1, "Parity mismatch\n");
    }
    if(_config.stopbits != stopbits)
    {
	fprintf(fptr1, "Stopbit mismatch\n");
    }
    #endif
}

/*
 * FUNCTION:
 *
 * amp_err_code_t amp_serial_jetson_tx_pkt(amp_serial_pkt_t * pkt, int * size)
 * 
 * takes in an arbitrary packet and sends the data from the 
 * initialized serial port.
 */
amp_err_code_t amp_serial_jetson_tx_pkt(amp_serial_pkt_t * pkt, int * size) {
    // Declare & Initialize Local Variables
    uint8_t s_data[AMP_SERIAL_MAX_PKT_SIZE] = {0};          // Contains Raw Data of Packet
    uint8_t * s_buf;

    // Wait for data to be sent out of the port
    while (port_state != AMP_SERIAL_STATE_IDLE || sp_output_waiting(port) > 0) {

	#ifdef DEBUG_TX
	fprintf(fptr1, "Waiting...\n");
        fprintf(fptr2, "Waiting...\n");
	#endif
    }

    // Indicate that the Serial Port is now Transmitting
    port_state = AMP_SERIAL_STATE_TX;

    amp_serial_jetson_build_packet(pkt, s_data);
    s_buf = (s_data + sizeof(uint8_t));
    
    // Send the Packet
    check(sp_nonblocking_write(port, (const void *)s_buf, s_data[0] * sizeof(uint8_t)), AMP_SERIAL_ERROR_TX);
    *size = s_data[0];

    // Indicate that the Serial Port is now Free
    port_state = AMP_SERIAL_STATE_IDLE;    

    #ifdef DEBUG_TX
    int i;
    for(i = 0; i < s_data[0]; i++) {
        fprintf(fptr1, "tx[%d]: %u\n", i, s_buf[i]);
        fprintf(fptr2, "tx[%d]: %u\n", i, s_buf[i]);
    }
    #endif

    return AMP_ERROR_NONE;
}

/*
 * FUNCTION:
 *
 * void amp_serial_jetson_build_packet(amp_serial_pkt_t * pkt, uint8_t * s_data)
 * 
 * builds the data buffer from an arbitrary packet for transmission
 */
void amp_serial_jetson_build_packet(amp_serial_pkt_t * pkt, uint8_t * s_data)
{
    uint8_t s_pos = 1;                                      // Current Position of Data Array
    uint8_t c_crc = 0;                                      // Used to calculate the current CRC
    int i; 
    
    // Start Byte
    s_data[s_pos++] = (uint8_t)AMP_SERIAL_START_PKT;

    // ID Byte
    s_data[s_pos++] = (uint8_t)pkt->id;
    c_crc += pkt->id & 0xFF;
    #ifdef DEBUG_TX
    fprintf(fptr1, "Packet contents\n");
    fprintf(fptr2, "Packet contents\n");
    fprintf(fptr1, "id: %u\n", (uint8_t)pkt->id);
    fprintf(fptr2, "id: %u\n", (uint8_t)pkt->id);
    #endif

    // Size Byte
    s_data[s_pos++] = 0xE0 + (uint8_t)pkt->size;
    c_crc += pkt->size & 0xFF;
    #ifdef DEBUG_TX
    fprintf(fptr1, "size: %u\n", (0xE0 + (uint8_t)pkt->size));
    fprintf(fptr2, "size: %u\n", (0xE0 + (uint8_t)pkt->size));
    #endif

    // Data Byte
    for (i = 0; i < pkt->size; i++) {
        s_data[s_pos++] = (uint8_t)pkt->msg[i];
        c_crc += pkt->msg[i] & 0xFF;
        #ifdef DEBUG_TX
	fprintf(fptr1, "msg: %u\n", (uint8_t)pkt->msg[i]);
	fprintf(fptr2, "msg: %u\n", (uint8_t)pkt->msg[i]);
        #endif
    }

    // Send the CRC of the Previous Packet
    pkt->crc = c_crc;
    s_data[s_pos++] = (uint8_t)pkt->crc;
    #ifdef DEBUG_TX
    fprintf(fptr1, "crc: %u\n", (uint8_t)pkt->crc);
    fprintf(fptr2, "crc: %u\n", (uint8_t)pkt->crc);
    #endif

    // Finish off by sending the ETX    
    s_data[s_pos] = (uint8_t)AMP_SERIAL_STOP_PKT;

    s_data[0] = s_pos;
}

/*
 * FUNCTION:
 *
 * amp_err_code_t amp_serial_jetson_tx_byte(uint8_t * s_byte)
 * 
 * byte by byte transmission for debugging
 */
amp_err_code_t amp_serial_jetson_tx_byte(uint8_t * s_byte) {

    // Wait for data to be sent out of the port
    while (port_state != AMP_SERIAL_STATE_IDLE || sp_output_waiting(port) > 0) {
    }

    // Indicate that the Serial Port is now Transmitting
    port_state = AMP_SERIAL_STATE_TX;
    
    // Send the Packet
    check(sp_nonblocking_write(port, (const void *)s_byte, sizeof(uint8_t)), AMP_SERIAL_ERROR_TX);

    // Indicate that the Serial Port is now Free
    port_state = AMP_SERIAL_STATE_IDLE;

    #ifdef DEBUG_TX
    fprintf(fptr1, "tx: %u\n", *s_byte);
    fprintf(fptr2, "tx: %u\n", *s_byte);
    #endif

    return AMP_ERROR_NONE;
}

/*
 * FUNCTION:
 *
 * amp_err_code_t amp_serial_jetson_rx_pkt(amp_serial_pkt_t * pkt, int bytes)
 * 
 * receiver for packets from arduino 
 */
amp_err_code_t amp_serial_jetson_rx_pkt(amp_serial_pkt_t * pkt, int bytes) {
    // Declare & Initialize Local Variables
    uint8_t c_crc;                                            // Used to calculate the current CRC
    uint8_t * s_buf = static_cast<uint8_t *>(malloc(bytes));  // Used for single byte reads i.e. STX, ETX
    

    // Verify Input Parameters and the Serial Port
    if (pkt == NULL) {
        #ifdef DEBUG_RX
        fprintf(fptr1, "ERROR: Unable to fill out packet with NULL pointer\n");
        fprintf(fptr3, "ERROR: Unable to fill out packet with NULL pointer\n");
	#endif
        return AMP_SERIAL_ERROR_RX;
    }

    // Verify the Current State of the Serial Port
    while (port_state != AMP_SERIAL_STATE_IDLE) {
    }

    // Verify that there is Data to read
    while(sp_input_waiting(port) == 0) {
        #ifdef DEBUG_RX
        fprintf(fptr1, "Waiting...\n");
        fprintf(fptr3, "Waiting...\n");
	#endif
    }

    port_state = AMP_SERIAL_STATE_RX;

    // Read in the data
    check(sp_nonblocking_read(port, s_buf, sizeof(uint8) * bytes), AMP_SERIAL_ERROR_RX);

    port_state = AMP_SERIAL_STATE_IDLE;

    // Verify that the input has no errors
    if (s_buf[0] != AMP_SERIAL_START_PKT) {
        #ifdef DEBUG_RX
        fprintf(fptr1, "ERROR: No Start Byte Found\n");
        fprintf(fptr3, "ERROR: No Start Byte Found\n");
	#endif
	return AMP_SERIAL_ERROR_RX_NO_START;
    }

    if (s_buf[bytes-1] != AMP_SERIAL_STOP_PKT) {
        #ifdef DEBUG_RX
        fprintf(fptr1, "ERROR: No Stop Byte Found\n");
        fprintf(fptr3, "ERROR: No Stop Byte Found\n");
	#endif
	return AMP_SERIAL_ERROR_RX_NO_STOP;
    }

    c_crc = amp_serial_jetson_rebuild_packet(pkt, s_buf);
    if(c_crc != pkt->crc) {
        #ifdef DEBUG_RX
        fprintf(fptr1, "ERROR: CRC Mismatch\n");
        fprintf(fptr3, "ERROR: CRC Mismatch\n");
	#endif
	return AMP_SERIAL_ERROR_CRC;
    }

    #ifdef DEBUG_RX
    int i;
    for(i = 0; i < bytes; i++)
    {
	fprintf(fptr1, "rx[%d]: %u\n", i, s_buf[i]);
	fprintf(fptr3, "rx[%d]: %u\n", i, s_buf[i]);
    }
    #endif

    // Free all allocated space 
    free(s_buf);
    return AMP_ERROR_NONE;
}

/*
 * FUNCTION:
 *
 * uint8_t amp_serial_jetson_rebuild_packet(amp_serial_pkt_t * pkt, uint8_t * s_buf)
 * 
 * rebuilds packet from received buffer and returns crc for confirmation
 */
uint8_t amp_serial_jetson_rebuild_packet(amp_serial_pkt_t * pkt, uint8_t * s_buf) {

    int i;
    uint8_t c_crc = 0;
    // ID from the Serial Port
    pkt->id = (amp_serial_pkt_id_t)s_buf[1];
    c_crc += pkt->id & 0xFF;
    #ifdef DEBUG_RX
    fprintf(fptr1, "Packet contents\n");
    fprintf(fptr2, "Packet contents\n");
    fprintf(fptr1, "id: %u\n", (uint8_t)pkt->id);
    fprintf(fptr2, "id: %u\n", (uint8_t)pkt->id);
    #endif

    // Size from the Serial Port
    pkt->size = s_buf[2];
    c_crc += pkt->size & 0xFF;
    #ifdef DEBUG_TX
    fprintf(fptr1, "size: %u\n", (uint8_t)pkt->size);
    fprintf(fptr2, "size: %u\n", (uint8_t)pkt->size);
    #endif

    // Data from the Incoming Packet
    for (i = 0; i < pkt->size; i++) 
    {
	pkt->msg[i] = s_buf[i+3];        
	c_crc += pkt->msg[i] & 0xFF;
        #ifdef DEBUG_TX
	fprintf(fptr1, "msg: %u\n", (uint8_t)pkt->msg[i]);
	fprintf(fptr2, "msg: %u\n", (uint8_t)pkt->msg[i]);
        #endif
    }

    // CRC from the Incoming Packet
    pkt->crc = s_buf[i+3];
    #ifdef DEBUG_TX
    fprintf(fptr1, "crc: %u\n", (uint8_t)pkt->crc);
    fprintf(fptr2, "crc: %u\n", (uint8_t)pkt->crc);
    #endif
  
    return c_crc;
}

/*
 * FUNCTION:
 *
 * amp_err_code_t amp_serial_jetson_rx_byte(uint8_t * s_byte)
 * 
 * byte by byte reciever for debugging
 */
amp_err_code_t amp_serial_jetson_rx_byte(uint8_t * s_byte) {

    // Verify the Current State of the Serial Port
    while (port_state != AMP_SERIAL_STATE_IDLE) {
    }

    // Verify that there is Data to read
    while(sp_input_waiting(port) == 0) {
        #ifdef DEBUG_RX
        fprintf(fptr1, "Waiting...\n");
        fprintf(fptr3, "Waiting...\n");
	#endif
    }

    // Read in the data
    check(sp_nonblocking_read(port, s_byte, sizeof(uint8)), AMP_SERIAL_ERROR_RX);

    port_state = AMP_SERIAL_STATE_IDLE;

    #ifdef DEBUG_RX
    fprintf(fptr1, "rx: %u\n", *s_byte);
    fprintf(fptr3, "rx: %u\n", *s_byte);
    #endif

    return AMP_ERROR_NONE;
}

/* FUNCTION:
 *
 * int float_to_int(float, max, float min, float num)
 * 
 * returns converted int from 0 to 255
 */
int float_to_int(float max, float min, float num)
{
	int val;
	
	val = (int)roundf((num-min)/(max-min)*255);

	return val;
}

/*
 * FUNCTION:
 *
 * const char *parity_name(enum sp_parity parity)
 * 
 * returns the parity state of the buffer
 */
const char *parity_name(enum sp_parity parity)
{
	switch (parity) {
	case SP_PARITY_INVALID:
		return "(Invalid)";
	case SP_PARITY_NONE:
		return "None";
	case SP_PARITY_ODD:
		return "Odd";
	case SP_PARITY_EVEN:
		return "Even";
	case SP_PARITY_MARK:
		return "Mark";
	case SP_PARITY_SPACE:
		return "Space";
	default:
		return NULL;
	}
}

/*
 * FUNCTION:
 *
 * void end_program(amp_err_code_t amp_err)
 * 
 * clean termination program for handling serial comm errors
 */
void end_program(amp_err_code_t amp_err)
{
	/* Free any structures we allocated. */
	if (&config != NULL)
		sp_free_config(&config);
	if (port != NULL)
		sp_free_port(port);

	/* Exit with the given return code. */
	exit(amp_err);
}

/*
 * FUNCTION:
 *
 * int check(enum sp_return result, amp_err_code_t amp_err)
 * 
 * error handling function for serial communication
 */
int check(enum sp_return result, amp_err_code_t amp_err)
{
	int error_code;
	char *error_message;

	switch (result) {

	case SP_ERR_ARG:
		/* When SP_ERR_ARG is returned, there was a problem with one
		 * or more of the arguments passed to the function, e.g. a null
		 * pointer or an invalid value. This generally implies a bug in
		 * the calling code. */
		printf("Error: Invalid argument.\n");
		#ifdef DEBUG
		fprintf(fptr1, "Error: Invalid argument.\n");
		#endif
		end_program(amp_err);

	case SP_ERR_FAIL:
		/* When SP_ERR_FAIL is returned, there was an error from the OS,
		 * which we can obtain the error code and message for. These
		 * calls must be made in the same thread as the call that
		 * returned SP_ERR_FAIL, and before any other system functions
		 * are called in that thread, or they may not return the
		 * correct results. */
		error_code = sp_last_error_code();
		error_message = sp_last_error_message();
		printf("Error: Failed: OS error code: %d, message: '%s'\n", error_code, error_message);
		#ifdef DEBUG
		fprintf(fptr1, "Error: Failed: OS error code: %d, message: '%s'\n", error_code, error_message);
		#endif
		/* The error message should be freed after use. */
		sp_free_error_message(error_message);
		end_program(amp_err);

	case SP_ERR_SUPP:
		/* When SP_ERR_SUPP is returned, the function was asked to do
		 * something that isn't supported by the current OS or device,
		 * or that libserialport doesn't know how to do in the current
		 * version. */
		printf("Error: Not supported.\n");
		#ifdef DEBUG
		fprintf(fptr1, "Error: Not supported.\n");
		#endif
		end_program(amp_err);

	case SP_ERR_MEM:
		/* When SP_ERR_MEM is returned, libserialport wasn't able to
		 * allocate some memory it needed. Since the library doesn't
		 * normally use any large data structures, this probably means
		 * the system is critically low on memory and recovery will
		 * require very careful handling. The library itself will
		 * always try to handle any allocation failure safely. */
		end_program(amp_err);

	case SP_OK:
	default:
		#ifdef DEBUG
		fprintf(fptr1, "Operation succeeded.\n");
		#endif
		return result;
	}
}

void amp_serial_jetson_enable_kart() {
    // Declare & Initialize Local Variables
    amp_serial_pkt_t t_pkt;
    int size;

    #ifdef DEBUG
    fprintf(fptr1, "Enabling Kart through Packet ID\n");
    #endif

    // Enable the kart
    t_pkt.id = AMP_SERIAL_ENABLE;
    t_pkt.size = 1;
    t_pkt.msg[0] = 0xFF;

    amp_serial_jetson_tx_pkt(&t_pkt, &size);
}

void amp_serial_jetson_enable_drive() {
    // Declare & Initialize Local Variables
    amp_serial_pkt_t t_pkt;
    int size;

    #ifdef DEBUG
    fprintf(fptr1, "Putting Kart in Drive Mode\n");
    #endif

    t_pkt.id = AMP_SERIAL_DRIVE;
    t_pkt.size = 1;
    t_pkt.msg[0] = 0xFF;

    amp_serial_jetson_tx_pkt(&t_pkt, &size);
}

void amp_serial_jetson_enable_default() {
    // Declare & Initialize Local Variables
    amp_serial_pkt_t t_pkt;
    int size;

    #ifdef DEBUG
    fprintf(fptr1, "Putting Kart in Default Mode\n");
    #endif

    t_pkt.id = AMP_SERIAL_DEFAULT;
    t_pkt.size = 1;
    t_pkt.msg[0] = 0xFF;

    amp_serial_jetson_tx_pkt(&t_pkt, &size);
}
