/*
 * amp_serial.h
 *
 *  Created on: Mar 13, 2019
 *      Author: David Pimley
 */

#ifndef SRC_AMP_SERIAL_H_
#define SRC_AMP_SERIAL_H_

#include "util.h"
#include "amp_err.h"

#define AMP_SERIAL_MAX_PKT_SIZE  0xFF                       // max packet size

#define AMP_SERIAL_START_PKT     0x02                       // start packet byte
#define AMP_SERIAL_STOP_PKT      0x03                       // end packet byte

#define AMP_SERIAL_CONFIG_BAUD   9600                       // The Configuration Baud Rate
#define AMP_SERIAL_CONFIG_BITS   8                          // The Configuration Bits per Byte
#define AMP_SERIAL_CONFIG_PARY   SP_PARITY_NONE             // The Configuration Parity Bits
#define AMP_SERIAL_CONFIG_STOP   1                          // The Configuration Stop Bits
#define AMP_SERIAL_CONFIG_CTS    SP_CTS_INVALID             // The Configuration CTS
#define AMP_SERIAL_CONFIG_DSR    SP_DSR_INVALID             // The Configuration DSR
#define AMP_SERIAL_CONFIG_DTR    SP_DTR_INVALID             // The Configuration DTR
#define AMP_SERIAL_CONFIG_RTS    SP_RTS_INVALID             // The Configuration RTS
#define AMP_SERIAL_CONFIG_XST    SP_XONXOFF_INVALID         // The Configuration XON/XOFF


#define AMP_SERIAL_RC_CTRL_MAX_VEL 10.0f                    // Max Remote Control Velocity in (m/s)
#define AMP_SERIAL_RC_CTRL_MAX_ANG 20.0f                    // Max Remote Control Steering Angle (rads)
#define AMP_MAX_VEL 10.0f
#define AMP_MIN_VEL 0.0f
#define AMP_MAX_ANG 5.0f
#define AMP_MIN_ANG 0.0f

#define PS3_BUTTON_SELECT            0
#define PS3_BUTTON_STICK_LEFT        1
#define PS3_BUTTON_STICK_RIGHT       2
#define PS3_BUTTON_START             3
#define PS3_BUTTON_CROSS_UP          4
#define PS3_BUTTON_CROSS_RIGHT       5
#define PS3_BUTTON_CROSS_DOWN        6
#define PS3_BUTTON_CROSS_LEFT        7
#define PS3_BUTTON_REAR_LEFT_2       8
#define PS3_BUTTON_REAR_RIGHT_2      9
#define PS3_BUTTON_REAR_LEFT_1       10
#define PS3_BUTTON_REAR_RIGHT_1      11
#define PS3_BUTTON_ACTION_TRIANGLE   12
#define PS3_BUTTON_ACTION_CIRCLE     13
#define PS3_BUTTON_ACTION_CROSS      14
#define PS3_BUTTON_ACTION_SQUARE     15
#define PS3_BUTTON_PAIRING           16

#define PS3_AXIS_STICK_LEFT_LEFTWARDS    0
#define PS3_AXIS_STICK_LEFT_UPWARDS      1
#define PS3_AXIS_STICK_RIGHT_LEFTWARDS   2
#define PS3_AXIS_STICK_RIGHT_UPWARDS     3
#define PS3_AXIS_BUTTON_CROSS_UP         4
#define PS3_AXIS_BUTTON_CROSS_RIGHT      5
#define PS3_AXIS_BUTTON_CROSS_DOWN       6
#define PS3_AXIS_BUTTON_CROSS_LEFT       7
#define PS3_AXIS_BUTTON_REAR_LEFT_2      8
#define PS3_AXIS_BUTTON_REAR_RIGHT_2     9
#define PS3_AXIS_BUTTON_REAR_LEFT_1      10
#define PS3_AXIS_BUTTON_REAR_RIGHT_1     11
#define PS3_AXIS_BUTTON_ACTION_TRIANGLE  12
#define PS3_AXIS_BUTTON_ACTION_CIRCLE    13
#define PS3_AXIS_BUTTON_ACTION_CROSS     14
#define PS3_AXIS_BUTTON_ACTION_SQUARE    15
#define PS3_AXIS_ACCELEROMETER_LEFT      16
#define PS3_AXIS_ACCELEROMETER_FORWARD   17
#define PS3_AXIS_ACCELEROMETER_UP        18
#define PS3_AXIS_GYRO_YAW                19


/*
 * Contains all packet ids possible from the NVIDIA Jetson TX2 On-Board
 * Computer to the TI Micro. Any packet id not found in the below enumeration
 * results in a kill kart function to stop the motion of the kart.
 */
typedef enum amp_serial_pkt_id_t {
    AMP_SERIAL_CONTROL = 0xF1,                              // packet of steering & translational floats
    //AMP_SERIAL_DAC_CONTROL,                                 // packet for control of DAC
    //AMP_SERIAL_PWM_CONTROL,                                 // packet for control of PWM
    AMP_SERIAL_DEFAULT = 0xF4,					    // packet for default mode of kart
    AMP_SERIAL_ENABLE = 0xF0,                               // packet to enter enabled state (Power to the MC/servo)
    AMP_SERIAL_DRIVE = 0xF5,                                       // packet to enter drive forward state (will follow throttle/steering commands in FWD direction)
    //AMP_SERIAL_DRIVE_REV,                                   // packet to enter drive reverse state
    AMP_SERIAL_KILL_KART = 0xF2                             // packet for stopping all motion
} amp_serial_pkt_id_t;

/*
 * States of the Current Serial Port. The point of creating the states
 * is to ensure that the software is only sending or receiving at one
 * time.
 */
typedef enum amp_serial_state_t {
    AMP_SERIAL_STATE_IDLE,
    AMP_SERIAL_STATE_RX,
    AMP_SERIAL_STATE_TX
} amp_serial_state_t;

/*
 * State of the current control of the system. Will be used as a hard toggle
 * i.e. starts in autonomous and can be controlled by rc if desired but can't be
 * switched back
 */
typedef enum amp_control_state_t {
    AMP_CONTROL_AUTONOMOUS,
    AMP_CONTROL_REMOTE
} amp_control_state_t;

// DECLARE PACKET DATA STRUCTURES
typedef struct amp_serial_pkt_control_t {
    int v_angle;                                          // vehicle steering angle
    int v_speed;                                          // vehicle speed
} amp_serial_pkt_control_t;

typedef struct amp_serial_pkt_dac_t {
    uint8_t d_voltage;                                        // voltage to set to the DAC
} amp_serial_pkt_dac_t;

typedef struct amp_serial_pkt_pwm_t {
    uint16_t p_freq;                                        // frequency to set to the PWM
} amp_serial_pkt_pwm_t;

typedef struct amp_serial_pkt_kill_t {
    uint8_t k_flag;                                         // whether or not to kill the kart
} amp_serial_pkt_kill_t;


/*
 *  Taken from libserialport.h to configure the serial port in a more
 *  concise manner.
 */
struct sp_port_config {
	int baudrate;                                           // Baudrate of the Serial Port
	int bits;                                               // Bits to Send per Byte
	enum sp_parity parity;                                  // Parity Checking of the Port
	int stopbits;                                           // Number of Stopbits
	enum sp_rts rts;
	enum sp_cts cts;
	enum sp_dtr dtr;
	enum sp_dsr dsr;
	enum sp_xonxoff xon_xoff;
};

/*
 *  Structure of the AMP serial packet. The size of the packet is limited
 *  to 0xFF bytes. All packets must start with the hex 0x02 & 0x03 respectively.
 */
typedef struct amp_serial_pkt_t {
    amp_serial_pkt_id_t id;                                 // packet id
    uint8_t size;                                           // packet size
    uint8_t msg[AMP_SERIAL_MAX_PKT_SIZE];                   // contains the data of the packet
    uint8_t crc;                                            // cyclical redundancy check (2's complement)
    uint32_t timeout;                                       // timeout for receiving the packet
} amp_serial_pkt_t;


// FUNCTION DECLARATIONS --------------------------------------------------
amp_err_code_t amp_serial_jetson_initialize(sp_port * _port);

void amp_serial_jetson_config_port(sp_port * _port, sp_port_config _config);

void amp_serial_jetson_check_port(sp_port * _port, sp_port_config _config);

//void key_cmd_callback(const geometry_msgs::Twist::ConstPtr& msg);

//void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg);

amp_err_code_t amp_serial_jetson_tx_pkt(amp_serial_pkt_t * pkt, int * size);

void amp_serial_jetson_build_packet(amp_serial_pkt_t * pkt, uint8_t * s_data);

amp_err_code_t amp_serial_jetson_tx_byte(uint8_t * s_byte);

amp_err_code_t amp_serial_jetson_rx_pkt(amp_serial_pkt_t * pkt, int bytes);

uint8_t amp_serial_jetson_rebuild_packet(amp_serial_pkt_t * pkt, uint8_t * s_buf);

amp_err_code_t amp_serial_jetson_rx_byte(uint8_t * s_byte);

int float_to_int(float max, float min, float num);

void amp_serial_jetson_enable_kart();

void amp_serial_jetson_enable_drive();

void amp_serial_jetson_enable_default();

int check(enum sp_return result, amp_err_code_t amp_err);

void end_program(amp_err_code_t amp_err);

const char *parity_name(enum sp_parity parity);

#endif /* SRC_AMP_SERIAL_H_ */
