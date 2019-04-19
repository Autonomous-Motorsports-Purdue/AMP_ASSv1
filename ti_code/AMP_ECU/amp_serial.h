/*
 * amp_serial.h
 *
 *  Created on: Mar 13, 2019
 *      Author: David Pimley
 */

#ifndef SRC_AMP_SERIAL_H_
#define SRC_AMP_SERIAL_H_

#include "F28x_Project.h"
#include "util.h"

#include "amp_err.h"

#define AMP_SERIAL_MAX_PKT_SIZE  0xFF   // max packet size

#define AMP_SERIAL_START_PKT     0x02   // start packet byte
#define AMP_SERIAL_STOP_PKT      0x03   // end packet byte

/*
 * Contains the states that describe the UART receive state machine
 */
typedef enum amp_serial_state_t
{
    AMP_SERIAL_STATE_START_SEEK,
    AMP_SERIAL_STATE_ID_SEEK,
    AMP_SERIAL_STATE_SIZE_SEEK,
    AMP_SERIAL_STATE_DATA_SEEK,
    AMP_SERIAL_STATE_CRC_SEEK,
    AMP_SERIAL_STATE_STOP_SEEK
} amp_serial_state_t;



/*
 * Contains all packet ids possible from the NVIDIA Jetson TX2 On-Board
 * Computer to the TI Micro. Any packet id not found in the below enumeration
 * results in a kill kart function to stop the motion of the kart.
 */
typedef enum amp_serial_pkt_id_t {
    AMP_SERIAL_CONTROL,                 // packet of steering & translational floats
    AMP_SERIAL_DAC_CONTROL,             // packet for control of DAC
    AMP_SERIAL_PWM_CONTROL,             // packet for control of PWM
    AMP_SERIAL_ENABLE,                  // packet to enter enabled state  (Power to the MC/servo)
    AMP_SERIAL_DRIVE_FWD,               // packet to enter drive forward state (will follow throttle/steering commands in FWD direction)
    AMP_SERIAL_DRIVE_REV,               // packet to enter drive reverse state (will follow throttle/steering commands in REV direction)
    AMP_SERIAL_KILL_KART = 0xFF         // packet for stopping all motion
} amp_serial_pkt_id_t;

// DECLARE PACKET DATA STRUCTURES

typedef struct amp_serial_pkt_control_t {
    float v_speed;                      // vehicle speed
    float v_angle;                      // vehicle steering angle
} amp_serial_pkt_control_t;

typedef struct amp_serial_pkt_dac_t {
    float d_voltage;                    // voltage to set to the DAC
} amp_serial_pkt_dac_t;

typedef struct amp_serial_pkt_pwm_t {
    uint16_t p_freq;                    // frequency to set to the PWM
} amp_serial_pkt_pwm_t;

typedef struct amp_serial_pkt_kill_t {
    uint8_t k_flag;                     // whether or not to kill the kart
} amp_serial_pkt_kill_t;

/*
 *  Structure of the AMP serial packet. The size of the packet is limited
 *  to 0xFF bytes. All packets must start with the hex 0x02 & 0x03 respectively.
 */
typedef struct amp_serial_pkt_t {
    amp_serial_pkt_id_t id;             // packet id
    uint8_t size;                       // packet size
    uint8_t msg[AMP_SERIAL_MAX_PKT_SIZE];
                                        // contains the data of the packet
    uint8_t crc;                        // cyclical redundancy check (2's complement)
    uint32_t timeout;                   // timeout for receiving the packet
} amp_serial_pkt_t;


// FUNCTION DECLARATIONS --------------------------------------------------

/* FUNCTION ---------------------------------------------------------------
 * amp_serial_err_code_t amp_serial_initialize()
 *
 * Initializes the SCIBREG
 */
amp_err_code_t amp_serial_initialize();

/* FUNCTION ---------------------------------------------------------------
 * bool amp_serial_get_rx_buffer_status()
 *
 * Looks for character in the SCI registers. If a character is not found,
 * return false otherwise return true to indicate a character is present.
 */
bool amp_serial_get_rx_buffer_status();


/* FUNCTION ---------------------------------------------------------------
 * uint8_t amp_serial_rx_raw()()
 *
 * Returns the next byte from the rx buffer when it is ready
 */
uint8_t amp_serial_rx_raw();


/* FUNCTION ---------------------------------------------------------------
 * amp_err_code_t amp_serial_rx_pkt()()
 *
 * Builds a packet from the receive buffer. If the packet's crc is correct
 * it returns the packet, otherwise the packet is disregarded.
 */
amp_err_code_t amp_serial_rx_pkt(amp_serial_pkt_t * pkt);


#endif /* SRC_AMP_SERIAL_H_ */
