/*
 * amp_service.c
 *
 *  Created on: Mar 13, 2019
 *      Author: David Pimley
 */

#include "amp_service.h"

/* FUNCTION ---------------------------------------------------------------
 * amp_err_code_t amp_service_pkt(amp_serial_pkt_t * pkt)
 *
 * Depending on the ID of the received packet, control something on the
 * microcontroller.
 */

amp_err_code_t amp_service_pkt(amp_serial_pkt_t * pkt) {
    // Declare & Initialize Local Variables
    amp_err_code_t fn_ret;              // possible error codes from other functions

    // Look at the ID of the Packet
    switch (pkt->id) {
        case AMP_SERIAL_CONTROL:
            if (!(fn_ret = amp_service_control(pkt))) {
                return fn_ret;
            }
            break;
        case AMP_SERIAL_DAC_CONTROL:
            if (!(fn_ret = amp_service_dac(pkt))) {
                return fn_ret;
            }
            break;
        case AMP_SERIAL_PWM_CONTROL:
            if (!(fn_ret = amp_service_pwm(pkt))) {
                return fn_ret;
            }
            break;
        case AMP_SERIAL_KILL_KART:
            if (!(fn_ret = amp_service_kill_kart(pkt))) {
                return fn_ret;
            }
        default:
            return AMP_ERROR_NONE;
    }

    return AMP_ERROR_NONE;
}


/* FUNCTION ---------------------------------------------------------------
 * amp_err_code_t amp_service_set_speed(float v_speed)
 *
 * Sets the speed of the kart, values input are taken as meters per second
 */
amp_err_code_t amp_service_set_speed(float v_speed) {

    //CONVERT THIS TO A PI CONTROL AFTER PROCESSING THE SIN COS ENCODER
    amp_dac_set_voltage(0.0);

    return AMP_ERROR_NONE;
}


/* FUNCTION ---------------------------------------------------------------
 * void amp_service_set_steering(float v_angle)
 *
 * Sets the steering angle of the kart, values input are taken as angles
 * in radians according to the diagram below.
 *
 *            > 0     0     < 0
 *             *      *      *
 *              *     *     *
 *               *    *    *
 *                *   *   *
 *                 *  *  *
 *                  =====
 *                  | F |
 *                  |   |
 *                  | R |
 *                  =====
 */
amp_err_code_t amp_service_set_steering(float v_angle) {

    // NEED MEASUREMENTS FOR THIS
    amp_pwm_set_freq(3000);

    return AMP_ERROR_NONE;
}


/* FUNCTION ---------------------------------------------------------------
 * amp_err_code_t amp_service_control(amp_serial_pkt_t * r_pkt)
 *
 * Controls the kart by servicing both the velocity and steering control
 * on the kart. Both values are passed into the packet at the same time.
 */
amp_err_code_t amp_service_control(amp_serial_pkt_t * r_pkt) {
    // Declare & Initialize Local Variables
    amp_err_code_t fn_ret;              // possible error codes from other functions
    amp_serial_pkt_control_t r_pkt_data;    // packet structure for data
    unsigned char i = 0;

    r_pkt_data.v_angle = 0;
    r_pkt_data.v_speed = 0;

    uint32_t _v_speed = * (uint32_t *)&r_pkt_data.v_angle;
    uint32_t _v_angle = * (uint32_t *)&r_pkt_data.v_speed;;

    // Copy the Data from the Received Packet and Cast it to the Appropriate Structure
    for (i = 0; i < 4; i++) {
        _v_angle = _v_angle | ((uint32_t)(r_pkt->msg[i] & 0xFF) << (8 * i));
    }

    for (i = 4; i < 8; i++) {
        _v_speed = _v_speed | ((uint32_t)(r_pkt->msg[i] & 0xFF) << (8 * (i - 4)));
    }

    r_pkt_data.v_angle = * (float *)&_v_angle;
    r_pkt_data.v_speed = * (float *)&_v_speed;


    // Set the Appropriate Velocity Control
    if (!(fn_ret = amp_service_set_speed(r_pkt_data.v_speed))) {
        return fn_ret;
    }

    // Set the Appropriate Steering Control
    if (!(fn_ret = amp_service_set_speed(r_pkt_data.v_angle))) {
        return fn_ret;
    }

    return AMP_ERROR_NONE;
}


/* FUNCTION ---------------------------------------------------------------
 * amp_err_code_t amp_service_dac(amp_serial_pkt_t * r_pkt)
 *
 * Sets the DAC to the appropriate value as indicated by the packet
 */
amp_err_code_t amp_service_dac(amp_serial_pkt_t * r_pkt) {
    // Declare & Initialize Local Variables
    amp_err_code_t fn_ret;              // possible error codes from other functions
    amp_serial_pkt_dac_t r_pkt_data;    // packet structure for data
    int i = 0;

    // Copy the Data from the Received Packet and Cast it to the Appropriate Structure
    r_pkt_data.d_voltage = 0;

    uint32_t _d_voltage = * (uint32_t *)&r_pkt_data.d_voltage;

    // Copy the Data from the Received Packet and Cast it to the Appropriate Structure
    for (i = 0; i < 4; i++) {
        _d_voltage = _d_voltage | ((uint32_t)(r_pkt->msg[i] & 0xFF) << (8 * i));
    }

    r_pkt_data.d_voltage = * (float *)&_d_voltage;

    // Set the Appropriate DAC Value
    if (!(fn_ret = amp_dac_set_voltage(r_pkt_data.d_voltage))) {
        return fn_ret;
    }

    return AMP_ERROR_NONE;
}


/* FUNCTION ---------------------------------------------------------------
 * amp_err_code_t amp_service_pwm(amp_serial_pkt_t * r_pkt)
 *
 * Sets the PWM to the appropriate value as indicated by the packet
 */
amp_err_code_t amp_service_pwm(amp_serial_pkt_t * r_pkt) {
    // Declare & Initialize Local Variables
    amp_err_code_t fn_ret;              // possible error codes from other functions
    amp_serial_pkt_pwm_t r_pkt_data;    // packet structure for data

    // Copy the Data from the Received Packet and Cast it to the Appropriate Structure
    r_pkt_data.p_freq = 0x0000U;

    r_pkt_data.p_freq = ((r_pkt->msg[0] & 0x00FF) << 8) | (r_pkt->msg[1] & 0x00FF);

    // Set the Appropriate PWM Value
    if (!(fn_ret = amp_pwm_set_freq(r_pkt_data.p_freq))) {
        return fn_ret;
    }

    return AMP_ERROR_NONE;
}


/* FUNCTION ---------------------------------------------------------------
 * amp_err_code_t amp_service_kill_kart(amp_serial_pkt_t * r_pkt)
 *
 * Kills the motion of the kart by stopping the motor and setting the
 * steering angle to straight
 */
amp_err_code_t amp_service_kill_kart(amp_serial_pkt_t * r_pkt) {
    // Declare & Initialize Local Variables
    amp_err_code_t fn_ret;              // possible error codes from other functions
    amp_serial_pkt_kill_t r_pkt_data;   // packet structure for data

    // Copy the Data from the Received Packet and Cast it to the Appropriate Structure
    memcpy(&r_pkt_data, r_pkt, sizeof(r_pkt_data));

    // Set the Appropriate PWM & DAC Values

    // Stop the Kart Motion
    if (r_pkt_data.k_flag) {
        if (!(fn_ret = amp_dac_set_voltage(AMP_DAC_VOLTAGE_MIN))) {
            return fn_ret;
        }

        // Center out the Steering Column
        if (!(fn_ret = amp_dac_set_voltage(AMP_PWM_TBCTR_CENTER_FREQ))) {
            return fn_ret;
        }
    }

    return AMP_ERROR_NONE;
}
