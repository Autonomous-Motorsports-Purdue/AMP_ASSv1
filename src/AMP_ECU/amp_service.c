/*
 * amp_service.c
 *
 *  Created on: Mar 13, 2019
 *      Author: David Pimley
 */

#include "amp_service.h"
#include "amp_cart_state.h"

extern amp_cart_state_t    cart; //state variable for the cart

extern float               spd_str; // global declared commanded speed
extern uint16_t            dir_change = 0; // flag to indicate change in direction

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
        case AMP_SERIAL_DEFAULT:
            if ((!(fn_ret = amp_service_default(pkt))))
            {
                return fn_ret;
            }
            break;
        case AMP_SERIAL_ENABLE:
            if (!(fn_ret = amp_service_enable(pkt)))
            {
                return fn_ret;
            }
            break;
        case AMP_SERIAL_DRIVE:
            if (!(fn_ret = amp_service_drive(pkt)))
            {
                 return fn_ret;
            }
            break;
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
    // Define & Initialize Local Variables
    uint32_t p_freq = AMP_PWM_TBCTR_CENTER_FREQ;

    // Input Validation
    if (v_angle < AMP_SERVICE_FREQ_MAX_ANG || v_angle > AMP_SERVICE_FREQ_MIN_ANG) {
        return AMP_PWM_ERROR_BOUNDS;
    }

    // If the angle is less than the mid (right turn)
    if (v_angle < AMP_SERVICE_FREQ_MID_ANG) {
        p_freq = ((v_angle / (float)AMP_SERVICE_FREQ_MAX_ANG) * (AMP_PWM_FREQ_MAX - AMP_PWM_TBCTR_CENTER_FREQ)) + AMP_PWM_TBCTR_CENTER_FREQ;
    // If the angle is greater than the mid (left turn)
    } else if (v_angle > AMP_SERVICE_FREQ_MID_ANG) {
        p_freq = AMP_PWM_TBCTR_CENTER_FREQ - ((v_angle / (float)AMP_SERVICE_FREQ_MIN_ANG) * (AMP_PWM_TBCTR_CENTER_FREQ - AMP_PWM_FREQ_MIN));
    }

    // Set the Frequency
    amp_pwm_set_freq(p_freq);

    return AMP_ERROR_NONE;
}

/* FUNCTION ---------------------------------------------------------------
 * amp_err_code_t amp_service_default(amp_serial_pkt_t * r_pkt)
 *
 * This function takes a packet with the enable id and alters the
 * cart state variable accordingly
 *
 */
amp_err_code_t amp_service_default(amp_serial_pkt_t * r_pkt) {

    cart = AMP_CART_STATE_DEFAULT;
    return AMP_ERROR_NONE;
}

/* FUNCTION ---------------------------------------------------------------
 * amp_err_code_t amp_service_enable(amp_serial_pkt_t * r_pkt)
 *
 * This function takes a packet with the enable id and alters the
 * cart state variable accordingly
 *
 */
amp_err_code_t amp_service_enable(amp_serial_pkt_t * r_pkt) {

    cart = AMP_CART_STATE_ENABLED;
    return AMP_ERROR_NONE;
}

/* FUNCTION ---------------------------------------------------------------
 * amp_err_code_t amp_service_drive(amp_serial_pkt_t * r_pkt)
 *
 * This function takes a packet with the drive id and alters the
 * cart state variable accordingly
 *
 */
amp_err_code_t amp_service_drive(amp_serial_pkt_t * r_pkt) {

    cart = AMP_CART_STATE_DRIVE;
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

    // Set the spd_str to the commanded speed
    spd_str = r_pkt_data.v_speed;

    // Set the Appropriate Steering Control
    if (!(fn_ret = amp_service_set_steering(r_pkt_data.v_angle))) {
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

    r_pkt_data.p_freq = (r_pkt->msg[0] & 0x00FF) | ((r_pkt->msg[1] & 0x00FF) << 8);

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
        if (!(fn_ret = amp_pwm_set_freq(AMP_PWM_TBCTR_CENTER_FREQ))) {
            return fn_ret;
        }
    }

    return AMP_ERROR_NONE;
}
