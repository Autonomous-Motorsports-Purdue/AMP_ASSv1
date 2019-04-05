/*
 * amp_service.h
 *
 *  Created on: Mar 13, 2019
 *      Author: David
 */

#ifndef SRC_AMP_SERVICE_H_
#define SRC_AMP_SERVICE_H_

#include "F28x_Project.h"
#include "util.h"

#include "amp_serial.h"
#include "amp_dac.h"
#include "amp_pwm.h"

#include "amp_err.h"

/* FUNCTION ---------------------------------------------------------------
 * amp_err_code_t amp_service_pkt(amp_serial_pkt_t * pkt)
 *
 * Depending on the ID of the received packet, control something on the
 * microcontroller.
 */
amp_err_code_t amp_service_pkt(amp_serial_pkt_t * pkt);


/* FUNCTION ---------------------------------------------------------------
 * amp_err_code_t amp_service_set_speed(float v_speed)
 *
 * Sets the speed of the kart, values input are taken as meters per second
 */
amp_err_code_t amp_service_set_speed(float v_speed);


/* FUNCTION ---------------------------------------------------------------
 * amp_err_code_t amp_service_set_steering(float v_angle)
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
amp_err_code_t amp_service_set_steering(float v_angle);


/* FUNCTION ---------------------------------------------------------------
 * amp_err_code_t amp_service_control(amp_serial_pkt_t * r_pkt)
 *
 * Controls the kart by servicing both the velocity and steering control
 * on the kart. Both values are passed into the packet at the same time.
 */
amp_err_code_t amp_service_control(amp_serial_pkt_t * r_pkt);


/* FUNCTION ---------------------------------------------------------------
 * amp_err_code_t amp_service_dac(amp_serial_pkt_t * r_pkt)
 *
 * Sets the DAC to the appropriate value as indicated by the packet
 */
amp_err_code_t amp_service_dac(amp_serial_pkt_t * r_pkt);


/* FUNCTION ---------------------------------------------------------------
 * amp_err_code_t amp_service_pwm(amp_serial_pkt_t * r_pkt)
 *
 * Sets the PWM to the appropriate value as indicated by the packet
 */
amp_err_code_t amp_service_pwm(amp_serial_pkt_t * r_pkt);


/* FUNCTION ---------------------------------------------------------------
 * amp_err_code_t amp_service_kill_kart(amp_serial_pkt_t * r_pkt)
 *
 * Kills the motion of the kart by stopping the motor and setting the
 * steering angle to straight
 */
amp_err_code_t amp_service_kill_kart(amp_serial_pkt_t * r_pkt);


#endif /* SRC_AMP_SERVICE_H_ */
