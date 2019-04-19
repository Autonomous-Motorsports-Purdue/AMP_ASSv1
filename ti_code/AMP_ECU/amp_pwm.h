/*
 * amp_pwm.h
 *
 *  Created on: Mar 13, 2019
 *      Author: David Pimley
 */

#ifndef SRC_AMP_PWM_H_
#define SRC_AMP_PWM_H_

#include "F28x_Project.h"
#include "util.h"

#include "amp_err.h"

#define AMP_PWM_TBCTR_BASE_VAL 25000U   // Represents 1 KHz
#define AMP_PWM_TBCTR_BASE_FREQ 1000U   // 1 KHz

#define AMP_PWM_TBCTR_CENTER_VAL  8333U // 3 KHz
#define AMP_PWM_TBCTR_CENTER_FREQ 3000U // 3 KHz

#define AMP_PWM_FREQ_MIN 1000U          // Minimum Frequency (Hz)
#define AMP_PWM_FREQ_MAX 5000U          // Maximum Frequency (Hz)

#define AMP_PWM_TBCTR_MIN 5000U         // Minimum Value, Represents 5 KHz
#define AMP_PWM_TBCTR_MAX 25000U        // Maximum Value, Represents 1 KHz

#define AMP_PWM_DUTY_CYCLE 0.5f         // Duty Cycle, For Square Wave

/* FUNCTION ---------------------------------------------------------------
 * amp_err_code_t amp_pwm_initialize()
 *
 * Initializes the current PWM to the specified defines above
 */
amp_err_code_t amp_pwm_initialize();

/* FUNCTION ---------------------------------------------------------------
 * amp_err_code_t amp_pwm_set_raw(float p_duty, uint16_t p_timer)
 *
 * Sets the PWM to a RAW Value
 */
amp_err_code_t amp_pwm_set_raw(float p_duty, uint16_t p_timer);

/* FUNCTION ---------------------------------------------------------------
 * amp_err_code_t amp_pwm_set_freq(uint16_t p_freq)
 *
 * Initializes the current PWM to the specified defines above
 */
amp_err_code_t amp_pwm_set_freq(uint16_t p_freq);

#endif /* SRC_AMP_PWM_H_ */
