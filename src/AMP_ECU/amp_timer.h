/*
 * amp_tim.h
 *
 *  Created on: Apr 10, 2019
 *      Author: David
 */

#ifndef SRC_AMP_TIMER_H_
#define SRC_AMP_TIMER_H_

#include "F28x_Project.h"
#include "util.h"

#include "amp_err.h"

#define AMP_TIMER_PERIOD      100    // (us)
#define AMP_TIMER_FREQ        200     // (mHz)

//Timer Variables
extern uint16_t            intr_count;


/* FUNCTION ---------------------------------------------------------------
 * amp_err_code_t amp_tim_initialize()
 *
 * Initializes the current TIM to the specified defines above
 */
amp_err_code_t amp_timer_initialize();


/* FUNCTION ---------------------------------------------------------------
 * amp_err_code_t amp_timer_start()
 *
 * Starts the timer counter
 */
amp_err_code_t amp_timer_start();

/* FUNCTION ---------------------------------------------------------------
 * amp_err_code_t amp_timer_stop()
 *
 * Stops the timer counter
 */
amp_err_code_t amp_timer_stop();

#endif /* SRC_AMP_TIM_H_ */
