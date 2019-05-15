/*
 * amp_control.h
 *
 *  Created on: May 14, 2019
 *      Author: Kai Strubel
 */

#ifndef AMP_CONTROL_H_
#define AMP_CONTROL_H_

#include "util.h"
#include "amp_err.h"

#define PROPORTIONAL        1
#define INTEGRAL            6
//#define ANTI_WINDUP
#define UP_LIM_SAT          2
#define LOW_LIM_SAT         0

float spd_error     = 0;



/* FUNCTION ---------------------------------------------------------------
 * amp_err_code_t amp_control_loop()
 *
 * This function executes the a PI loop
 */
amp_err_code_t amp_control_loop(float spd_str);


#endif /* AMP_CONTROL_H_ */
