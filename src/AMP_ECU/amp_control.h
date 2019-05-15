/*
 * amp_control.h
 *
 *  Created on: May 14, 2019
 *      Author: Kai Strubel
 */

#ifndef AMP_CONTROL_H_
#define AMP_CONTROL_H_

#include "util.h"
#include "F28x_Project.h"
#include "amp_err.h"
#include "amp_dac.h"

#define PROPORTIONAL        2
#define INTEGRAL            0.1
//#define ANTI_WINDUP
#define UP_LIM_SAT          1
#define LOW_LIM_SAT         0

/*#define DELTA_T 10              // 10 ms update rate

//Control Variables
extern float    spd_meas;       //measured speed from eQEP module
extern float    spd_str;      //commanded speed from JETSON
extern float    spd_err_sum;    //running sum of error
extern float    trq_dbl_str;    //raw output of PI loop
extern float    trq_str;        //satured output of PI loop
float           spd_err = 0;*/



/* FUNCTION ---------------------------------------------------------------
 * amp_err_code_t amp_control_loop()
 *
 * This function executes the a PI loop
 */
amp_err_code_t amp_control_loop();


#endif /* AMP_CONTROL_H_ */
