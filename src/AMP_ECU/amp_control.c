/*
 * amp_control.c
 *
 *  Created on: May 14, 2019
 *      Author: Tommy Krause
 */

#include "amp_control.h"

#define DELTA_T 10              // 10 ms update rate

//Control Variables
extern float    spd_meas;       //measured speed from eQEP module
//extern float    spd_str;      //commanded speed from JETSON
extern float    spd_err_sum;    //running sum of error
extern float    trq_dbl_str;    //raw output of PI loop
extern float    trq_str;        //satured output of PI loop


/* FUNCTION ---------------------------------------------------------------
 * amp_err_code_t amp_control_loop()
 *
 * This function executes the a PI loop
 */
amp_err_code_t amp_control_loop(float spd_str) {
    //Calculate Error
    spd_error = spd_str - spd_meas;

    //Integrate
    spd_error_sum = sped_error_sum + (v_qd_error * DELTA_T);

    //PI expression
    trq_dbl_str = PROPORTIONAL * spd_error + INTEGRAL * spd_error_sum;

    //Saturation
    if(trq_dbl_str > UP_LIM_SAT) {
        trq_str = UP_LIM_SAT;
    } else if(trq_dbl_str < LOW_LIM_SAT) {
        trq_str = LOW_LIM_SAT;
    } else {
        trq_str = trq_dbl_str;
    }

    //Set DAC voltage
    amp_dac_set_voltage(trq_str);
}


