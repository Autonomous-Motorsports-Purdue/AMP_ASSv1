/*
 * amp_pwm.c
 *
 *  Created on: Mar 13, 2019
 *      Author: David Pimley
 */

#include "amp_pwm.h"

/* FUNCTION ---------------------------------------------------------------
 * amp_err_code_t amp_pwm_initialize()
 *
 * Initializes the current PWM to the specified defines above
 */
amp_err_code_t amp_pwm_initialize() {

    // Enable PWM1
    CpuSysRegs.PCLKCR2.bit.EPWM1 = 1;

    // Initialize GPIO pins for EPWM1
    InitEPwm1Gpio();

    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;
    EDIS;

    // Setup TBCLK

    // Set timer period 25000 TBCLKs
    EPwm1Regs.TBPRD = AMP_PWM_TBCTR_CENTER_VAL;

    // Phase is 0
    EPwm1Regs.TBPHS.bit.TBPHS = 0x0000;

    // Clear counter
    EPwm1Regs.TBCTR = 0x0000;


    // Set Compare values

    // Set compare A value
    EPwm1Regs.CMPA.bit.CMPA = AMP_PWM_DUTY_CYCLE * AMP_PWM_TBCTR_CENTER_VAL;

    // Setup counter mode
    // Count up and down
    EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;

    // Disable phase loading
    EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;

    // Clock ratio to SYSCLKOUT
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;

    EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;


    // Set actions

    // Set PWM1A on event A, up count
    EPwm1Regs.AQCTLA.bit.CAU = AQ_SET;

    // Clear PWM1A on event A, down count
    EPwm1Regs.AQCTLA.bit.CAD = AQ_CLEAR;

    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;
    EDIS;

    return AMP_ERROR_NONE;
}

/* FUNCTION ---------------------------------------------------------------
 * amp_err_code_t amp_pwm_set_raw(float p_duty, uint16_t p_timer)
 *
 * Sets the PWM to a RAW Value
 *
 * MAPPING:
 *
 * 25000 -> 1    kHz
 * 20000 -> 1.25 kHz
 *
 */
amp_err_code_t amp_pwm_set_raw(float p_duty, uint16_t p_timer) {
    // Check Bounds
    if (p_timer < AMP_PWM_TBCTR_MIN || p_timer > AMP_PWM_TBCTR_MAX) {
        return AMP_PWM_ERROR_BOUNDS;
    }

    // Set the Counter and Duty Cycle
    EPwm1Regs.TBPRD = p_timer;
    EPwm1Regs.CMPA.bit.CMPA = p_duty * p_timer;

    return AMP_ERROR_NONE;
}

/* FUNCTION ---------------------------------------------------------------
 * amp_err_code_t amp_pwm_set_freq(uint16_t p_freq)
 *
 * Sets the PWM Output to the specified frequency
 */
amp_err_code_t amp_pwm_set_freq(uint16_t p_freq){
    // Declare & Initialize Local Variables
    uint32_t timer_val = 0;             // Value to be sent to TBCTR
    amp_err_code_t fn_ret;          // Return Variable for Called Functions

    // Check Bounds
    if (p_freq < AMP_PWM_FREQ_MIN || p_freq > AMP_PWM_FREQ_MAX) {
        return AMP_PWM_ERROR_BOUNDS;
    }

    // Calculate Necessary PWM TBCTR Value
    timer_val = ((uint32_t)AMP_PWM_TBCTR_BASE_VAL * (uint32_t)AMP_PWM_TBCTR_BASE_FREQ) / p_freq;

    // Set the PWM TBCTR Value & Check Errors
    if (!(fn_ret = amp_pwm_set_raw(AMP_PWM_DUTY_CYCLE, timer_val & 0x0000FFFF))) {
        return fn_ret;
    }

    return AMP_ERROR_NONE;
}
