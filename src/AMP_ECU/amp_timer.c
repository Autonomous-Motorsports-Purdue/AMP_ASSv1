/*
 * amp_timer.c
 *
 *  Created on: Apr 10, 2019
 *      Author: David Pimley
 */

#include "amp_timer.h"

/* FUNCTION ---------------------------------------------------------------
 * amp_err_code_t amp_timer_initialize()
 *
 * Initializes the current timer to the specified defines above
 */
amp_err_code_t amp_timer_initialize() {
    // Set the Period of the Timer
    CpuTimer0Regs.PRD.all = (long)(AMP_TIMER_PERIOD * AMP_TIMER_FREQ) - 1;

    // Set Pre-Scale Values
    CpuTimer0Regs.TPR.all = 0x0000;
    CpuTimer0Regs.TPRH.all = 0x0000;

    // Initially Have the Timer Stopped
    CpuTimer0Regs.TCR.bit.TSS = 0x1;

    // Reset the Timer
    CpuTimer0Regs.TCR.bit.TRB = 0x1;

    // Breakpoint Configs
    CpuTimer0Regs.TCR.bit.SOFT = 0;
    CpuTimer0Regs.TCR.bit.FREE = 0;

    // Enable Interrupts
    CpuTimer0Regs.TCR.bit.TIE = 0x01;

    return AMP_ERROR_NONE;
}

/* FUNCTION ---------------------------------------------------------------
 * amp_err_code_t amp_timer_start()
 *
 * Starts the timer counter
 */
amp_err_code_t amp_timer_start() {
    CpuTimer0Regs.TCR.bit.TSS = 0x0;

    return AMP_ERROR_NONE;
}

/* FUNCTION ---------------------------------------------------------------
 * amp_err_code_t amp_timer_stop()
 *
 * Stops the timer counter
 */
amp_err_code_t amp_timer_stop() {
    CpuTimer0Regs.TCR.bit.TSS = 1;

    count = 0;

    return AMP_ERROR_NONE;
}
