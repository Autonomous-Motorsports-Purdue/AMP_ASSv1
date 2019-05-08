/*
 * amp_dac.c
 *
 *  Created on: Mar 13, 2019
 *      Author: David Pimley
 */

#include "amp_dac.h"

volatile struct DAC_REGS* DAC_PTR[4] = {0x0, &DacaRegs, &DacbRegs, &DaccRegs};

/* FUNCTION ---------------------------------------------------------------
 * amp_err_code_t amp_dac_initialize()
 *
 * Initializes the current DAC to the specified defines above
 */
amp_err_code_t amp_dac_initialize() {
    // Enable the Write Protection
    EALLOW;

    // DAC Reference Control Register
    DAC_PTR[AMP_DAC_NUM]->DACCTL.bit.DACREFSEL = AMP_DAC_REFERENCE_VREF;

    // DAC Enable Output
    DAC_PTR[AMP_DAC_NUM]->DACOUTEN.bit.DACOUTEN = 0x0001;

    // Initialize DAC Output Register
    DAC_PTR[AMP_DAC_NUM]->DACVALS.all = 0x0000;

    // Disable the Write Protection
    EDIS;

    return AMP_ERROR_NONE;
}


/* FUNCTION ---------------------------------------------------------------
 * amp_err_code_t amp_dac_set_raw(uint16_t d_val)
 *
 * Sets the DAC to a raw 12 Bit Integer
 */
amp_err_code_t amp_dac_set_raw(uint16_t d_val) {
    // Check for Bounds (12 Bit DAC)
    if (d_val > AMP_DAC_VALUE_MAX) {
        return AMP_DAC_ERROR_BOUNDS;
    }

    // Set the DAC Value (Mask off Lower 12 Bits (JIC))
    DAC_PTR[AMP_DAC_NUM]->DACVALS.all = d_val & 0x0FFF;

    return AMP_ERROR_NONE;
}


/* FUNCTION ---------------------------------------------------------------
 * amp_err_code_t amp_dac_set_voltage(float d_voltage)
 *
 * Sets the DAC to the voltage specified.
 *
 * IMPORTANT: This function uses pre-specified values for the amplifier
 * and the max and min voltages. If the amplifier for the DAC changes these
 * constants must also change.
 */
amp_err_code_t amp_dac_set_voltage(float d_voltage) {
    // Declare & Initialize Local Variables
    float scale_factor = 0.0f;          // Used to scale dac_val to correct number
    uint16_t dac_val = 0;               // 12 Bit DAC Value used to set the DAC
    amp_err_code_t fn_ret;          // Return Variable for Called Functions

    // Check for Bounds (VOLTAGE_MIN & VOLTAGE_MAX)
    if (d_voltage > AMP_DAC_VOLTAGE_MAX || d_voltage < AMP_DAC_VOLTAGE_MIN) {
        return AMP_DAC_ERROR_BOUNDS;
    }

    // Perform Scaling to Voltage Value
    scale_factor = d_voltage / (AMP_DAC_VOLTAGE_MAX - AMP_DAC_VOLTAGE_MIN);
    dac_val = (uint16_t)(scale_factor * AMP_DAC_VALUE_MAX);

    // Set DAC & Check Errors
    if(!(fn_ret = amp_dac_set_raw(dac_val))) {
        return fn_ret;
    }

    return AMP_ERROR_NONE;
}
