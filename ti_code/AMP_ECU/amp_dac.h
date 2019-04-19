/*
 * amp_dac.h
 *
 *  Created on: Mar 13, 2019
 *      Author: David Pimley
 */

#ifndef SRC_AMP_DAC_H_
#define SRC_AMP_DAC_H_

#include "F28x_Project.h"
#include "util.h"

#include "amp_err.h"

#define AMP_DAC_REFERENCE_VDAC  0
#define AMP_DAC_REFERENCE_VREF  1

#define AMP_DAC_VALUE_MAX 0x0FFF
#define AMP_DAC_VALUE_MIN 0x0000

#define AMP_DAC_VOLTAGE_MIN 0.0f
#define AMP_DAC_VOLTAGE_MAX 10.0f

#define AMP_DAC_NUM 2

// FUNCTION DECLARATIONS --------------------------------------------------

/* FUNCTION ---------------------------------------------------------------
 * amp_err_code_t amp_dac_initialize()
 *
 * Initializes the current DAC to the specified defines above
 */
amp_err_code_t amp_dac_initialize();

/* FUNCTION ---------------------------------------------------------------
 * amp_err_code_t amp_dac_set_raw(uint16_t d_val)
 *
 * Sets the DAC to a raw 12 Bit Integer
 */
amp_err_code_t amp_dac_set_raw(uint16_t d_val);

/* FUNCTION ---------------------------------------------------------------
 * amp_err_code_t amp_dac_set_voltage(float d_voltage)
 *
 * Sets the DAC to the voltage specified.
 */
amp_err_code_t amp_dac_set_voltage(float d_voltage);

#endif /* SRC_AMP_DAC_H_ */
