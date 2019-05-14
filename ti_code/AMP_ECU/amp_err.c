/*
 * amp_err.c
 *
 *  Created on: May 7, 2019
 *      Author: thomaskrause
 */

#include "amp_err.h"
#include "amp_dac.h"
#include "amp_pwm.h"
#include "amp_cart_state.h"

extern amp_cart_state_t cart;

void amp_err_handler(amp_err_code_t err) {
    // Declare & Initialize Local Variables

    switch(err) {
    // TODO: Implement Timeout for RX
    case AMP_SERIAL_ERROR_RX_TIMEOUT:

    // CRC Error Means Improper Serial Transmission / Reception
    case AMP_SERIAL_ERROR_CRC:

    case AMP_SERIAL_ERROR_RX_NO_STOP:

    case AMP_PWM_ERROR_BOUNDS:

    case AMP_DAC_ERROR_BOUNDS:

    default:
        // Stop the Motor by Zeroing out the DAC
        amp_dac_set_voltage(AMP_DAC_VOLTAGE_MIN);

        // Center out the Steering Column
        amp_pwm_set_freq(AMP_PWM_TBCTR_CENTER_FREQ);

        // Resets the Cart State to Default
        cart = AMP_CART_STATE_DEFAULT;

    }
}


