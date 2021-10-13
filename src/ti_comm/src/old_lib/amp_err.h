/*
 * amp_err.h
 *
 *  Created on: Mar 15, 2019
 *      Author: David
 */

#ifndef SRC_AMP_ERR_H_
#define SRC_AMP_ERR_H_

/*
 * Contains the pertinent error codes regarding serial transmission.
 */
typedef enum amp_err_code_t {
    AMP_ERROR_NONE        =  1,                              // successful operation

    // Errors Regarding Serial Transmission (-1 -> -20)
    AMP_SERIAL_ERROR_RX_TIMEOUT  = -1,                       // timed out waiting for data
    AMP_SERIAL_ERROR_CRC         = -2,                       // indicates bad crc (invalid rx bytes)
    AMP_SERIAL_ERROR_RX_NO_STOP  = -3,                       // no stop byte received
    AMP_SERIAL_ERROR_RX_NO_START = -3,                       // no start byte received
    AMP_SERIAL_ERROR_INIT        = -4,                       // initialization errors
    AMP_SERIAL_ERROR_TX          = -5,                       // transmission issues
    AMP_SERIAL_ERROR_RX          = -6,                       // receive issues

    // Error Regarding PWM Module (-21 -> -40)
    AMP_PWM_ERROR_BOUNDS         = -21,                      // PWM Value out of Bounds

    // Errors Regarding DAC Module (-41 - 60)
    AMP_DAC_ERROR_BOUNDS         = -41                       // DAC Value out of Bounds (12 Bit)
} amp_err_code_t;

#endif /* SRC_AMP_ERR_H_ */
