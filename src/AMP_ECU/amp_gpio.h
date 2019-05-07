/*
 * amp_gpio.h
 *
 *  Created on: Apr 14, 2019
 *      Author: Tommy Krause
 */

#ifndef AMP_GPIO_H_
#define AMP_GPIO_H_

#include "F28x_Project.h"
#include "amp_err.h"
#include "amp_cart_state.h"

/* FUNCTION ---------------------------------------------------------------
 * amp_err_code_t amp_gpio_initialize()
 *
 * Initializes GPIO pins used by this project
 */
amp_err_code_t amp_gpio_initialize();

/* FUNCTION ---------------------------------------------------------------
 * amp_err_code_t amp_gpio_service()
 *
 * This function checks the GPIO pin outputs for a given cart state
 */
amp_err_code_t amp_gpio_service(amp_cart_state_t cart);


#endif /* AMP_GPIO_H_ */
