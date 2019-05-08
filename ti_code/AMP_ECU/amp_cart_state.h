/*
 * amp_cart_state.h
 *
 *  Created on: Apr 18, 2019
 *      Author: Tommy Krause
 */

#ifndef AMP_CART_STATE_H_
#define AMP_CART_STATE_H_

#include "F28x_Project.h"

/*
 * Contains the possible cart states
 */
typedef enum amp_cart_state_t
{
    AMP_CART_STATE_DEFAULT,
    AMP_CART_STATE_ENABLED,
    AMP_CART_STATE_DRIVE
} amp_cart_state_t;

#endif /* AMP_CART_STATE_H_ */
