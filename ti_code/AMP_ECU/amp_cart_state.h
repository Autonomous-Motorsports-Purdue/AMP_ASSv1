/*
 * amp_cart_state.h
 *
 *  Created on: Apr 18, 2019
 *      Author: thomaskrause
 */

#ifndef AMP_CART_STATE_H_
#define AMP_CART_STATE_H_

/*
 * Contains the possible cart states
 */
typedef enum amp_cart_state_t
{
    AMP_CART_STATE_DEFAULT    =   0,
    AMP_CART_STATE_ENABLED    =   1,
    AMP_CART_STATE_DRIVE      =   2
} amp_cart_state_t;

#endif /* AMP_CART_STATE_H_ */
