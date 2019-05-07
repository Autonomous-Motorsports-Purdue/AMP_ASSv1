/*
 * amp_interrupts.h
 *
 *  Created on: Apr 18, 2019
 *      Author: Tommy Krause
 */

#ifndef AMP_INTERRUPTS_H_
#define AMP_INTERRUPTS_H_

#include "F28x_Project.h"
#include "amp_err.h"

/* FUNCTION ---------------------------------------------------------------
 * amp_err_code_t amp_interrupts_initialize()
 *
 * Initializes interrupts
 */
amp_err_code_t amp_interrupts_initialize();

/* FUNCTION (INTERRUPT SERVICE ROUTINE)
 * interrupt void sciaRxIsr(void)
 *
 * This ISR is triggered by a character being placed in the RXBUFFER
 *
 */
interrupt void scibRxIsr(void);

/* FUNCTION (INTERRUPT SERVICE ROUTINE)
 * interrupt void sciaRxIsr(void)
 *
 * This ISR is triggered by a character being placed in the TXBUFFER
 *
 */
interrupt void scibTxIsr(void);

/* FUNCTION ---------------------------------------------------------------
 * __interrupt void cpu_timer0_isr(void)
 *
 * Timer interrupt service routine
 */
__interrupt void cpu_timer0_isr(void);

#endif /* AMP_INTERRUPTS_H_ */
