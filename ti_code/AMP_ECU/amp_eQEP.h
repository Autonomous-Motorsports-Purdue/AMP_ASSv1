/*
 * amp_eQEP.h
 *
 *  Created on: May 14, 2019
 *      Author: Tommy Krause
 */

#ifndef AMP_EQEP_H_
#define AMP_EQEP_H_

#include "F28x_Project.h"
#include "util.h"

#include "amp_err.h"

// FUNCTION DECLARATIONS --------------------------------------------------

/* FUNCTION ---------------------------------------------------------------
 * amp_err_code_t amp_eQEP_initialize()
 *
 * Initializes the eQEP module to the specified settings
 */
amp_err_code_t amp_eQEP_initialize();

/* FUNCTION ---------------------------------------------------------------
 * amp_err_code_t amp_eQEP_serviceSpeed()
 *
 * This function checks to see if an upevent has occured. If one has, the
 * function updates the measured speed variable
 */
amp_err_code_t amp_eQEP_serviceSpeed();

#endif /* AMP_EQEP_H_ */
