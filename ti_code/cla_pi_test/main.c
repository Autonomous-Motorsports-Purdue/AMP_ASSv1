/**
 * CMPSS_test:  This project tests two peripherals the eQEP peripheral and cla functionality
 *
 *
 */

//Preprocessor Directives
//Includes
#include "F28x_Project.h"

//Defines

//Function Declarations
void amp_eQEP_initialize(void);



int main(void) {
    //Initialize System Control
    InitSysCtrl();

    //Initialize GPIO
    InitGpio();

    //Clear all interrupts and initialize PIE vector table
    DINT;

    //Initialize PIE control registers to default state
    InitPieCtrl();

    //Disable CPU interrupts and clear CPU interrupt flags
    IER = 0x0000;
    IFR = 0x0000;

    //Initialize Comparator to desired settings



    return 0;
}

/* FUNCTION ---------------------------------------------------------------
 * void amp_eQEP_initialize(void)
 *
 * Initializes the eQEP peripheral with the desired settings
 */
void amp_eQEP_initialize(void) {

    /*  Quadrature Decoder Control
     *      -Quadrature count mode
     *      -Count Rising and Falling edge
     *      -Not swapped
     *      -No polarity changes
     */
    EQep1Regs.QDECCTL = 0x0000;

    /*  Quadrature Capture Control
     *      -Enable eQEP capture unit
     *      -CAPCLK = SYSCLKOUT/128
     *      -UPEVNT = QCLK/1
     */
    EQep1Regs.QCAPCTL = 0x8070;



}
