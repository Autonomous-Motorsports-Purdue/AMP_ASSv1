/**
 * CMPSS_test:  This Project tests the comparator subsystem of the TMS320F28379D microcontroller,
 *              specifically on th LAUNCHXL-F28379D
 *
 *              Comparators used: 2, 5
 *
 *              Comparator 2:
 *                  -J7 pin 69 CMPIN2P -> cosine input from sin cos quadrature encoder
 *                  -Inverting input driven by internal DAC
 *                  -DAC reference VDDA
 *                  -Shadow Register updated on SYSCLK
 *                  -No hysteresis or output filtering
 *
 *              Comparator 5:
 *                  -J7 pin 67 CMPIN5P -> cosine input from sin cos quadrature encoder
 *                  -Inverting input driven by internal DAC
 *                  -DAC reference VDDA
 *                  -Shadow Register updated on SYSCLK
 *                  -No hysteresis or output filtering
 */

//Preprocessor Directives
//Includes
#include "F28x_Project.h"

//Function Declarations
void amp_comparator_initialize(void);

//Main Loop
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
    amp_comparator_initialize();

    //Configure GPIO Pins
    GPIO_SetupPinMux(25, GPIO_MUX_CPU1, 1);
    GPIO_SetupPinMux(15, GPIO_MUX_CPU1, 6);

    while(1)
    {
        //do nothing
    }


    return 0;
}

/* FUNCTION ---------------------------------------------------------------
 * void amp_comparator_initialize(void)
 *
 * Initializes comparators 2 and 5 with desired settings
 */
void amp_comparator_initialize(void) {
    //Enable write to set of registers
    EALLOW;

    //First we enable the comparator/DAC subsystem
    Cmpss2Regs.COMPCTL.bit.COMPDACE = 1;
    Cmpss5Regs.COMPCTL.bit.COMPDACE = 1;

    //Initialize values for digital filter sample/threshold/clock pre-scale regs

    /* CTRIPH Filter Control Register
     * struct CTRIPHFILCTL_BITS {              // bits description
        Uint16 rsvd1:4;                     // 3:0 Reserved
        Uint16 SAMPWIN:5;                   // 8:4 Sample Window
        Uint16 THRESH:5;                    // 13:9 Majority Voting Threshold
        Uint16 rsvd2:1;                     // 14 Reserved
        Uint16 FILINIT:1;                   // 15 Filter Initialization Bit
    };
     */
    //
    //Maximum SAMPWIN value provides largest number of samples
    //
    Cmpss2Regs.CTRIPHFILCTL.bit.SAMPWIN = 0x1F;
    Cmpss5Regs.CTRIPHFILCTL.bit.SAMPWIN = 0x1F;

    //
    //Maximum THRESH value requires static value for entire window
    //  THRESH should be GREATER than half of SAMPWIN
    //
    Cmpss2Regs.CTRIPHFILCTL.bit.THRESH = 0x1F;
    Cmpss5Regs.CTRIPHFILCTL.bit.THRESH = 0x1F;

    Cmpss2Regs.CTRIPHFILCTL.bit.FILINIT = 1;
    Cmpss5Regs.CTRIPHFILCTL.bit.FILINIT = 1;

    /* CTRIPH Filter Clock Control Register
     * struct CTRIPHFILCLKCTL_BITS {           // bits description
    Uint16 CLKPRESCALE:10;              // 9:0 Sample Clock Prescale
    Uint16 rsvd1:6;                     // 15:10 Reserved
    };
     */
    //Maximum CLKPRESCALE value provides the most time between samples
    //
    Cmpss2Regs.CTRIPHFILCLKCTL.bit.CLKPRESCALE = 0x3FF;
    Cmpss5Regs.CTRIPHFILCLKCTL.bit.CLKPRESCALE = 0x3FF;

    //Specify that the inverting input is driven by internal DAC
    Cmpss2Regs.COMPCTL.bit.COMPHSOURCE = 0;
    Cmpss5Regs.COMPCTL.bit.COMPHSOURCE = 0;

    //Set VDDA as DAC reference
    Cmpss2Regs.COMPDACCTL.bit.SELREF = 0;
    Cmpss5Regs.COMPDACCTL.bit.SELREF = 0;

    //Set DAC value register to be updated from shadow on SYSCLK
    Cmpss2Regs.COMPDACCTL.bit.SWLOADSEL = 0;
    Cmpss5Regs.COMPDACCTL.bit.SWLOADSEL = 0;

    //Set DAC
    Cmpss2Regs.DACHVALS.bit.DACVAL = ((float) (1.5 / 3.0)) * 4096;
    Cmpss5Regs.DACHVALS.bit.DACVAL = ((float) (1.5 / 3.0)) * 4096;

    //Configure Comparator Outputs
    //Pin 25 is xbar output 2, we want to assign it to mux 2 (cmpss2H output)
    OutputXbarRegs.OUTPUT2MUX0TO15CFG.bit.MUX2 = 0;
    //Pin 15 is xbar output 4, we want to assign it to mux 8 (cmpss5H output)
    OutputXbarRegs.OUTPUT4MUX0TO15CFG.bit.MUX8 = 0;

    //Enable Mux for Output
    OutputXbarRegs.OUTPUT2MUXENABLE.bit.MUX2 = 1;
    OutputXbarRegs.OUTPUT4MUXENABLE.bit.MUX8 = 1;

    //Histeresis
    Cmpss2Regs.COMPHYSCTL.bit.COMPHYS = 0x2;
    Cmpss5Regs.COMPHYSCTL.bit.COMPHYS = 0x2;

    //Disable write to set of registers
    EDIS;
}
