/*
 * File: pwm_test
 * This is code written for the TMS320F2837xD launchpad development kit. The goal was to be able to enable a PWM channel.
 * I started from the example file epwm_updown_aq_cpu01.c from the CONTROLSUITE package. I had to update the included files to be able to run on a macOS system.
 */
//
// Included Files
//
#include "F28x_Project.h"

//
// Defines
//
#define REFERENCE_VDAC  0
#define REFERENCE_VREF  1
#define DACB    2
//
// Globals
//
//The timer period value of 25000 corresponds to a frequency of 1kHz. We can work backwards from there
float duty = .5;
Uint16 timer_period = 25000;
volatile struct DAC_REGS* DAC_PTR[4] = {0x0, &DacaRegs, &DacbRegs, &DaccRegs};
//
// Function Prototypes
//
void InitEPwm1Example(void);
__interrupt void epwm1_isr(void);
void ConfigureDAC(Uint16 dac_num);

//
// Main
//
void main(void)
{
//
// Step 1. Initialize System Control:
// PLL, WatchDog, enable Peripheral Clocks
// This example function is found in the F2837xD_SysCtrl.c file.
//
    InitSysCtrl();

//
// enable PWM1
//
    CpuSysRegs.PCLKCR2.bit.EPWM1=1;

//
// For this case just init GPIO pins for ePWM1
// These functions are in the F2837xD_EPwm.c file
//
    InitEPwm1Gpio();

//
// Step 3. Clear all interrupts and initialize PIE vector table:
// Disable CPU interrupts
//
    DINT;

//
// Initialize the PIE control registers to their default state.
// The default state is all PIE interrupts disabled and flags
// are cleared.
// This function is found in the F2837xD_PieCtrl.c file.
//
    InitPieCtrl();

//
// Disable CPU interrupts and clear all CPU interrupt flags:
//
    IER = 0x0000;
    IFR = 0x0000;

//
// Initialize the PIE vector table with pointers to the shell Interrupt
// Service Routines (ISR).
// This will populate the entire table, even if the interrupt
// is not used in this example.  This is useful for debug purposes.
// The shell ISR routines are found in F2837xD_DefaultIsr.c.
// This function is found in F2837xD_PieVect.c.
//
    InitPieVectTable();

//
// Interrupts that are used in this example are re-mapped to
// ISR functions found within this file.
//
    EALLOW; // This is needed to write to EALLOW protected registers
    PieVectTable.EPWM1_INT = &epwm1_isr;
    EDIS;   // This is needed to disable write to EALLOW protected registers

//
// For this example, only initialize the ePWM
//
    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;
    EDIS;

    InitEPwm1Example();

    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;
    EDIS;

//
// Step 4. User specific code, enable interrupts:
//

//
// Enable CPU INT3 which is connected to EPWM1-3 INT:
//
    IER |= M_INT3;

//
// Enable EPWM INTn in the PIE: Group 3 interrupt 1-3
//
    PieCtrlRegs.PIEIER3.bit.INTx1 = 1;

//
// Enable global Interrupts and higher priority real-time debug events:
//
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM

    ConfigureDAC(DACB);

//THIS IS OUR MAIN LOOP
    while(1)
    {
        DAC_PTR[DACB]->DACVALS.all = 2048;
    }
}

//
// epwm1_isr - EPWM1 ISR
//
__interrupt void epwm1_isr(void)
{
    //
    // Clear INT flag for this timer
    //
    EPwm1Regs.ETCLR.bit.INT = 1;

    //
    // Acknowledge this interrupt to receive more interrupts from group 3
    //
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}

//
// Configure DAC - Configure specified DAC
void ConfigureDAC(Uint16 dac_num)
{
    EALLOW;
    DAC_PTR[dac_num]->DACCTL.bit.DACREFSEL = REFERENCE_VREF;
    DAC_PTR[dac_num]->DACOUTEN.bit.DACOUTEN = 1;
    DAC_PTR[dac_num]->DACVALS.all = 0;
    EDIS;
}

void InitEPwm1Example()
{
    //
    // Setup TBCLK
    //
    EPwm1Regs.TBPRD = timer_period;       // Set timer period 25000 TBCLKs
    EPwm1Regs.TBPHS.bit.TBPHS = 0x0000;        // Phase is 0
    EPwm1Regs.TBCTR = 0x0000;                  // Clear counter

    //
    // Set Compare values
    //
    EPwm1Regs.CMPA.bit.CMPA = duty * timer_period;    // Set compare A value

    //
    // Setup counter mode
    //
    EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up and down
    EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;        // Disable phase loading
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       // Clock ratio to SYSCLKOUT
    EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;


    //
    // Set actions
    //
    EPwm1Regs.AQCTLA.bit.CAU = AQ_SET;            // Set PWM1A on event A, up
                                                  // count
    EPwm1Regs.AQCTLA.bit.CAD = AQ_CLEAR;          // Clear PWM1A on event A,
                                                  // down count
}
//
// End of file
//
