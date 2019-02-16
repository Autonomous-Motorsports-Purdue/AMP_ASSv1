// This file combines sci_test and pwm_test to create a file that can control the servo motor (PWM) and throttle command (DAC)
//
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
void scib_echoback_init(void);
void scib_fifo_init(void);
void scib_xmit(char a);
void scib_msg(char *msg);

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
// Step 2. Initialize GPIO:
// This example function is found in the F2837xD_Gpio.c file and
// illustrates how to set the GPIO to it's default state.
//
   InitGpio();

//
// For this example, only init the pins for the SCI-B port.
//  GPIO_SetupPinMux() - Sets the GPxMUX1/2 and GPyMUX1/2 register bits
//  GPIO_SetupPinOptions() - Sets the direction and configuration of the GPIOS
// These functions are found in the F2837xD_Gpio.c file.
//
   GPIO_SetupPinMux(19, GPIO_MUX_CPU1, 2);
   GPIO_SetupPinOptions(19, GPIO_INPUT, GPIO_PUSHPULL);
   GPIO_SetupPinMux(18, GPIO_MUX_CPU1, 2);
   GPIO_SetupPinOptions(18, GPIO_OUTPUT, GPIO_ASYNC);

//
// For this case just init GPIO pins for ePWM1
// These functions are in the F2837xD_EPwm.c file
//
    InitEPwm1Gpio();

//
// Step 3. Clear all __interrupts and initialize PIE vector table:
// Disable CPU __interrupts
//
   DINT;

//
// Initialize PIE control registers to their default state.
// The default state is all PIE __interrupts disabled and flags
// are cleared.
// This function is found in the F2837xD_PieCtrl.c file.
//
   InitPieCtrl();

//
// Disable CPU __interrupts and clear all CPU __interrupt flags:
//
   IER = 0x0000;
   IFR = 0x0000;

//
// Initialize the PIE vector table with pointers to the shell Interrupt
// Service Routines (ISR).
// This will populate the entire table, even if the __interrupt
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

//
// Step 4. User specific code:
//
   Uint16 ReceivedChar[10];//Character array
   char *msg;   //Message to output
   int count = 0;   //Number of characters to read
   int read_data = 0;   //flag that signals when to read data, set once carriage return is seen
   int total_val = 0;   //total sum after character to int conversion
   int curr_val = 0;    //current value to add to total value
   int multiplier = 1;  //multiplier that ensures curr_val is contributing to the correct decimal place
   int dac_code = 0;
   int freq_code = 0;

   int temp_val = 0; //value to DAC is set to
   scib_fifo_init();       // Initialize the SCI FIFO
   scib_echoback_init();   // Initialize SCI for echoback, CHANGE THESE NAMES PERHAPS?

   msg = "\nHello World!\n\0";
   scib_msg(msg);

   for(;;) //main loop
   {
       if(dac_code) //if dac_val has changed, update dac output, may be unnecessary
       {
           dac_code = 0; //clear flag
           DAC_PTR[DACB]->DACVALS.all = temp_val; //set dac value
       }
       if(freq_code)
       {
           freq_code = 0;
           timer_period = temp_val;
           EPwm1Regs.TBPRD = timer_period;
       }

       if(ScibRegs.SCIFFRX.bit.RXFFST != 0) //this checks if there is a character to read
       {
           ReceivedChar[count] = ScibRegs.SCIRXBUF.all; //put character in array
           if(ReceivedChar[count] == 10) //check if character is "end of statement" i.e. carriage return
           {
               read_data = 1; //set flag
           }
           else
           {
               count++;
           }
           //count value handling so there is no buffer overflow
           if(count > 9)
           {
               count = 0;
               msg = "\nCan only accommodate 8 characters, re-enter command\n\0";
               scib_msg(msg);
               read_data = 0;
           }
       }
       if(read_data)
       {
           read_data = 0;//take care of flag to eliminate infinite loop
           while(count > 1)
           {
               //This switch statement converts the character at [count-2] to an integer
               if(ReceivedChar[count-2] == 'd')
               {
                   dac_code = 1;
               }
               else if(ReceivedChar[count - 2] == 'f')
               {
                   freq_code = 1;
               }
               switch(ReceivedChar[count - 3])
               {
               case '0' :
                   curr_val = 0;
                   break;
               case '1' :
                   curr_val = 1;
                   break;
               case '2' :
                   curr_val = 2;
                   break;
               case '3' :
                   curr_val = 3;
                   break;
               case '4' :
                   curr_val = 4;
                   break;
               case '5' :
                   curr_val = 5;
                   break;
               case '6' :
                   curr_val = 6;
                   break;
               case '7' :
                   curr_val = 7;
                   break;
               case '8' :
                   curr_val = 8;
                   break;
               case '9' :
                   curr_val = 9;
               default :
                   curr_val = 0; //contribute nothing to total value
               }
               total_val = total_val + curr_val * multiplier; //add integer times its power multiplier to the total value
               count--;
               multiplier = multiplier * 10;
           }
           scib_msg((char *) ReceivedChar);
           msg = "\n\0";
           scib_msg(msg);
           count = 0;
           multiplier = 1;
           temp_val = total_val;
           total_val = 0;
       }
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
//
//  scib_echoback_init - Test 1,scib  DLB, 8-bit word, baud rate 0x000F,
//                       default, 1 STOP bit, no parity
//
void scib_echoback_init()
{
    //
    // Note: Clocks were turned on to the scib peripheral
    // in the InitSysCtrl() function
    //

    ScibRegs.SCICCR.all = 0x0007;   // 1 stop bit,  No loopback
                                    // No parity,8 char bits,
                                    // async mode, idle-line protocol
    ScibRegs.SCICTL1.all = 0x0003;  // enable TX, RX, internal SCICLK,
                                    // Disable RX ERR, SLEEP, TXWAKE
    ScibRegs.SCICTL2.all = 0x0003;
    ScibRegs.SCICTL2.bit.TXINTENA = 1;
    ScibRegs.SCICTL2.bit.RXBKINTENA = 1;

    //
    // scib at 9600 baud
    // @LSPCLK = 50 MHz (200 MHz SYSCLK) HBAUD = 0x02 and LBAUD = 0x8B.
    // @LSPCLK = 30 MHz (120 MHz SYSCLK) HBAUD = 0x01 and LBAUD = 0x86.
    //
    ScibRegs.SCIHBAUD.all = 0x0002;
    ScibRegs.SCILBAUD.all = 0x008B;

    ScibRegs.SCICTL1.all = 0x0023;  // Relinquish SCI from Reset
}

//
// scib_xmit - Transmit a character from the SCI
//
void scib_xmit(char b)
{
    while (ScibRegs.SCIFFTX.bit.TXFFST != 0) {}
    ScibRegs.SCITXBUF.all = b;
}

//
// scib_msg - Transmit message via scib
//
void scib_msg(char * msg)
{
    int i;
    i = 0;
    while(msg[i] != '\0')
    {
        scib_xmit(msg[i]);
        i++;
    }
}

//
// scib_fifo_init - Initialize the SCI FIFO
//
void scib_fifo_init()
{
    ScibRegs.SCIFFTX.all = 0xE040;
    ScibRegs.SCIFFRX.all = 0x2044;
    ScibRegs.SCIFFCT.all = 0x0;
}

//
// End of file
//
