//###########################################################################
//
// BASED OFF OF FILE:    Example_2837xDSci_Echoback.c
//
//!  \note If you are unable to open the .ht file, or you are using
//!  a different terminal, you can open a COM port with the following settings
//!  -  Find correct COM port
//!  -  Bits per second = 9600 THIS IS WRONG, HAD TO CONFIGURE TERMINAL TO COMMUNICATE AT 4800 BPS
//!  -  Date Bits = 8
//!  -  Parity = None
//!  -  Stop Bits = 1
//!  -  Hardware Control = None
//!
//!  \b Watch \b Variables \n
//!
//! \b External \b Connections \n
//!  Connect the SCI-B port to a PC via a transceiver and cable.
//!  - GPIO19 is SCI_B-RXD
//!  - GPIO18 is SCI_B-TXD
//!
//!  This File communicates through scia to a terminal
//
// Included Files
//
#include "F28x_Project.h"

//
// Globals
//


//
// Function Prototypes
//
void scia_echoback_init(void);
void scia_fifo_init(void);
void scia_xmit(char a);
void scia_msg(char *msg);

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
   GPIO_SetupPinMux(28, GPIO_MUX_CPU1, 2);
   GPIO_SetupPinOptions(28, GPIO_INPUT, GPIO_PUSHPULL);
   GPIO_SetupPinMux(29, GPIO_MUX_CPU1, 2);
   GPIO_SetupPinOptions(29, GPIO_OUTPUT, GPIO_ASYNC);

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
// Step 4. User specific code:
//
   Uint16 ReceivedChar[10];//Character array
   char *msg;   //Message to output
   int count = 0;   //Number of characters to read
   int read_data = 0;   //flag that signals when to read data, set once carriage return is seen
   int total_val = 0;   //total sum after character to int conversion
   int curr_val = 0;    //current value to add to total value
   int multiplier = 1;  //multiplier that ensures curr_val is contributing to the correct decimal place

   scia_fifo_init();       // Initialize the SCI FIFO
   scia_echoback_init();   // Initialize SCI for echoback, CHANGE THESE NAMES PERHAPS?

   msg = "\nHello World!\n\0";
   scia_msg(msg);

   for(;;) //main loop
   {
       if(SciaRegs.SCIFFRX.bit.RXFFST != 0) //this checks if there is a character to read
       {
           ReceivedChar[count] = SciaRegs.SCIRXBUF.all; //put character in array
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
               scia_msg(msg);
               read_data = 0;
           }
       }
       if(read_data)
       {
           read_data = 0;//take care of flag to eliminate infinite loop
           while(count > 1)
           {
               //This switch statement converts the character at [count-2] to an integer
               switch(ReceivedChar[count - 2])
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
           scia_msg((char *) ReceivedChar);
           msg = "\n\0";
           scia_msg(msg);
           count = 0;
           multiplier = 1;
           total_val = 0;
       }
   }
}

//
//  scia_echoback_init - Test 1,scia  DLB, 8-bit word, baud rate 0x000F,
//                       default, 1 STOP bit, no parity
//
void scia_echoback_init()
{
    //
    // Note: Clocks were turned on to the scia peripheral
    // in the InitSysCtrl() function
    //

    SciaRegs.SCICCR.all = 0x0007;   // 1 stop bit,  No loopback
                                    // No parity,8 char bits,
                                    // async mode, idle-line protocol
    SciaRegs.SCICTL1.all = 0x0003;  // enable TX, RX, internal SCICLK,
                                    // Disable RX ERR, SLEEP, TXWAKE
    SciaRegs.SCICTL2.all = 0x0003;
    SciaRegs.SCICTL2.bit.TXINTENA = 1;
    SciaRegs.SCICTL2.bit.RXBKINTENA = 1;

    //
    // scia at 9600 baud
    // @LSPCLK = 50 MHz (200 MHz SYSCLK) HBAUD = 0x02 and LBAUD = 0x8B.
    // @LSPCLK = 30 MHz (120 MHz SYSCLK) HBAUD = 0x01 and LBAUD = 0x86.
    //
    SciaRegs.SCIHBAUD.all = 0x0002;
    SciaRegs.SCILBAUD.all = 0x008B;

    SciaRegs.SCICTL1.all = 0x0023;  // Relinquish SCI from Reset
}

//
// scia_xmit - Transmit a character from the SCI
//
void scia_xmit(char b)
{
    while (SciaRegs.SCIFFTX.bit.TXFFST != 0) {}
    SciaRegs.SCITXBUF.all = b;
}

//
// scia_msg - Transmit message via scia
//
void scia_msg(char * msg)
{
    int i;
    i = 0;
    while(msg[i] != '\0')
    {
        scia_xmit(msg[i]);
        i++;
    }
}

//
// scia_fifo_init - Initialize the SCI FIFO
//
void scia_fifo_init()
{
    SciaRegs.SCIFFTX.all = 0xE040;
    SciaRegs.SCIFFRX.all = 0x2044;
    SciaRegs.SCIFFCT.all = 0x0;
}

//
// End of file
//
