/*
 * Packet Servicer from the NVIDIA Jetson TX2. The main functionality
 * of the software running on the TI is to read packets over UART and
 * set the correct modules given the packet received. The flow of logic
 * can be described in a state machine type way. A packet should be
 * received and then serviced. If at any point errors occur the errors
 * are handled and the software continues.
 *
 * Authors - Tommy Krause & David Pimley
 */

#include "F28x_Project.h"
#include "util.h"

#include "amp_dac.h"
#include "amp_pwm.h"
#include "amp_serial.h"
#include "amp_service.h"

#include "amp_err.h"

// Main
void main(void)
{
// Step 1. Initialize System Control:
// PLL, WatchDog, enable Peripheral Clocks
// This example function is found in the F2837xD_SysCtrl.c file.
   InitSysCtrl();

// Step 2. Initialize GPIO:
// This example function is found in the F2837xD_Gpio.c file and
// illustrates how to set the GPIO to it's default state.
   InitGpio();

// For this example, only init the pins for the SCI-B port.
//  GPIO_SetupPinMux() - Sets the GPxMUX1/2 and GPyMUX1/2 register bits
//  GPIO_SetupPinOptions() - Sets the direction and configuration of the GPIOS
// These functions are found in the F2837xD_Gpio.c file.
   GPIO_SetupPinMux(19, GPIO_MUX_CPU1, 2);
   GPIO_SetupPinOptions(19, GPIO_INPUT, GPIO_PUSHPULL);
   GPIO_SetupPinMux(18, GPIO_MUX_CPU1, 2);
   GPIO_SetupPinOptions(18, GPIO_OUTPUT, GPIO_ASYNC);

// Step 3. Clear all __interrupts and initialize PIE vector table:
// Disable CPU __interrupts
   DINT;

// Initialize PIE control registers to their default state.
// The default state is all PIE __interrupts disabled and flags
// are cleared.
// This function is found in the F2837xD_PieCtrl.c file.
   InitPieCtrl();

// Disable CPU __interrupts and clear all CPU __interrupt flags:
   IER = 0x0000;
   IFR = 0x0000;

// Enable global Interrupts and higher priority real-time debug events:
   EINT;  // Enable Global interrupt INTM
   ERTM;  // Enable Global realtime interrupt DBGM

// Module Initializations
   amp_serial_initialize();
   amp_pwm_initialize();
   amp_dac_initialize();

// Local Variables
   amp_serial_pkt_t pkt;
   amp_err_code_t err_code;

   for(;;) //main loop
   {
       // Main Logic will be made like a state machine.
       // BEGIN -> Receive Packet -> Service Packet -> Handle Errors -> BEGIN
       // At ANY Point, however, if errors are received they should be handled
       // there and not later.

       // Receive the Packet over Serial
       err_code = amp_serial_rx_pkt(&pkt);
       // Service the Packet
       err_code = amp_service_pkt(&pkt);


   }
}

