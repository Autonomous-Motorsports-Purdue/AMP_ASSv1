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

//PREPROCESSOR DIRECTIVES
#include "F28x_Project.h"
#include "util.h"

#include "amp_gpio.h"
#include "amp_interrupts.h"
#include "amp_serial.h"
#include "amp_pwm.h"
#include "amp_dac.h"
//i2c
//timer
#include "amp_service.h"

#include "amp_err.h"
#include "amp_cart_state.h"

//GLOBAL DECLARATIONS
//State variables
amp_cart_state_t    cart    = AMP_CART_STATE_DEFAULT;       //state variable for the cart
amp_serial_state_t  serial  = AMP_SERIAL_STATE_START_SEEK;  //state variable for servicing serial packets

//Serial variables
Uint_16             c_byte  = 0;
Uint_16             c_crc   = 0;
Uint_16             i       = 0;    //iteration variable for reading packet data
amp_serial_pkt_t    c_pkt;

//Flags
Uint_16             new_pkt = 0;    //

//MAIN FUNCTION

void main(void)
{
// Initialize System Control:
// PLL, WatchDog, enable Peripheral Clocks
// This example function is found in the F2837xD_SysCtrl.c file.
   InitSysCtrl();

// Module Initializations

   amp_gpio_initialize();
   amp_interrupts_initialize();
   amp_serial_initialize();
   amp_pwm_initialize();
   amp_dac_initialize();
   //i2c initialize
   //timer initialize

// Local Variables
   amp_serial_pkt_t pkt;
   amp_err_code_t err_code;


// MAIN LOOP
   for(;;)
   {
       // Main Logic will be made like a state machine.
       // BEGIN -> Receive Packet -> Service Packet -> Handle Errors -> BEGIN
       // At ANY Point, however, if errors are received they should be handled
       // there and not later.

       switch(cart)
       {
           case AMP_CART_STATE_DEFAULT:
               //This is the DEFAULT state
               //Looking for enable packet
               //Add code to transmit "IN DEFAULT STATE" over UART

               break;
           case AMP_CART_STATE_ENABLED:
               //This is the ENABLED state
               //Contactor enabled, Power delivered to MC and servo, but not yet enabled
               break;
           case AMP_CART_STATE_DRIVE:
               //This is the DRIVE state
               if(service_pkt)
               {
                   new_pkt = 0;
                   //service the new packet
               }
               break;
       }

       // Receive the Packet over Serial
       err_code = amp_serial_rx_pkt(&pkt);
       // Service the Packet
       err_code = amp_service_pkt(&pkt);


   }
}


interrupt void sciaRxIsr(void)
{
    //Receive interrupt code
    c_byte = ScibRegs.SCIRXBUF.bit.SAR & 0x00FF;
    switch(serial)
    {
        case AMP_SERIAL_STATE_START_SEEK:
            //Looking for start byte
            if(c_byte == AMP_SERIAL_START_PKT)
            {
                serial = AMP_SERIAL_STATE_ID_SEEK;
            }
            else
            {
                //ERROR, current byte is not start sequence
            }
            break;
        case AMP_SERIAL_STATE_ID_SEEK:
            // Get the ID of the Packet
            c_pkt->id = c_byte;
            c_crc = (c_crc + pkt->id) & 0xFF;
            serial = 2;
            break;
        case AMP_SERIAL_STATE_SIZE_SEEK:
            // Get the Size of the Packet
            c_pkt->size = c_byte;
            c_crc = (c_crc + c_pkt->size) & 0xFF;
            serial = 3;
            break;
        case AMP_SERIAL_STATE_DATA_SEEK:
            // Get the data from the Packet
            c_pkt->msg[i] = c_byte;
            if(i < c_pkt->size)
            {
                serial = AMP_SERIAL_STATE_DATA_SEEK;
            }
            else
            {
                i = 0;
                serial = AMP_SERIAL_STATE_CRC_SEEK;
            }
        case AMP_SERIAL_STATE_CRC_SEEK:
            // Get the CRC from the Packet
            c_pkt->crc = c_byte;
            if(crc != c_pkt->crc)
            {
                //ERROR AMP_SERIAL_ERROR_CRC
            }
            else
            {
                serial = AMP_SERIAL_STATE_STOP_SEEK;
            }
        case AMP_SERIAL_STATE_STOP_SEEK:
            // Verify that the Stope Byte was Received
            if(c_byte != AMP_SERIAL_STOP_PKT)
            {
                //ERROR, stop byte was not found
            }
            new_pkt = 1;//SET FLAG TO SERVICE PACKET
            serial = AMP_SERIAL_STATE_START_SEEK; //return to default state;
    }
}
