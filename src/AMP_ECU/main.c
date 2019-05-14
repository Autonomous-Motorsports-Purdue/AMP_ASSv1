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
#include "amp_serial.h"
#include "amp_pwm.h"
#include "amp_dac.h"
#include "amp_interrupts.h"
//i2c
//timer
#include "amp_service.h"
#include "amp_control.h"
#include "amp_eQEP.h"

#include "amp_err.h"
#include "amp_cart_state.h"

//GLOBAL DECLARATIONS
//State Variables
amp_cart_state_t    cart    = AMP_CART_STATE_DEFAULT;       //state variable for the cart
amp_serial_state_t  serial  = AMP_SERIAL_STATE_START_SEEK;  //state variable for servicing serial packets

//Serial Variables
uint16_t            c_byte  = 0;   //current byte, read from ScibRegs.SCIRXBUF.bit.SAR
uint16_t            c_crc   = 0;   //current cyclic redundancy check
uint16_t            i       = 0;   //iteration variable for reading packet data
amp_serial_pkt_t    c_pkt;         //current packet being assembled

//Timer Variables
uint16_t            intr_count = 0;
                                   // time between updates in pi loop

//Flags
uint16_t            new_pkt = 0;    //flag to indicate a packet needs to be serviced
uint16_t            timeout = 0;    //flag to indicate a timeout condition
uint16_t            control_flag = 0;
                                    // flag to indicate control loop update
uint16_t            dir_change = 0; // flag to indicate change in direction

//Control Variables
float               spd_meas    = 0;    //measured speed from eQEP module
float               spd_str     = 0;    //commanded speed from JETSON
float               spd_err_sum = 0;
float               trq_dbl_str = 0;    //raw output of PI loop
float               trq_str     = 0;    //satured output of PI loop

//eQEP Variables
long                prd_count   = 0;    //number of periods of QCLK
float               prd_time    = 0;    //prd_count in time (secs)
float               motor_speed = 0;    //angular speed of motor shaft (revs / sec)
float               wheel_speed = 0;    //angular speed of rear axis (wheel) (revs / sec)
float               cart_speed  = 0;    //translatioinal speed of cart (meteres / sec)

//MAIN FUNCTION
void main(void) {
    // Initialize System Control:
    // PLL, WatchDog, enable Peripheral Clocks
    // This example function is found in the F2837xD_SysCtrl.c file.
    InitSysCtrl();

    // Module Initializations

    amp_gpio_initialize();
    amp_serial_initialize();
    amp_pwm_initialize();
    amp_dac_initialize();
    //amp_timer_initialize();
    //i2c initialize
    //timer initialize
    amp_eQEP_initialize();
    amp_interrupts_initialize();


    // MAIN LOOP
    for(;;) {
        // Main Logic will be made like a state machine.
        // BEGIN -> Receive Packet -> Service Packet -> Handle Errors -> BEGIN
        // At ANY Point, however, if errors are received they should be handled
        // there and not later.

        switch(cart) {
            case AMP_CART_STATE_DEFAULT:
                //This is the DEFAULT state
                //Looking for enable packet
                //Add code to transmit "IN DEFAULT STATE" over UART
                amp_gpio_service(cart);
                if(new_pkt) {
                    new_pkt = 0;
                    //service the new packet
                    amp_service_pkt(&c_pkt);
                }
                break;
            case AMP_CART_STATE_ENABLED:
                //This is the ENABLED state
                //Contactor enabled, Power delivered to MC and servo, but not yet enabled
                amp_gpio_service(cart);
                if(new_pkt) {
                    new_pkt = 0;
                    //service the new packet
                    amp_service_pkt(&c_pkt);
                }
                break;
            case AMP_CART_STATE_DRIVE:
                //This is the DRIVE state
                amp_gpio_service(cart);
                amp_eQEP_serviceSpeed();
                if(control_flag) {
                    amp_control_loop(spd_str);
                    control_flag = 0;
                }
                if(new_pkt) {
                    new_pkt = 0;
                    //service the new packet
                    amp_service_pkt(&c_pkt);
                }
                break;
        }
    }
}
