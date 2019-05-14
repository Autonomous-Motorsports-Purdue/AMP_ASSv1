/*
 * amp_gpio.c
 *
 *  Created on: Apr 14, 2019
 *      Author: Tommy Krause
 */


#include "amp_gpio.h"
#include "amp_err.h"

extern uint16_t dir_change;
extern float spd_str;

/* FUNCTION ---------------------------------------------------------------
 * amp_err_code_t amp_gpio_initialize()
 *
 * Initializes GPIO pins using the GPIO module support functions found in F2837xD_GPIO.c
 *
 * Could alternatively set register bits
 */
amp_err_code_t amp_gpio_initialize()
{
    // Following Descriptions are from F2837xD_Gpio.c

    // InitGpio - Sets all pins to be muxed to GPIO in input mode with pull-ups
    //            enabled. Also resets CPU control to CPU1 and disables open
    //            drain and polarity inversion and sets the qualification to
    //            synchronous. Also unlocks all GPIOs. Only one CPU should call
    //            this function.

    // GPIO_SetupPinMux - Set the peripheral muxing for the specified pin. The
    //                    appropriate parameters can be found in the GPIO Muxed
    //                    Pins table(4.4) in the  datasheet. Use the GPIO index
    //                    row (0 to 15) to select a muxing option for the GPIO.
    //
    // GPIO_SetupPinOptions - Setup up the GPIO input/output options for the
    //                        specified pin.
    //
       InitGpio();

    // Initialize Pins for SCI-B
    // GPIO19   ->  SCIB_RX         (INPUT, PUSHPULL)
    // GPIO18   ->  SCIB_TX         (OUPUT, No Synchronization ASYNC)
       GPIO_SetupPinMux(19, GPIO_MUX_CPU1, 2);
       GPIO_SetupPinOptions(19, GPIO_INPUT, GPIO_PUSHPULL);
       GPIO_SetupPinMux(18, GPIO_MUX_CPU1, 2);
       GPIO_SetupPinOptions(18, GPIO_OUTPUT, GPIO_ASYNC);

     // Initialize Pins for the adafruit Feather
     // GPIO95  ->  FEATHER_RESET   (OUTPUT, Synchronization to SYSCLKOUT)
     // GPIO97  ->  KILL_SIGNAL     (INPUT, PULLUP)
       GPIO_SetupPinOptions(95, GPIO_OUTPUT, GPIO_SYNC);
       //GPIO_SetupPinOptions(97, GPIO_INPUT, GPIO_PULLUP);

     // Initialize Pins for Servo Motor Control/Monitoring
     // GPIO24  ->  EN+_CTL         (OUTPUT, Synchronization to SYSCLKOUT)
     // GPIO16  ->  INPUT_A+_CTL    (OUTPUT, Synchronization to SYSCLKOUT)
     // GPIO14  ->  MC_HLFB+        (INPUT, PULLUP)
       GPIO_SetupPinOptions(24, GPIO_OUTPUT, GPIO_SYNC);
       GPIO_SetupPinOptions(16, GPIO_OUTPUT, GPIO_SYNC);
       GPIO_SetupPinOptions(14, GPIO_INPUT, GPIO_PULLUP);

     // Initialize Pins for Sevcon Motor Controller Control/ME1117 Speed Monitoring
     // GPIO6   ->  FS1_CTL         (OUTPUT)
     // GPIO7   ->  FWD_CTL         (OUTPUT)
     // GPIO8   ->  REV_CTL         (OUTPUT)
     // GPIO9   ->  MC_U            (INPUT)
     // GPIO10  ->  MC_V            (INPUT)
     // GPIO11  ->  MC_W            (INPUT)
       GPIO_SetupPinOptions(6, GPIO_OUTPUT, GPIO_SYNC);
       GPIO_SetupPinOptions(7, GPIO_OUTPUT, GPIO_SYNC);
       GPIO_SetupPinOptions(8, GPIO_OUTPUT, GPIO_SYNC);
       GPIO_SetupPinOptions(9, GPIO_INPUT, GPIO_PULLUP);
       GPIO_SetupPinOptions(10, GPIO_INPUT, GPIO_PULLUP);
       GPIO_SetupPinOptions(11, GPIO_INPUT, GPIO_PULLUP);

       return AMP_ERROR_NONE;
}

/* FUNCTION ---------------------------------------------------------------
 * amp_err_code_t amp_gpio_service()
 *
 * This function checks the GPIO pin settings and outputs the correct
 * settings for a given cart state
 */
amp_err_code_t amp_gpio_service(amp_cart_state_t cart)
{
    //NOTE: Should implement KEYSWITCH for version 2
    GpioDataRegs.GPASET.bit.GPIO16 = 1;
    //SERVO ENABLE LOGIC
    /* EASIER TO UNDERSTAND BUT NOT ELEGANT
    if(cart == AMP_CART_STATE_DRIVE) {
        //clear pin (set low) to send high signal to servo
        GpioDataRegs.GPACLEAR.bit.GPIO24 = 1;
    }
    else {
        //set pin high to send low signal to servo
        GpioDataRegs.GPASET.bit.GPIO24 = 1;
    }*/
    //MORE ELEGANT
    if(GpioDataRegs.GPADAT.bit.GPIO24 == (cart == AMP_CART_STATE_DRIVE)) {
        GpioDataRegs.GPATOGGLE.bit.GPIO24 = 1;
    }

    //PMAC ENABLE (FS1) LOGIC
    if(GpioDataRegs.GPADAT.bit.GPIO6 != (cart == AMP_CART_STATE_DRIVE)) {
        GpioDataRegs.GPATOGGLE.bit.GPIO6 = 1;
    }

    // If the v_speed is greater than zero, set the forward config
    if(spd_str > 0) {
        //forward
        GpioDataRegs.GPACLEAR.bit.GPIO8 = 1;
        GpioDataRegs.GPASET.bit.GPIO7 = 1;
    }
    // If the v_speed is less than zero, set the reverse config
    else if(spd_str < 0) {
        //reverse
        GpioDataRegs.GPACLEAR.bit.GPIO7 = 1;
        GpioDataRegs.GPASET.bit.GPIO8 = 1;
    }
    else {
        GpioDataRegs.GPACLEAR.bit.GPIO7 = 1;
        GpioDataRegs.GPACLEAR.bit.GPIO8 = 1;
    }

    return AMP_ERROR_NONE;
}
