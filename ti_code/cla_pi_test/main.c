/**
 * cla_pi_test:     This project tests two peripherals the eQEP peripheral and cla functionality
 *
 *                  Really I think this is a eQEP peripheral test
 *
 *                  eQEP1
 *                      -eQEP1 input A -> J8 Pin 77
 *                      -eQEP1 input B -> j8 Pin 76
 *
 *
 */

//Preprocessor Directives
//Includes
#include "F28x_Project.h"

//Defines
//ALL DIAMETERS are in INCHES
#define MOTOR_D         2
#define AXIS_D          8
#define WHEEL_D         10
#define PI              3.14159
#define INCH_TO_METER   0.0254
#define SYSCLK          200000000
#define PRESCALE        128
//Global Variables
float prd_time = 0;
long prd_count = 0;
float motor_speed = 0;
float wheel_speed = 0;
float cart_speed = 0;

//Function Declarations
void amp_eQEP_initialize(void);


int main(void) {
    //Initialize System Control
    InitSysCtrl();

    //Initialize GPIO
    //InitGpio();

    InitEQep1Gpio();

    //Clear all interrupts and initialize PIE vector table
    DINT;

    //Initialize PIE control registers to default state
    InitPieCtrl();

    //Disable CPU interrupts and clear CPU interrupt flags
    IER = 0x0000;
    IFR = 0x0000;

    //Initialize eQEP module
    amp_eQEP_initialize();

    while(1) {
        //Polling loop to see if upevent has occured
        if(EQep1Regs.QEPSTS.bit.UPEVNT == 1) {
            if(EQep1Regs.QEPSTS.bit.COEF == 0)
            {
                //No Capture overflow
                prd_count = (unsigned long) EQep1Regs.QCPRD;
            }
            else // Capture overflow has occurred, hardcode result
            {
                prd_count = 0xFFFF;
            }


            /* FROM Technical Reference Manual
             * The capture timer (QCTMR) runs from prescaled SYSCLKOUT and the prescaler is programmed
             * by the QCAPCTL[CCPS] bits. The capture timer (QCTMR) value is latched into the capture period register
             * (QCPRD) on every unit position event and then the capture timer is reset, a flag is set in
             * QEPSTS:UPEVNT to indicate that new value is latched into the QCPRD register. Software can check this
             * status flag before reading the period register for low speed measurement, and clear the flag by writing 1.
             */

            //Now we have the value of the QCPRD, calculate speed
            //Each upevent is 1/4 rotation of the motor
            prd_time = prd_count * (1.0 / (SYSCLK / PRESCALE));
            motor_speed = 1 / (4 * prd_time); //Revolutions per second (CHECK this with DAVID)
            wheel_speed = ((float) MOTOR_D / AXIS_D) * motor_speed; //Revolutions per second
            cart_speed = wheel_speed * PI * INCH_TO_METER * WHEEL_D;

            //Clear Flag
            EQep1Regs.QEPSTS.all = 0x88; // Clear Unit position event flag, Clear overflow error flag
        }
    }

    return 0;
}

/* FUNCTION ---------------------------------------------------------------
 * void amp_eQEP_initialize(void)
 *
 * Initializes the eQEP peripheral with the desired settings
 */
void amp_eQEP_initialize(void) {

    //EQep1Regs.QUPRD = 2000000;

    //EQep1Regs.QEPCTL.bit.PCRM = 00;       // PCRM=00 mode - QPOSCNT reset on
                                          // index event
    //EQep1Regs.QEPCTL.bit.UTE = 1;         // Unit Timeout Enable
    //Qep1Regs.QEPCTL.bit.QCLM = 1;        // Latch on unit time out
    //Qep1Regs.QPOSMAX = 0xffffffff;
    //EQep1Regs.QEPCTL.bit.QPEN = 1;        // QEP enable

    /*  Quadrature Decoder Control
     *      -FREE_SOFT = 2
     *      -Quadrature count mode
     *      -Count Rising and Falling edge
     *      -Not swapped
     *      -No polarity changes
     */
    EQep1Regs.QDECCTL.all = 0x0000;

    /*  Quadrature Capture Control
     *      -Enable eQEP capture unit
     *      -CAPCLK = SYSCLKOUT/128
     *      -UPEVNT = QCLK/1
     */
    EQep1Regs.QCAPCTL.all = 0x8070;

    /* QEP Control Register
     *      -FREE_SOFT = 2
     *      -Enable eQEP peripheral
     */
    EQep1Regs.QEPCTL.all = 0x8008;



}
