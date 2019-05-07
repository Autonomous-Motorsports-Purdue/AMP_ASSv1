/*
 * amp_serial.c
 *
 *  Created on: Mar 13, 2019
 *      Author: David Pimley and Tommy Krause
 */

#include "amp_serial.h"
#include "util.h"

/* FUNCTION ---------------------------------------------------------------
 * amp_serial_err_code_t amp_serial_initialize()
 *
 * Initializes the SCIBREG
 */
amp_err_code_t amp_serial_initialize() {

    // Note: Clocks were turned on to the scib peripheral
    // in the InitSysCtrl() function

    // 1 stop bit,  No loopback
    // No parity, 8 data bits
    // async mode, idle-line protocol
    ScibRegs.SCICCR.all = 0x0007;

    //Enable RX, TX, RX ERR, internal SCICLK
    //Disable SLEEP, TXWAKE
    ScibRegs.SCICTL1.bit.RXENA = 1;
    ScibRegs.SCICTL1.bit.TXENA = 1;
    ScibRegs.SCICTL1.bit.RXERRINTENA = 1;

    //Enable TXINT, RXINT
    ScibRegs.SCICTL2.bit.TXINTENA = 1;
    ScibRegs.SCICTL2.bit.RXBKINTENA = 1;

    // scib at 9600 baud
    // @LSPCLK = 50 MHz (200 MHz SYSCLK) HBAUD = 0x02 and LBAUD = 0x8B.
    // @LSPCLK = 30 MHz (120 MHz SYSCLK) HBAUD = 0x01 and LBAUD = 0x86.
    ScibRegs.SCIHBAUD.all = 0x0002;
    ScibRegs.SCILBAUD.all = 0x008B;

    ScibRegs.SCICTL1.bit.SWRESET = 1;      // Relinquish SCI from Reset

    return AMP_ERROR_NONE;
}
