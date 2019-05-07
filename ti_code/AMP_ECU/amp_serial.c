/*
 * amp_serial.c
 *
 *  Created on: Mar 13, 2019
 *      Author: David Pimley and Tommy Krause
 */

#include "amp_serial.h"

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


/* FUNCTION ---------------------------------------------------------------
 * bool amp_serial_get_rx_buffer_status()
 *
 * Looks for character in the SCI registers. If a character is not found,
 * return false otherwise return true to indicate a character is present.
 */
bool amp_serial_get_rx_buffer_status() {
    if(ScibRegs.SCIFFRX.bit.RXFFST != 0)
    {
        return true;
    }
    else {
        return false;
    }
}

/* FUNCTION ---------------------------------------------------------------
 * uint8_t amp_serial_rx_raw()()
 *
 * Returns the next byte from the rx buffer when it is ready
 */
uint8_t amp_serial_rx_raw() {
    // Declare & Initialize Local Variables
    uint8_t         c_byte = 0x00;      // current byte received

    // Main Logic

    // Loop Until Character Found (Would be Better with Interrupts)
    while (!amp_serial_get_rx_buffer_status()) {
    }

    // Mask off the Necessary Byte (Others Reserved)
    c_byte = ScibRegs.SCIRXBUF.bit.SAR & 0x00FF;

    return c_byte;
}


/* FUNCTION ---------------------------------------------------------------
 * amp_err_code_t amp_serial_rx_pkt()()
 *
 * Builds a packet from the receive buffer. If the packet's crc is correct
 * it returns the packet, otherwise the packet is disregarded.
 */
amp_err_code_t amp_serial_rx_pkt(amp_serial_pkt_t * pkt) {
    // Declare & Initialize Local Variables
    uint8_t          c_byte = 0x00;     // current byte being received
    uint8_t          c_crc  = 0x00;     // current calculated crc
    amp_serial_pkt_t c_pkt;             // current packet being populated
    uint16_t         i      = 0;        // iteration variable

    // Main Logic

    // Check for Start Byte
        if ((c_byte = amp_serial_rx_raw()) == AMP_SERIAL_START_PKT) {
            // Until the End Byte has Been Seen, Build the Packet

            // Get the ID of the Packet
            pkt->id = amp_serial_rx_raw();
            c_crc = (c_crc + pkt->id) & 0xFF;

            // Get the Size of the Packet
            pkt->size = amp_serial_rx_raw();
            c_crc = (c_crc + pkt->size) & 0xFF;

            // Get the Data from the Packet
            for (i = 0; i < pkt->size; i++) {
                pkt->msg[i] = amp_serial_rx_raw();
                c_crc = (c_crc + pkt->msg[i]) & 0xFF;
            }

            // Get the CRC from the Packet
            pkt->crc = amp_serial_rx_raw();

            // Calculate Current CRC & Compare
            c_crc = (~c_crc + 1) & 0xFF;

            if (c_crc != pkt->crc) {
                return AMP_SERIAL_ERROR_CRC;
            }

            // Verify that the Stop Byte was Received
            if (amp_serial_rx_raw() != AMP_SERIAL_STOP_PKT) {
                return AMP_SERIAL_ERROR_RX_NO_STOP;
            }
        }
        return AMP_ERROR_NONE;
}


