/*
 * amp_interrupts.c
 *
 *  Created on: Apr 18, 2019
 *      Author: Tommy Krause
 */

#include "amp_interrupts.h"

/* FUNCTION ---------------------------------------------------------------
 * amp_err_code_t amp_interrupts_initialize()
 *
 * Initializes interrupts
 */
amp_err_code_t amp_interrupts_initialize()
{
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
       EALLOW;  // This is needed to write to EALLOW protected registers
       PieVectTable.SCIA_RX_INT = &sciaRxIsr;
       PieVectTable.SCIA_TX_INT = &sciaTxIsr;
       EDIS;    // This is needed to disable write to EALLOW protected registers

    //
    // Enable interrupts required for this example
    //
       PieCtrlRegs.PIECTRL.bit.ENPIE = 1;   // Enable the PIE block
       PieCtrlRegs.PIEIER9.bit.INTx1 = 1;   // PIE Group 9, INT1
       PieCtrlRegs.PIEIER9.bit.INTx2 = 1;   // PIE Group 9, INT2
       IER = 0x100;                         // Enable CPU INT

    // Enable global Interrupts and higher priority real-time debug events:
       EINT;  // Enable Global interrupt INTM
       ERTM;  // Enable Global realtime interrupt DBGM
}

/* FUNCTION (INTERRUPT SERVICE ROUTINE)
 * interrupt void sciaRxIsr(void)
 *
 * This ISR is triggered by a character being placed in the RXBUFFER
 *
 */
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

/* FUNCTION (INTERRUPT SERVICE ROUTINE)
 * interrupt void sciaRxIsr(void)
 *
 * This ISR is triggered by a character being placed in the TXBUFFER
 *
 */

interrupt void sciaTxIsr(void)
{
    //Transmit interrupt code
}
