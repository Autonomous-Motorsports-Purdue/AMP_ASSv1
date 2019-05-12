/*
 * amp_interrupts.c
 *
 *  Created on: Apr 18, 2019
 *      Author: Tommy Krause
 */

#include "F28x_Project.h"
#include "amp_serial.h"

#include "amp_interrupts.h"
#include "amp_err.h"

//Serial variables
extern amp_serial_state_t  serial;  //state variable for servicing serial packets
extern uint16_t             c_byte; //current byte, read from ScibRegs.SCIRXBUF.bit.SAR
extern uint16_t             c_crc;  //current cyclic redundancy check
extern uint16_t             i;      //iteration variable for reading packet data
extern amp_serial_pkt_t     c_pkt;  //current packet being assembled

//Flags
extern uint16_t            new_pkt; //flag to indicate a packet needs to be serviced
extern uint16_t            count;

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
    // Interrupts that are used are re-mapped to
    // ISR functions found within this file.
    //
       EALLOW;  // This is needed to write to EALLOW protected registers
       PieVectTable.SCIB_RX_INT = &scibRxIsr;
       PieVectTable.SCIB_TX_INT = &scibTxIsr;
       PieVectTable.TIMER0_INT = &cpu_timer0_isr;
       EDIS;    // This is needed to disable write to EALLOW protected registers

    //
    // Enable required interrupts
    //
       PieCtrlRegs.PIECTRL.bit.ENPIE = 1;   // Enable the PIE block
       PieCtrlRegs.PIEIER9.bit.INTx3 = 1;   // PIE Group 9, INT3
       PieCtrlRegs.PIEIER9.bit.INTx4 = 1;   // PIE Group 9, INT4
       PieCtrlRegs.PIEIER1.bit.INTx7 = 1;   // PIE Group 7, INT7
       IER = 0x100;                         // Enable CPU INT

    // Enable global Interrupts and higher priority real-time debug events:
       EINT;  // Enable Global interrupt INTM
       ERTM;  // Enable Global realtime interrupt DBGM

       return AMP_ERROR_NONE;
}

/* FUNCTION (INTERRUPT SERVICE ROUTINE)
 * interrupt void sciaRxIsr(void)
 *
 * This ISR is triggered by a character being placed in the RXBUFFER
 *
 */
interrupt void scibRxIsr(void)
{

    // Acknowledge this interrupt to receive more interrupts from group 1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;

    c_byte = ScibRegs.SCIRXBUF.bit.SAR & 0x00FF;
    switch(serial)
    {
        case AMP_SERIAL_STATE_START_SEEK:
            //Looking for start byte
            if(c_byte == AMP_SERIAL_START_PKT)
            {
                //amp_timer_start();
                serial = AMP_SERIAL_STATE_ID_SEEK;
            }
            else
            {
                //ERROR, current byte is not start sequence
            }
            break;
        case AMP_SERIAL_STATE_ID_SEEK:
            // Get the ID of the Packet
            c_pkt.id = (amp_serial_pkt_id_t) c_byte;
            c_crc = (c_crc + c_pkt.id) & 0xFF;
            serial = AMP_SERIAL_STATE_SIZE_SEEK;
            break;
        case AMP_SERIAL_STATE_SIZE_SEEK:
            // Get the Size of the Packet
            c_pkt.size = c_byte;
            c_crc = (c_crc + c_pkt.size) & 0xFF;
            serial = AMP_SERIAL_STATE_DATA_SEEK;
            break;
        case AMP_SERIAL_STATE_DATA_SEEK:
            // Get the data from the Packet
            serial = AMP_SERIAL_STATE_DATA_SEEK;
            c_pkt.msg[i] = c_byte;
            c_crc = (c_crc + c_pkt.msg[i++]) & 0xFF;

            if (i >= c_pkt.size) {
                serial = AMP_SERIAL_STATE_CRC_SEEK;
            }
            break;
        case AMP_SERIAL_STATE_CRC_SEEK:
            // Get the CRC from the Packet
            c_pkt.crc = c_byte;
            if(c_crc != c_pkt.crc)
            {
                //ERROR AMP_SERIAL_ERROR_CRC
            }
            else
            {
                serial = AMP_SERIAL_STATE_STOP_SEEK;
            }
            break;
        case AMP_SERIAL_STATE_STOP_SEEK:
            // Verify that the Stop Byte was Received
            if(c_byte != AMP_SERIAL_STOP_PKT)
            {
                //ERROR, stop byte was not found
            }
            //amp_timer_stop();
            new_pkt = 1; //SET FLAG TO SERVICE PACKET
            i = 0; //
            c_crc = 0; //
            serial = AMP_SERIAL_STATE_START_SEEK; //return to default state;
            break;
    }
}

/* FUNCTION (INTERRUPT SERVICE ROUTINE)
 * interrupt void sciaRxIsr(void)
 *
 * This ISR is triggered by a character being placed in the TXBUFFER
 *
 */
interrupt void scibTxIsr(void)
{
    //Transmit interrupt code

}

/* FUNCTION (INTERRUPT SERVICE ROUTINE)
 * __interrupt void cpu_timer0_isr(void)
 *
 * This ISR is trigger a cpu_timer
 */
__interrupt void cpu_timer0_isr(void) {
    if(count > 500) {
        //throw timeout error, over 500 milliseconds
    }

    count++;

    // Acknowledge this __interrupt to receive more __interrupts from group 1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}
