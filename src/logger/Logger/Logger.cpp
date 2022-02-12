// Serial logger class

// Standard defines
#include <stdio.h>
#include <iostream>
#include <math.h>
#include <string.h>
#include <fstream>
#include <iostream>
#include <string>

// Ros defines

/*
 * Temporarily commenting out
 * #include "ros/ros.h"
*/

/*
 * As of Feb 12, 2021 (time here)
 * This code will not compile due to parameters not being passed in
 *
 * - Fraser Dougall
 */
using namespace std;

class Logger {

    /*
     * Byte constants
     */
    #define AMP_SERIAL_START_PKT   0x02
    #define AMP_SERIAL_STOP_PKT    0x03
    #define AMP_SERIAL_ENABLE_PKT      0xF0
    #define AMP_SERIAL_KILL_PKT   0xF2


    ofstream outfile;

    // function that writes buf to out_file.
    // !0 == failure; 0 == success
    int append_to_file(uint8_t *buf, char *out_file){
        /*
        * General structure:
        * Pass buffer into parse_packet then append to out_file
        */

        try {
            outfile.open("%c", out_file);
            outfile << parse_packet(buf) << endl;
        } catch(const ostream::failure& e){
            return -1;
        }

        outfile.close();

    }

    // returns formatted filename with prefix and current time.
    // e.g. g_start_time = 12 Feb 13:01, prefix = "buffer_output", extension = ".txt"
    // returns : "buffer_output_12_Feb_13:01.txt"
    // Code will figure out if extension has a "." in it
    string *file_name_from_time(time_t *current_time, char *prefix, char *extension){
        string file_name;

        file_name = "blah";

        return file_name;
    }

    void log(char *msg, int ros_code){

    }


    /*
     * Pass by address
     */
    string parse_packet(uint8_t *buf){

        string retval;

        /*
         * Loop through buffer using a switch statement inside of a for loop
         * Concatenate each string returned into a parsed packet
         */

        /*
         * Get the ID of the packet by getting the second byte stored in the buffer
         */
        uint8_t identity = &(++buf);

        /*
         * If enable or disable, just print out enabling / disabling and CRC
         * Otherwise loop through the buffer and parse values
         */
        switch (identity){
            case (AMP_SERIAL_ENABLE_PKT):
                retval = "Enabled Cart | ";
                retval << track_CRC(/*value of crc here*/);
        }
    }

    /*
     * Pass a single byte here by value
     */
    int byte_to_dec(uint8_t inByte){
        return (unsigned int) inByte;
    }

    string parse_break(uint8_t value){

    }

    string parse_throttle(uint8_t value){

    }

    string parse_steering(uint8_t value){

    }

    string track_CRC(uint8_t value){

    }



    /*
     * TODO: Required Functions
     * byte_to_dec
     * parse_break
     * parse_throttle
     * parse_steering
     * track_CRC
     *
     */
};

