// Serial logger class
// Akshat Bisht <arb@purdue.edu>
// Fraser Dougall <fdougall@purdue.edu>

// Standard defines
#include <math.h>
#include <ctime>
#include <string.h>
#include <fstream>
#include <string>
#include <stdio.h>
#include <iostream>

// Ros defines
#include <ros/console.h>

/*
 * Temporarily commenting out so my compiler won't complain
 * #include "ros/ros.h"
*/

/*
 * As of Feb 13, 2021 (time here)
 * This code will not compile due to parameters not being passed in
 * Working with strings rather than C-strings since C-strings are a pain :(
 * - Fraser Dougall
 */
using namespace std;

class Logger {

    /*
     * Byte constants
     */
    #define AMP_SERIAL_START_PKT   0x02
    #define AMP_SERIAL_STOP_PKT    0x03
    #define AMP_SERIAL_ENABLE_PKT  0xF0
    #define AMP_SERIAL_KILL_PKT    0xF2
    #define AMP_SERIAL_CONTROL     0xF1

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

        return 0;
    }

    // returns formatted filename with prefix and current time.
    // e.g. g_start_time = 12 Feb 13:01, prefix = "buffer_output", extension = ".txt"
    // returns : "buffer_output_12_Feb_13:01.txt"
    // Code will figure out if extension has a "." in it
    string file_name_from_time(time_t current_time, string prefix, string extension){
        tm* curr = localtime(&current_time);

        int sec = curr->tm_sec;
        int min = curr->tm_min;
        int hour = curr->tm_hour;
        int day = curr->tm_mday;
        int month = curr->tm_mon + 1;
        int year = curr->tm_year + 1900;

        string filename = prefix;
        filename.append("_");
        filename.append(to_string(hour));
        filename.append(":");
        filename.append(to_string(min));
        filename.append(":");
        filename.append(to_string(sec));
        filename.append("_");
        filename.append(to_string(day));
        filename.append("/");
        filename.append(to_string(month));
        filename.append(extension);

        return filename;
    }

    void log(char *msg, int ros_code){

    }

    /*
     * todo: pass in a past packet to see if cart is speeding up / slowing down
     */
    string parse_packet(uint8_t *buf){

        string retval;

        /*
         * Get the value of the identity byte of the packet by getting the second byte stored in the buffer
         */
        uint8_t identity = *(&buf + 1);
        uint8_t crcValue;

        /*
         * Location of the CRC byte is dependent on the packet identity
         * See GitHub wiki for serial packet structure
         */
        if (identity == AMP_SERIAL_ENABLE_PKT || identity == AMP_SERIAL_KILL_PKT){
            crcValue = *(&buf + 2);
        } else{
            crcValue = *(&buf + 4);
        }

        /*
         * If enable or disable, just print out enabling / disabling and CRC
         * Otherwise loop through the buffer and parse values
         * Each parse_XXX function returns a string that will be concatenated onto retval
         */
        switch (identity){
            case (AMP_SERIAL_ENABLE_PKT) :
                retval = "Enabled Cart | CRC: ";
                retval << track_CRC(crcValue);
                break;
            case (AMP_SERIAL_KILL_PKT) :
                retval = "Disabled Cart | CRC: ";
                retval << track_CRC(crcValue);
            case (AMP_SERIAL_CONTROL) :
                /*
                 * todo figure out ID
                 */
                retval = parse_packet();
            default :
                retval = "Error in processing";
        }

        return retval;
    }

    /*
     * Pass a single byte here by value
     * Returns the integer value of the byte (0-255)
     *
     * Using short here since the value of the byte is small & no need to use an integer
     */
    short byte_to_dec(uint8_t inByte){
        return (unsigned short) inByte;
    }

    string parse_break(uint8_t breakBuf){
        string breakOutput;
        short breakValue = byte_to_dec(breakBuf);

        breakOutput << "Breaking with value: " << breakValue << endl;

        return breakOutput;
    }

    string parse_throttle(uint8_t throttleBuf){
        string throttleOutput;
        short throttleValue = byte_to_dec(throttleBuf);

        throttleOutput << "Breaking with value: " << breakValue << endl;

        return throttleOutput;
    }

    string parse_steering(uint8_t steeringBuf){
        string steeringOutput;
        short steering


        return steeringOutput;
    }
    /*
     * Uses overflow addition to track math of CRC
     *
     * TODO: Check with Isaac if this is the correct implementation of overflow addition
     */
    string track_CRC(uint8_t crc){
        string crcOutput;
        uint8_t postMath;

        postMath = crc + INT8_MAX; //overflows

        if (postMath == crc){
            crcOutput << "Overflow addition success" << endl;
        } else {
            crcOutput << "Overflow addition failed" << endl;
        }

        return crcOutput;
    }



    /*
     * TODO: Required Functions
     * byte_to_dec √
     * parse_break √
     * parse_throttle √
     * parse_steering √
     * track_CRC √
     *
     */
};

