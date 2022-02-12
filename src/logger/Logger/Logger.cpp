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

using namespace std;

class Logger {

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
    char *file_name_from_time(time_t *current_time, char *prefix, char *extension){

    }

    void log(char *msg, int ros_code){

    }


    string parse_packet(uint8_t *buf){

    }

    /*
     * TODO: Required Functions
     * hex_to_dec
     *
     */
};

