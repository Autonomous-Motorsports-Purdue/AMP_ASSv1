// Serial logger class

// Standard defines
#include <stdio.h>
#include <iostream>
#include <math.h>
#include <string.h>

// Ros defines
#include "ros/ros.h"

using namespace std;

class Logger {
    // function that writes buf to out_file.
    // !0 == failure; 0 == success
    int append_to_file(uint8_t *buf, char *out_file);

    // returns formatted filename with prefix and current time.
    // e.g. g_start_time = 12 Feb 13:01, prefix = "buffer_output", extension = ".txt"
    // returns : "buffer_output_12_Feb_13:01.txt"
    // Code will figure out if extension has a "." in it
    char *file_name_from_time(time_t *current_time, char *prefix, char *extension);

    void log(char *msg, int ros_code);

    char *parse_packet(uint8_t *buf);
};

