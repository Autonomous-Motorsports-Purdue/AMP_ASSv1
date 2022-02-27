//
// Created by Fraser Dougall on 2/20/22.
//

#include <stdio.h>
#include <iostream>
#include <stdint.h>

using namespace std;
/*
 * crcValue & FF
 * Count how many times it rolls over
 * Return the final value of the crc
 */
int main() {
    //CRC is some arbitrary value for testing
    uint8_t crc = 3;
    uint8_t result;

    result = crc + 1;

    //result = crc & FF;
    //int a = sizeof(uint8_t);

    //cout << "Result: " << result << endl;
    cout << "Blah: " << result << endl;

    return 0;
}