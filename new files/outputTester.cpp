using namespace std;

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>


void getPackets();

const uint8_t START_BYTE = 0x02;
const uint8_t STOP_BYTE = 0x03;
const uint8_t ID_ENABLE = 0xf0;
const uint8_t ID_CONTROL = 0xf1;
const uint8_t ID_KILL = 0xf2;
const uint8_t RESET_SERIAL_STATE = 0xdd;
const uint8_t DATA_LEN_1 = 0xe1;
const uint8_t DATA_LEN_2 = 0xe2;
const uint8_t DATA_LEN_3 = 0xe3;

//serial data byte buffers
uint8_t serial_id_buf; //holds incoming packet id
uint8_t data_len_buf;  //holds incoming length data
uint8_t brake_buf;     //holds incoming breaking data
uint8_t throttle_buf;  //holds incoming throttle data
uint8_t steering_buf;  //holds incoming steering data
uint8_t serial_crc;

void getPacket(){
    
    cout << "hello";
    
}