#ifndef KART_H
#define KART_H

#include "serial.h"
#include "brake.h"
#include "steering.h"
#include "throttle.h"

#define FS1 35 //Foot Switch
//#define KS1   //Key Switch
#define FWD 29     //Forward
#define REV 53     //Reverse
#define EN_P 47    //Enable DONT USE THIS
#define INPUT_A 41 //input a for steeering motor DONT USE THIS

#define MOTOR_CONTROL_DELAY delay(1000);

uint8_t kart_brake;    //holds kart braking control data
uint8_t kart_throttle; //holds kart throttle control data
uint8_t kart_steering; //holds kart steering control data

uint8_t control_flag; //shows if control has been updated

//serial state acknowledge bytes
const uint8_t KART_IDLE_ACK = 0xa0;
const uint8_t KART_ENABLE_ACK = 0xa1;
const uint8_t KART_ERROR_ACK = 0xa2;
const uint8_t KART_CTRL_ACK = 0xa3;

enum KART_STATE
{
    IDLE,
    ENABLED,
    ERROR
};

void set_idle()
{
    digitalWrite(FS1, LOW);
    //    digitalWrite(KS1, LOW);
    digitalWrite(FWD, LOW);
    digitalWrite(REV, LOW);
    digitalWrite(EN_P, HIGH);
    digitalWrite(INPUT_A, HIGH);
    set_brake_raw(255);
}

void set_enabled()
{
    digitalWrite(FS1, HIGH);
    digitalWrite(REV, LOW);
    digitalWrite(EN_P, LOW);
    digitalWrite(INPUT_A, HIGH);
    //    digitalWrite(KS1, HIGH);
    MOTOR_CONTROL_DELAY
    digitalWrite(FWD, HIGH);
}

void set_error()
{
    while(true)
    {
        digitalWrite(LED_BUILTIN, HIGH);
        digitalWrite(FS1, LOW);
        //    digitalWrite(KS1, HIGH);
        digitalWrite(FWD, LOW);
        digitalWrite(REV, LOW);
        digitalWrite(EN_P, LOW);
        digitalWrite(INPUT_A, LOW);
        set_brake_raw(255);
        set_throttle_raw(0);
        set_steering_raw(255 / 2);
    }
}

KART_STATE cur_kart_state;

bool kart_init()
{
    pinMode(FS1, OUTPUT);
    pinMode(FWD, OUTPUT);
    pinMode(REV, OUTPUT);
    pinMode(EN_P, OUTPUT);
    pinMode(INPUT_A, OUTPUT);

    set_idle();
  
    cur_kart_state = IDLE;
    control_flag = 0;
    return true;
}

bool kart_control()
{
    //SerialUSB.print(kart_brake, HEX);
    //SerialUSB.print(kart_throttle, HEX);
    //SerialUSB.print(kart_steering, HEX);
    set_brake_raw(kart_brake);
    set_throttle_raw(kart_throttle);
    set_steering_raw(kart_steering);
    control_flag = 0;
    return true;
}

bool req_kart_state_change(KART_STATE req)
{
    switch (cur_kart_state)
    {
    case IDLE:
        if (req == ENABLED)
        {
            cur_kart_state = ENABLED;
            //            SerialUSB.print(KART_ENABLE_ACK);
            set_enabled();
            return true;
        }
        else if (req == ERROR)
        {
            cur_kart_state = ERROR;
            //            SerialUSB.print(KART_ERROR_ACK);
            set_error();
        }
        break;
    case ENABLED:
        if (req == ERROR)
        {
            cur_kart_state = ERROR;
            //            SerialUSB.print(KART_ERROR_ACK);
            set_error();
            return true;
        }
        break;
    case ERROR:
        if (req == IDLE)
        {
            cur_kart_state = IDLE;
            //            SerialUSB.print(KART_ERROR_ACK);
            set_idle();
        }
        break;
    }

    //all valid cases handled above
    cur_kart_state = ERROR;
    return false;
}

#endif /* KART_H */
