#ifndef STEERING_H
#define STEERING_H

#include "pwm_lib.h"
#include "tc_lib.h"

using namespace arduino_due::pwm_lib;

pwm<pwm_pin::PWMH5_PC19> pwm_pin44; //steering pin is 44

// 1kHz (left)
// 3kHz (center)
// 5kHz (right)

#define STEERING_PACKET_MAX 255
#define STEERING_PACKET_MIN 0

#define STEERING_CONTROL_MAX 5000
#define STEERING_CONTROL_MIN 1000

bool set_steering_raw(int steeringRequest)
{
  if(steeringRequest < STEERING_PACKET_MIN || steeringRequest > STEERING_PACKET_MAX)
  {
    return false;
  }
  //TODO: set the pwm frequency value of the steering pin
  //SerialUSB.print("inside set steering raw: steering = ");
  //SerialUSB.println(steeringRequest);

  //Scales floating point steering value to int value to request from PWM
  //int steeringCommand = STEERING_CONTROL_MIN + (steeringRequest - STEERING_PACKET_MIN) * (STEERING_CONTROL_MAX - STEERING_CONTROL_MIN) / (STEERING_PACKET_MAX - STEERING_PACKET_MIN);

    int steeringCommand = map(steeringRequest, 0, 255, 1000, 5000);

    uint32_t periodCommand = (100000000.0/steeringCommand);

  pwm_pin44.set_period_and_duty(periodCommand,periodCommand/2);

  return true; 
}

void steering_init() {
      pwm_pin44.start(33333,33333/2); //center the steering
}

#endif /* STEERING_H */
