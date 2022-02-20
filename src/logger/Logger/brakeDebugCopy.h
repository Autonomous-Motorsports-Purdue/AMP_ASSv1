#ifndef BRAKE_H
#define BRAKE_H

#define BRAKE_PIN1 17 //Brake pin 1
#define BRAKE_PIN2 16 //Brake pin 2


bool set_brake_raw(int brakeRequest)
{

    if(brakeRequest > 127)
    {
        digitalWrite(BRAKE_PIN1, HIGH);
        digitalWrite(BRAKE_PIN2, LOW);
    }
    else
    {
        digitalWrite(BRAKE_PIN1, LOW);
        digitalWrite(BRAKE_PIN2, HIGH);
    }
  
//  analogWrite(BRAKE_PIN1, brakeRequest);
    return true;

}

#endif /* BRAKE_H */
