#include "serial.h"
#include "kart.h"

#include "DueTimer.h"

int pl = 0;
bool inter = false;

void check_Serial()
{
/*
    if(pl < 10)
      digitalWrite(13, HIGH);
    else
      digitalWrite(13, LOW);//after 5 seconds write the LED low
    pl++;

*/
    if(inter == true)
    {
        set_error();
        while(true)
            digitalWrite(13, LOW);
        //return -1;
    }
    inter = true;
    //return 1;
}

void (*check_ptr)() = &check_Serial;

void start_Interrupt()
{
    Timer3.attachInterrupt(check_ptr).start(5000000);//5 second interrupt
}
