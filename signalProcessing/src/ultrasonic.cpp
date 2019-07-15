#include <Arduino.h>
#include "ultrasonic.h"

#define CM 0.034 // this is 0.034 cm/microsecond speed of sound

ultrasonic::ultrasonic(int pin_trig, int pin_echo):
trig(pin_trig),
echo(pin_echo)
{
    pinMode(trig, OUTPUT); 
    pinMode(echo, INPUT);
}

int get_distance(){
    // set trig pin to low to clear it
    digitalWrite(trig, LOW);
    delayMicroseconds(2);

    // set trig pin to high to send out 10microsecond blast
    digitalWrite(trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig, LOW);

    // use pulseIn function described in header file to capture duration of the echo pin reading
    duration = pulseIn(echo, HIGH);

    // multiply by CM to convert to centimetres, and divide by two, because burst travels there and back
    distance = duration*CM/2;

    return distance;
}

bool is_there_obj(int range){
    int distance_to_obs = get_distance();
    if (distance_to_obs < range){
        return true;
    }
    else {
        return false;
    }
}

