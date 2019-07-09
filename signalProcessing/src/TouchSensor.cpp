#include "Arduino.h"
#include "TouchSensor.h"

TouchSensor::TouchSensor(int pin_){
    pin = pin_;
    pinMode(pin, INPUT_PULLUP);
    //attachInterrupt(digitalPinToInterrupt(pin), touched(this), RISING);
    touch = 0;
}; 
unsigned int TouchSensor::getTouch(){
    return touch;
} 
// static void TouchSensor::touched(){
//     touch++;
// } 
