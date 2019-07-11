#include <Arduino.h>
#include "WheelRotary.h"

WheelRotary::WheelRotary(int pina, int pinb){
    pin_a = pina;
    pin_b = pinb;
    pinMode(pin_a, INPUT_PULLUP);
    pinMode(pin_b, INPUT_PULLUP);
    counter = 0;
    rotation = 0;
    aLastState = digitalRead(pin_a);
    bLastState = digitalRead(pin_b);
}
unsigned int WheelRotary::countA(int pina, int pinb){
    aState = digitalRead(pina);
    if (aState != aLastState){     
        if (digitalRead(pinb) != aState) { 
            counter ++;
        } else {
            counter --;
        }
    }
    aLastState = aState;
    return counter;
}
unsigned int WheelRotary::countB(int pina, int pinb){
    bState = digitalRead(pinb);
    if (bState != bLastState){
        if (digitalRead(pina) != bState) { 
            counter --;
        } else {
            counter ++;
        }    
    }
    bLastState = bState;
    return counter;
}  
