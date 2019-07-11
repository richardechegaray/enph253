#include <Arduino.h>
#include "IRsensor.h"
#include "IRdecision.h"


IRdecision::IRdecision(int pin_left, int pin_center, int pin_right, int set_mode){
    mode = set_mode;
    left = IRsensor(pin_left, mode);
    center = IRsensor(pin_center, mode);
    right = IRsensor(pin_right, mode);
}

int IRdecision::strongest_signal(){
    left.corr();
    center.corr();
    right.corr();

    corrleft = left.correlation;
    corrcenter = center.correlation;
    corrright = right.correlation;

    



    return max_pin;
};