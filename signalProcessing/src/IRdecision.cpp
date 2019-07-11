#include <Arduino.h>
#include "IRdecision.h"

IRdecision::IRdecision(int pin_left, int pin_center, int pin_right, int set_mode){
    mode = set_mode;
    pinleft = pin_left;
    pincenter = pin_center;
    pinright = pin_right;
    left = IRsensor(pinleft, mode);
    center = IRsensor(pincenter, mode);
    right = IRsensor(pinright, mode);
}

int IRdecision::strongest_signal(){
    left.corr();
    center.corr();
    right.corr();

    corrleft = left.correlation;
    corrcenter = center.correlation;
    corrright = right.correlation;

    if ((corrleft > corrcenter) && (corrleft > corrright)){
        max_pin = pinleft;
    }
    else if ((corrcenter > corrleft) && (corrcenter > corrright)){
        max_pin = pincenter;
    }
    else if ((corrright > corrleft) && (corrright > corrcenter)){
        max_pin = pinright;
    }

    return max_pin;
};