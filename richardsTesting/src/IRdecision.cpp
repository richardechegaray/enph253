#include <Arduino.h>
#include "IRdecision.h"

IRdecision::IRdecision(int pin_left, int pin_center, int pin_right, int set_mode) : 
    left(IRsensor(pin_left, set_mode)),
    center(IRsensor(pin_center, set_mode)),
    right(IRsensor(pin_right, set_mode)),
    mode(set_mode),
    pinleft(pin_left),
    pincenter(pin_center),
    pinright(pin_right)
{
    
}


int IRdecision::strongest_signal(){
    left.corr();
    center.corr();
    right.corr();

    corrleft = left.correlation;
    corrcenter = center.correlation;
    corrright = right.correlation;

    if ((corrleft > corrcenter) && (corrleft > corrright)){
        return pinleft;
    }
    else if ((corrcenter > corrleft) && (corrcenter > corrright)){
        return pincenter;
    }
    else if ((corrright > corrleft) && (corrright > corrcenter)){
        return pinright;
    }
    else return -1;
};