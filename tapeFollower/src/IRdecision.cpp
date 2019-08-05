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

IRdecision::IRdecision()
{
    
}

int IRdecision::strongest_signal(){
    left.corr();
    center.corr();
    right.corr();

    corrleft = 4*left.correlation/3;  //EDITED TO COMPENSATE MID INTENSITY
    corrcenter = center.correlation/5;
    corrright = 4*right.correlation/3;

    if ((corrleft >= corrcenter) && (corrleft >= corrright)){
        return pinleft;
    }
    else if ((corrright >= corrleft) && (corrright >= corrcenter)){
        return pinright;
    }
    else if ((corrcenter >= corrleft) && (corrcenter >= corrright)){
        return pincenter;
    }
   
    else return -1;
};