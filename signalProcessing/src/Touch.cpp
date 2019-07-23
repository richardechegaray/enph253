#include "Touch.h"

volatile bool Touch::bottomTouch;
volatile bool Touch::topTouch;
volatile bool Touch::clawTouch;

Touch::Touch(){
    pinMode(CLAW_BOTTOM, INPUT);
    pinMode(CLAW_TOP, INPUT);
    pinMode(CLAW, INPUT);
    attachInterrupt(digitalPinToInterrupt(CLAW_BOTTOM), bottomTouch_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(CLAW_TOP), topTouch_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(CLAW), clawTouch_ISR, CHANGE);
    bottomTouch = false;
    topTouch = false;
    clawTouch = false;
    //Serial.begin(9600);
}

void Touch::bottomTouch_ISR(){
    if(digitalRead(CLAW_BOTTOM)){
        //Serial.println("b rise");
        bottomTouch = true;
    } else{
        //Serial.println("b fall");
        bottomTouch = false;
    }
}
void Touch::topTouch_ISR(){
    if(digitalRead(CLAW_TOP)){
        topTouch = true;
        //Serial.println("t rise");
    } else{
        topTouch = false;
        //Serial.println("t fall");
    }
}
void Touch::clawTouch_ISR(){
    if(digitalRead(CLAW)){
        topTouch = true;
        //Serial.println("c rise");
    } else{
        topTouch = false;
        //Serial.println("c fall");
    }
}
bool Touch::getBottomTouch(){
    return bottomTouch;
}
bool Touch::getTopTouch(){
    return topTouch;
}
bool Touch::getClawTouch(){
    return clawTouch;
}