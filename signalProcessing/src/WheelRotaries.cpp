#include <Arduino.h>
#include "WheelRotaries.h"

volatile int WheelRotaries::pos_1;
volatile int WheelRotaries::pos_2;
volatile int WheelRotaries::pos_3;
volatile int WheelRotaries::pos_4;
// long WheelRotaries::last_pos_1;
// long WheelRotaries::last_pos_2;
// long WheelRotaries::last_pos_3;
// long WheelRotaries::last_pos_4;

WheelRotaries::WheelRotaries() {
    pinMode(WHEEL1_A, INPUT_PULLUP);
    pinMode(WHEEL1_B, INPUT_PULLUP);
    //list all
    attachInterrupt(digitalPinToInterrupt(WHEEL1_A), wheelRotation1_ISR, FALLING);
    //list all
    pos_1 = 0;
    //list all
    //Serial.begin(9600);
}

void WheelRotaries::wheelRotation1_ISR(){
//we know pulse_A is on a falling edge, therefore we need to check if pulse_B is seeing high or low
    if(digitalRead(WHEEL1_B) == LOW){
        pos_1--;
    } else {
        pos_1++;
    }
} 

int WheelRotaries::wheelDistance(Wheel_Id id){
    // int p = 0;
    // int last_p = 0;
    // switch(id){
    //     case wid_1:
    //         p = pos_1;
    //         last_p = last_pos_1;
    //     case wid_2:
    //         p = pos_2;
    //         last_p = last_pos_2;
    //     case wid_3:
    //         p = pos_3;
    //         last_p = last_pos_3;
    //     case wid_4:
    //         p = pos_4;
    //         last_p = last_pos_4;
    //     default:
    //         return -1;
    // }
    return pos_1/CPR; //increment once per full rotation
    //return pos_1; //increment for every rotation
}


