#include <Arduino.h>
#include "WheelRotaries.h"

volatile int WheelRotaries::pos_1;
volatile int WheelRotaries::pos_2;
volatile int WheelRotaries::pos_3;
volatile int WheelRotaries::pos_4;

WheelRotaries::WheelRotaries() {
    pinMode(WHEEL1_A, INPUT_PULLUP);
    pinMode(WHEEL1_B, INPUT_PULLUP);
    pinMode(WHEEL2_A, INPUT_PULLUP);
    pinMode(WHEEL2_B, INPUT_PULLUP);
    // pinMode(WHEEL3_A, INPUT_PULLUP);
    // pinMode(WHEEL3_B, INPUT_PULLUP);
    // pinMode(WHEEL4_A, INPUT_PULLUP);
    // pinMode(WHEEL4_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(WHEEL1_A), wheelRotation1_ISR, FALLING);
    attachInterrupt(digitalPinToInterrupt(WHEEL2_A), wheelRotation2_ISR, FALLING);
    // attachInterrupt(digitalPinToInterrupt(WHEEL3_A), wheelRotation3_ISR, FALLING);
    // attachInterrupt(digitalPinToInterrupt(WHEEL4_A), wheelRotation4_ISR, FALLING);
    pos_1 = 0;
    pos_2 = 0;
    pos_3 = 0;
    pos_4 = 0;
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

void WheelRotaries::wheelRotation2_ISR(){
//we know pulse_A is on a falling edge, therefore we need to check if pulse_B is seeing high or low
    if(digitalRead(WHEEL2_B) == LOW){
        pos_2--;
    } else {
        pos_2++;
    }
}

// void WheelRotaries::wheelRotation3_ISR(){
// //we know pulse_A is on a falling edge, therefore we need to check if pulse_B is seeing high or low
//     if(digitalRead(WHEEL3_B) == LOW){
//         pos_3--;
//     } else {
//         pos_3++;
//     }
// }

// void WheelRotaries::wheelRotation4_ISR(){
// //we know pulse_A is on a falling edge, therefore we need to check if pulse_B is seeing high or low
//     if(digitalRead(WHEEL4_B) == LOW){
//         pos_4--;
//     } else {
//         pos_4++;
//     }
// }


int WheelRotaries::rotaryRotation(Wheel_Id id){
    int p = 0;
    switch(id){
        case wid_1:
            p = pos_1;
        case wid_2:
            p = pos_2;
        case wid_3:
            p = pos_3;
        case wid_4:
            p = pos_4;
        default:
            return -1;
    }
    return p;
}

float WheelRotaries::wheelDistance(Wheel_Id id){
    int p = 0;
    switch(id){
        case wid_1:
            p = pos_1;
        case wid_2:
            p = pos_2;
        case wid_3:
            p = pos_3;
        case wid_4:
            p = pos_4;
        default:
            return -1;
    }
    return ((float)p/CPR)*(2*PI*MOTOR_RADIUS_CM);
    //return pos_1/CPR; //increment once per full rotation
    //return pos_1; //increment for every rotation
}


