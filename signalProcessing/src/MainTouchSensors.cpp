#include "Arduino.h"
#include "MainTouchSensors.h"

volatile unsigned int MainTouchSensors::touchCount_0;
volatile unsigned int MainTouchSensors::touchCount_1;
volatile unsigned int MainTouchSensors::touchCount_2;
unsigned int MainTouchSensors::lastTouchCount_0;
unsigned int MainTouchSensors::lastTouchCount_1;
unsigned int MainTouchSensors::lastTouchCount_2;

MainTouchSensors::MainTouchSensors(){
    pinMode(TOUCHPIN_0, INPUT_PULLUP);
    pinMode(TOUCHPIN_1, INPUT_PULLUP);
    pinMode(TOUCHPIN_2, INPUT_PULLUP);
    touchCount_0 = 0;
    touchCount_1 = 0;
    touchCount_2 = 0;
    lastTouchCount_0 = 0;
    lastTouchCount_1 = 0;
    lastTouchCount_2 = 0;
    attachInterrupt(digitalPinToInterrupt(TOUCHPIN_0), touchCount0_ISR, RISING);
    attachInterrupt(digitalPinToInterrupt(TOUCHPIN_1), touchCount1_ISR, RISING);
    attachInterrupt(digitalPinToInterrupt(TOUCHPIN_2), touchCount2_ISR, RISING);
    Serial.begin(9600);
}
void MainTouchSensors::touchCount0_ISR(){
    touchCount_0++;
}
void MainTouchSensors::touchCount1_ISR(){
    touchCount_1++;
}
void MainTouchSensors::touchCount2_ISR(){
    touchCount_2++;
}
unsigned int MainTouchSensors::getTouchCount(Touch_Id tid){
    switch(tid){
        case tid_0:
            return touchCount_0;
        case tid_1:
            return touchCount_1;
        case tid_2:
            return touchCount_2;
        default:
            return -1;
    }
}
bool MainTouchSensors::getTouched(Touch_Id tid){
    unsigned int tc = 0;
    unsigned int last_tc = 0;
    switch(tid){
        case tid_0:
            tc = touchCount_0;
            last_tc = lastTouchCount_0;
            lastTouchCount_0 = touchCount_0;
            break;
        case tid_1:
            tc = touchCount_1;
            last_tc = lastTouchCount_1;
            lastTouchCount_1 = touchCount_1;
            break;
        case tid_2:
            tc = touchCount_2;
            last_tc = lastTouchCount_2;
            lastTouchCount_2 = touchCount_2;
            break;
        default:
            return -1;
    } 
    if(tc > last_tc){
        return true;
    } else{
        return false;
    }
}    
