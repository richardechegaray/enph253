#include "Arduino.h"
//#include "interrupts.cpp"

class TouchSensor{
    private:
        int pin;
        volatile unsigned int touch;
    public:
        TouchSensor(int pin_); 
        unsigned int getTouch();
        static void touched(); 
};