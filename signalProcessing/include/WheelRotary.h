#include <Arduino.h>
#define PI 3.14159265358979323846
#define PPR 24
#define CPR PPR*4
//things that we won't define as parameters will go to define statements

class WheelRotary{
    private:
        int pin_a;
        int pin_b;    
    public:
        WheelRotary(int pina, int pinb);
        volatile int aState;
        volatile int aLastState;
        volatile int bState;
        volatile int bLastState;
        volatile int counter;
        int rotation;
        unsigned int countA(int pina, int pinb);
        unsigned int countB(int pina, int pinb);
        //unsigned int rotationCount(int pina, int pinb); 
};