#include "Arduino.h"

#define TOUCHPIN_0 PB11
#define TOUCHPIN_1 PB10
#define TOUCHPIN_2 PA2

enum Touch_Id{
    tid_0,
    tid_1,
    tid_2
};

class MainTouchSensors{
    private:
        
    public:
        MainTouchSensors();
        volatile static unsigned int touchCount_0;
        volatile static unsigned int touchCount_1;
        volatile static unsigned int touchCount_2;
        static unsigned int lastTouchCount_0;
        static unsigned int lastTouchCount_1;
        static unsigned int lastTouchCount_2;
        static void touchCount0_ISR();
        static void touchCount1_ISR();
        static void touchCount2_ISR();
        unsigned int getTouchCount(Touch_Id tid);
        bool getTouched(Touch_Id tid);
};