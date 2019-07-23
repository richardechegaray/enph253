#include "Arduino.h"

#define CLAW_BOTTOM PB11
#define CLAW_TOP PB10
#define CLAW PA0


class Touch{
    private:
        
    public:
        Touch();
        volatile static bool bottomTouch;
        volatile static bool topTouch;
        volatile static bool clawTouch;
        static void bottomTouch_ISR();
        static void topTouch_ISR();
        static void clawTouch_ISR();
        bool getBottomTouch();
        bool getTopTouch();
        bool getClawTouch();
};