#include <Arduino.h>
#include "IRsensor.h"

class IRdecision{
    private:
        IRsensor left, center, right;
        int pinleft, pincenter, pinright;
        int mode;

    public:
        IRdecision(int, int, int, int);
        IRdecision();
        void setMode(int set_mode);

        float corrleft, corrcenter, corrright;
        int max_pin;
    
        // return pin that has strongest signal
        int strongest_signal();
};