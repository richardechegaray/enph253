#include <Arduino.h>
#include "IRsensor.h"

class IRdecision{
    private:
        IRsensor left, center, right;
        int mode;
    public:
        IRdecision(int, int, int, int);

        float corrleft, corrcenter, corrright;
        int max_pin;
    
        // return pin that has strongest signal
        int strongest_signal();
};