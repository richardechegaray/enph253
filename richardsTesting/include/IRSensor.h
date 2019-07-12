#include <Arduino.h>

class IRsensor{
    private:
        int pin;
        int mode;
    public:
        IRsensor();
        IRsensor(int, int);
        int sample_size;
        int power;

        float average;
        float time_start;
        float time_end;
        float duration;
        float time_per_sample;

        float correlation;
    
        // collect the sample of size sample_size into array
        void sample(int);
        void corr();
};