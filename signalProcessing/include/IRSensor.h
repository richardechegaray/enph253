#include <Arduino.h>
#define PI 3.14159265358979323846
#define NUMBER_OF_SAMPLES 200 //need to experiment with
#define ONE_KHZ 1000
#define TEN_KHZ 10000
class IRSensor{
    private:
        int pin;
        int mode;
        int sample[NUMBER_OF_SAMPLES];
        int onek[NUMBER_OF_SAMPLES*2];
        int tenk[NUMBER_OF_SAMPLES*2];
        int sample_ms;
    public:
        IRSensor(int pin_, int mode_);
        void ir_sample(int ir_pin);
        void reference_signal();
        int sample_intensity();
        void sample_frequency();
        void depositPlushies();
};