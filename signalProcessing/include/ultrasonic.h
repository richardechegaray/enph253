#include <Arduino.h>

class ultrasonic{
    private:
        int trig;
        int echo;
    public:
        ultrasonic(int, int);

        unsigned long duration;
        int distance;

        int get_distance();

};

// vcc pin requires 5V
// trig and echo pins go into digital io
// trig pin needs to be set to high for 10microseconds, this emits an 8 cycle sonic blast that travels speed of sound 340m/s
// this is detected by echo pin that will output the time in microseconds that the sound wave travelled
// 

// info on pulseIn() function:
// Reads a pulse (either HIGH or LOW) on a pin. 
// For example, if value is HIGH, pulseIn() waits for the pin to go from LOW to HIGH, starts timing, 
// then waits for the pin to go LOW and stops timing. Returns the length of the pulse in microseconds 
// or gives up and returns 0 if no complete pulse was received within the timeout. 
// The timing of this function has been determined empirically and will probably show errors in longer pulses. 
// Works on pulses from 10 microseconds to 3 minutes in length.
// Returns: The length of the pulse (in microseconds) or 0 if no pulse started before the timeout. Data type: unsigned long.