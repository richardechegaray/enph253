#include <Arduino.h>
#include <Servo.h>

#define RANGE 30 //cm

class ultrasonic{
    private:
        int trig;
        int echo;
        Servo myservo;
        int angle;
        int center_angle;
    public:
        ultrasonic(int, int);

        unsigned long duration;
        int distance;

        enum location {left, left_center, center, center_right, right, left_right, all, none} loc;
        enum points {c80, l60, r60, lr60, l40, r40, lr40, l20, r20, lr20, lr0} point; //assign based on "points" (bits), instead of returning the bits or the bit array

        int zero, one, two, three, four, five, six, distance_zero;

        // returns distance in cm to obstacle
        int get_distance();
        // returns bool if there is obstacle within provided range
        bool is_there_obj(int);
        // returns location of object within provided range
        // sweep -30 to 30 degrees
        enum location loc_of_obj(int);

        enum points checkLocation(int);

        //center the servo motor:
        int servo_center(int range);
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