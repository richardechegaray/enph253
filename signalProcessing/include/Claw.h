#include "Arduino.h"
#include "Servo.h"

#define CLAW_TOUCH_PIN PB11
#define CLAW_SERVO_PIN_0 PB10 //move up & down
#define CLAW_SERVO_PIN_1 PB12 //move right & left

class Claw{
    private:

    public:
        static Servo servo_y;
        static Servo servo_x;
        Claw();
        void moveUp(int degrees);
        void moveDown(int degrees);
        void moveLeft(int degrees);
        void moveRight(int degrees);
        void grabStone(); //the rest is driving to the stone deposit area        
};