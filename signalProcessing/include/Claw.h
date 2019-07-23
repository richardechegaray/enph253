//#include "Arduino.h"
#include "Servo.h"
#include "Touch.h"

#define CLAW_QRD PA0
#define CLAW_SERVO PB12
#define OPEN_ANGLE 90 //determine
#define CLOSE_ANGLE 0 //determine
#define DROP_REFLECTANCE_BEGIN 500
#define DROP_REFLECTANCE_END 700
#define LEAD_SCREW_UP PB_11
#define LEAD_SCREW_DOWN PB_10

class Claw{
    private:

    public:
        static Touch touch_sensors;
        static Servo claw_servo;
        static float clockFreq;
        static float period;
        static float speed;
        Claw();
        void getStone();
        void depositStone(); 
};