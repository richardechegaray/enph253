#include <Arduino.h>
//#define PI 3.14159265358979323846
//#define MOTOR_RADIUS_CM 2
#define CPR 24

#define WHEEL1_A PB12
#define WHEEL1_B PB13
//define all motor pins

enum Wheel_Id{
    wid_1,
    wid_2,
    wid_3,
    wid_4
};

class WheelRotaries{
    private:
           
    public:
        WheelRotaries();
        volatile static int pos_1;
        volatile static int pos_2;
        volatile static int pos_3;
        volatile static int pos_4;
        // static long last_pos_1;
        // static long last_pos_2;
        // static long last_pos_3;
        // static long last_pos_4;
        static void wheelRotation1_ISR();
        // static void wheelRotation2_ISR();
        // static void wheelRotation3_ISR();
        // static void wheelRotation4_ISR();
        int wheelDistance(Wheel_Id id); 
};

        