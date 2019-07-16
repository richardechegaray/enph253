#include <Arduino.h>
//keep track of the distance we traveled
#define MOTOR_RADIUS_CM 2
#define CPR 24

#define WHEEL1_A PB12
#define WHEEL1_B PB13
#define WHEEL2_A PB3
#define WHEEL2_B PB4
// #define WHEEL3_A PB12
// #define WHEEL3_B PB13
// #define WHEEL4_A PB12
// #define WHEEL4_B PB13

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
        static void wheelRotation2_ISR();
        static void wheelRotation3_ISR();
        static void wheelRotation4_ISR();
        int rotaryRotation(Wheel_Id id);
        float wheelDistance(Wheel_Id id); 
};

        