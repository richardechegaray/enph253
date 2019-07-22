#include <Arduino.h>
#include <Servo.h>

class SideDoors{
    private:
        int left_door;
        int right_door;
        Servo left_door_servo;
        Servo right_door_servo;
    public:
        SideDoors(int, int);
        void leftDoorWrite(int angle);
        void rightDoorWrite(int angle);
        void doorsWrite(int angle);
        int getLeftAngle();
        int getRightAngle();
};