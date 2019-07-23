#include "SideDoors.h"

SideDoors::SideDoors(int leftDoorServo, int rightDoorServo):
    left_door(leftDoorServo), right_door(rightDoorServo)
{
    left_door_servo.attach(left_door);
    right_door_servo.attach(right_door);
    left_door_servo.write(160); //start with doors closed
    right_door_servo.write(20); //start with doors closed
    delay(200);
}
void SideDoors::leftDoorWrite(int angle){
    left_door_servo.write(180-angle);
}
void SideDoors::rightDoorWrite(int angle){
    right_door_servo.write(angle);
}
void SideDoors::doorsWrite(int angle){ //pass the angle based on left door position
    left_door_servo.write(180-angle);
    right_door_servo.write(angle);
}
int SideDoors::getLeftAngle(){
    return left_door_servo.read();
}
int SideDoors::getRightAngle(){
    return right_door_servo.read();
}