#include "SideDoors.h"

/*
@params: left servo motor pin, right servo motor pin
Attaches the left servo motor and the right servo motor to the corresponding input pins, and writes to the motors to close the doors
*/
SideDoors::SideDoors(int leftDoorServo, int rightDoorServo):
    left_door(leftDoorServo), right_door(rightDoorServo)
{
    right_door_servo.attach(right_door);
    delay(100);
    left_door_servo.attach(left_door);
    delay(100);
    left_door_servo.write(180-0);
    delay(100);
    right_door_servo.write(0);
}

/*
@params: angle to write to the left servo motor
Writes the specified angle to the left servo motor
*/
void SideDoors::leftDoorWrite(int angle){
    left_door_servo.write(180-angle);
}

/*
@params: angle to write to the right servo motor
Writes the specified angle to the right servo motor
*/
void SideDoors::rightDoorWrite(int angle){
    right_door_servo.write(angle);
}

/*
@params: angle to write to the left and right servo motors
Writes the specified angle to the left and right servo motors
*/
void SideDoors::doorsWrite(int angle){
    right_door_servo.write(angle);
    delay(500);
    left_door_servo.write(180-angle);
}

/*
Writes to the motors to close the doors, closing the left door first
*/
void SideDoors::doorsClose(){
  left_door_servo.write(180-0);
  delay(750);
  right_door_servo.write(0);
}

/*
Writes to the motors such that the outer sides of the doors will touch each other, in the form of a triangular shape, closing the left door first
*/
void SideDoors::doorsTogether(){
  left_door_servo.write(180-50);
  delay(500);
  right_door_servo.write(50);
}

/*
Writes to the motors such that the doors will be open with the right door open more than the left door
*/
void SideDoors::doorsOpenT(){
  right_door_servo.write(135);
  delay(200);
  left_door_servo.write(180-58);
}

/*
Writes to the motors such that the doors will be open with the left door open more than the left door
*/
void SideDoors::doorsOpenM(){ // right door smaller
  right_door_servo.write(85);
  delay(200);
  left_door_servo.write(180-140);
}