#include "SideDoors.h"

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

void SideDoors::leftDoorWrite(int angle){
    left_door_servo.write(180-angle);
}

void SideDoors::rightDoorWrite(int angle){
    right_door_servo.write(angle);
}
  
void SideDoors::doorsWrite(int angle){
    right_door_servo.write(angle);
    delay(500);
    left_door_servo.write(180-angle);
}

void SideDoors::doorsClose(){ // close left first
  left_door_servo.write(180-0);
  delay(750);
  right_door_servo.write(0);
}

void SideDoors::doorsTogether(){ // close left first
  left_door_servo.write(180-50);
  delay(500);
  right_door_servo.write(50);
}

//for collecting plushies (experimental angle measurements): 
void SideDoors::doorsOpenT(){ // left door smaller
  right_door_servo.write(135);
  delay(200);
  left_door_servo.write(180-58);
}

void SideDoors::doorsOpenM(){ // right door smaller
  right_door_servo.write(85);
  delay(200);
  left_door_servo.write(180-140);
}