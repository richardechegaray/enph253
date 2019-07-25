#include <Arduino.h>
#include "ultrasonic.h"
#include "SideDoors.h"
#include "Claw.h"

#define TRIG PB6
#define ECHO PB7
ultrasonic ultra = ultrasonic(TRIG, ECHO);
//ultrasonic::location loc;
ultrasonic::points point;

#define LEFT_DOOR_SERVO PB13
#define RIGHT_DOOR_SERVO PB12
SideDoors side_doors = SideDoors(LEFT_DOOR_SERVO, RIGHT_DOOR_SERVO);

#define TX3 PB10
#define RX3 PB11
HardwareSerial Serial3 = HardwareSerial(RX3, TX3);

Claw claw = Claw();

int currentMajorState;
float initialTime;
float timeElapsed;

void setup() {
  Serial.begin(115200);
  Serial3.begin(115200);
  delay(500);
  serialComm();
  initialTime = millis();
}

void serialComm(){
  int check_available = Serial3.available();
  while (!check_available)
        check_available = Serial3.available(); 
  return;      
}

void ultrasonicStateMachine_new(){
  point = ultra.checkLocation(RANGE);
  switch(point){
    case ultrasonic::c80:
      Serial3.write(1); //indicate that we need to avoid collision by turning
      break;
    case ultrasonic::l60:
      side_doors.leftDoorWrite(48); //left is 180-angle
      break;
    case ultrasonic::r60:
      side_doors.rightDoorWrite(48);
      break;
    case ultrasonic::lr60:
      side_doors.doorsWrite(48);
      break;
    case ultrasonic::l40:
      side_doors.leftDoorWrite(72);
      break;
    case ultrasonic::r40:
      side_doors.rightDoorWrite(72);
      break;
    case ultrasonic::lr40:
      side_doors.doorsWrite(72);
      break;
    case ultrasonic::l20: 
      side_doors.leftDoorWrite(24);
      break;
    case ultrasonic::r20:
      side_doors.rightDoorWrite(24);
      break;
    case ultrasonic::lr20:
      side_doors.doorsWrite(24);
      break;
    case ultrasonic::lr0:
      side_doors.doorsWrite(120); //normal plushie collection angle!
      break;
  }
}

void loop() {
  currentMajorState = Serial3.read(); //get the state that the driver MCU is in
  switch(currentMajorState){
    case 0: //up-ramp
      //Serial.println("up-ramp");
      break;
    case 1: //collect plushie
      ultrasonicStateMachine_new();
      //Serial.println("collect");
      break;
    case 2: //deposit plushie
      ultrasonicStateMachine_new();
      //Serial.println("deposit");
      break;  
    case 3: //stone collection
      claw.getStone();
    case 4: //stone deposit
      claw.depositStone();    
  }
}