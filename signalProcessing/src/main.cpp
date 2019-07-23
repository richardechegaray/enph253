#include <Arduino.h>
#include "ultrasonic.h"
#include "SideDoors.h"
//the following pin initializations could later on go to Robot.cpp, instead of main.cpp:

#define TRIG PB10
#define ECHO PB11
ultrasonic ultra = ultrasonic(TRIG, ECHO);
ultrasonic::location loc;
#define RANGE 30 //cm

#define LEFT_DOOR_SERVO PB12
#define RIGHT_DOOR_SERVO PB13
SideDoors side_doors = SideDoors(LEFT_DOOR_SERVO, RIGHT_DOOR_SERVO);

#define TX3 PB10
#define RX3 PB11
HardwareSerial Serial3 = HardwareSerial(RX3, TX3);

//based on the currentState we receive from the driving MCU, we will close/open the side doors, as well give instructions to the driving MCU when we need to change our path to avoid collision
//switch case for the state we receive from the driving MCU
//check the state sent from the driving MCU with a timer interrupt? or at the beginning of each loop?
//we need to also check what the ultrasonic sensor is "seeing"
//a bit like we will have "sub state machines" in plushie collection and deposit states, to both check the MCU state and check the ultrasonic sensor's output
int currentMajorState;
float initialTime;
float timeElapsed;

void setup() {
      Serial.begin(115200);
      //Serial3.begin(115200);
      delay(500);
      initialTime = millis();
}

void serialComm(){
      int check_available = Serial3.available();
      while (!check_available)
            check_available = Serial3.available(); 
      return;      
}

//turning strategy: turn right or left until main sensors read black again 
//need to turn left (METHANOS) or right (THANOS) until we read black tape again: in THANOS mode we are running into a pillar on our left, so we need to turn right until we find the tape, and vice versa in METHANOS mode
//void handleUltrasonicInfo(int turnInfo); : add this to tapeFollower.cpp
void ultrasonicStateMachine(){
      loc = ultra.loc_of_obj(RANGE);
      switch(loc){
       case ultrasonic::left:
            side_doors.leftDoorWrite(90); //left 90, right same as before
            break;
        case ultrasonic::left_center:
            //need to turn right until we read black tape again
            //Serial3.write(loc);
            side_doors.leftDoorWrite(30); //left 90, right same as before
            break;
        case ultrasonic::center:
            //need to turn left (METHANOS) or right (THANOS) until we read black tape again
            //Serial3.write(loc);
            //?
            break;
        case ultrasonic::center_right:
            //need to turn left until we read black tape again
            //Serial3.write(loc);
            side_doors.rightDoorWrite(30);
            break;
        case ultrasonic::right:
            side_doors.rightDoorWrite(90); //right 90, left same as before
            break;
        case ultrasonic::left_right:
            side_doors.doorsWrite(90);  //left 90, right 90
            break;
        case ultrasonic::all:
            //need to turn left (METHANOS) or right (THANOS) until we read black tape again
            //Serial3.write(loc);
            //?
            break;
        case ultrasonic::none:
            side_doors.doorsWrite(120); //left 120, right 60 (normal plushie collection position!)
            break;        
      }
}

void loop() {
      // serialComm();
      // currentMajorState = Serial3.read(); //get the state that the driver MCU is in
      // switch(currentMajorState){
      //       case 0: //up-ramp
      //             side_doors.doorsWrite(0); //left 0, right 180
      //             break;
      //       case 1: //collect plushie
      //             side_doors.doorsWrite(120); //left 120, right 60
      //             ultrasonicStateMachine();
      //             break;
      //       case 2: //deposit plushie
      //             ultrasonicStateMachine();
      //             break;      
      // }
      timeElapsed = (millis() - initialTime)/1000;
      if (timeElapsed < 13)
            side_doors.doorsWrite(0); //left 0, right 180
      else if (timeElapsed < 28)
            ultrasonicStateMachine();
      else{} 
}

