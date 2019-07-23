#include <Arduino.h>
#include "ultrasonic.h"
#include "SideDoors.h"

#define TRIG PB6
#define ECHO PB7
ultrasonic ultra = ultrasonic(TRIG, ECHO);
//ultrasonic::location loc;
ultrasonic::points point;
#define RANGE 30 //cm

#define LEFT_DOOR_SERVO PB13
#define RIGHT_DOOR_SERVO PB12
SideDoors side_doors = SideDoors(LEFT_DOOR_SERVO, RIGHT_DOOR_SERVO);

// #define TX3 PB10
// #define RX3 PB11
// HardwareSerial Serial3 = HardwareSerial(RX3, TX3);

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

// void serialComm(){
//       int check_available = Serial3.available();
//       while (!check_available)
//             check_available = Serial3.available(); 
//       return;      
// }
/*
void ultrasonicStateMachine(){
      loc = ultra.loc_of_obj(RANGE);
      switch(loc){
       case ultrasonic::left:
            side_doors.leftDoorWrite(90); //left 90, right same as before
            break;
        case ultrasonic::left_center:
            //need to turn right until we read black tape again
            //Serial3.write(loc);
            side_doors.leftDoorWrite(45); //left 90, right same as before
            break;
        case ultrasonic::center:
            //need to turn left (METHANOS) or right (THANOS) until we read black tape again
            //Serial3.write(loc);
            //?
            break;
        case ultrasonic::center_right:
            //need to turn left until we read black tape again
            //Serial3.write(loc);
            side_doors.rightDoorWrite(45);
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
*/
void ultrasonicStateMachine_new(){
  point = ultra.checkLocation(RANGE);
  switch(point){
    case ultrasonic::c80:
      side_doors.doorsWrite(24);
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
      ultrasonicStateMachine_new();
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
      //ultrasonicStateMachine();
      // Serial.println(loc);
      // Serial.print(ultra.zero);
      // Serial.print(ultra.one);
      // Serial.print(ultra.two);
      // Serial.print(ultra.three);
      // Serial.print(ultra.four);
      // Serial.print(ultra.five);
      // Serial.println(ultra.six);
      //delay(200);
      // timeElapsed = (millis() - initialTime)/1000;
      // if (timeElapsed < 13)
      //       side_doors.doorsWrite(0); //left 0, right 180
      // else if (timeElapsed < 28)
      //       ultrasonicStateMachine();
      // else{}
}