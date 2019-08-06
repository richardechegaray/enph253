#include <Arduino.h>
#include "ultrasonic.h"
#include "SideDoors.h"

#define TRIG PB6
#define ECHO PB7
ultrasonic ultra = ultrasonic(TRIG, ECHO);
ultrasonic::location loc;

#define LEFT_DOOR_SERVO PB13
#define RIGHT_DOOR_SERVO PB12
SideDoors side_doors = SideDoors(LEFT_DOOR_SERVO, RIGHT_DOOR_SERVO);
enum doorStates{doorsClosed, doorsOpen, doorsTogether} currentDoorState;

#define TX3 PB10
#define RX3 PB11
HardwareSerial Serial3 = HardwareSerial(RX3, TX3);

#define THANOS 0
#define METHANOS 1
bool role;
uint8_t currentMajorState;
void calibrateDoors();
void ultrasonicStateMachine();
int counter;

void setup() {
  Serial.begin(115200);
  Serial3.begin(9600);
  currentDoorState = doorsOpen;
  counter = 0;
}

void loop() {  
  if(Serial3.available()){
    currentMajorState = Serial3.read();
    role = (currentMajorState >> 4) & 1U; //checking to see if that bit is high
    if(role){
      role = METHANOS;
      currentMajorState &= ~(1UL << 4); //clears 5th bit
      Serial.println("m");
    } else{
      role = THANOS;
      Serial.println("t");
    }
    switch (currentMajorState) {
      case 0: // upRamp
        //Serial.println("up-ramp");
        // if(currentDoorState!=doorsClosed){
        //   side_doors.doorsClose();
        //   currentDoorState = doorsClosed;
        // }
        break;

      case 1: // plushieCollection
        // #if (role == THANOS)
        //   if(currentDoorState!=doorsOpen){
        //     side_doors.doorsOpenT();
        //     currentDoorState = doorsOpen;
        //   } 
        // #elif (role == METHANOS)
        //   if(currentDoorState!=doorsOpen){
        //     side_doors.doorsOpenM();
        //     currentDoorState = doorsOpen;
        //   }
        // #endif
        //Serial.println("collection");
        ultrasonicStateMachine();
        break;
      
      case 2: // plushieDeposit
        // if(currentDoorState!=doorsTogether){
        //     side_doors.doorsTogether();
        //     currentDoorState = doorsTogether;
        //   }
        //Serial.println("deposit");
        ultrasonicStateMachine();
        break;

      case 3: //stones
        // if(currentDoorState!=doorsClosed){
        //   side_doors.doorsWrite(90);
        //   delay(500);
        //   side_doors.doorsClose();
        //   currentDoorState = doorsClosed;
        // }
        // break;
        //Serial.println("stones");
        break;

      default:
        break;
    } 
  }
}

void ultrasonicStateMachine(){
      if(currentMajorState == 1){
        loc = ultra.loc_of_obj_collection(RANGE, role);
      } else if(currentMajorState == 2){
        loc = ultra.loc_of_obj_deposit(RANGE);
      }
      switch(loc){
       case ultrasonic::left:
          //Serial.println("left");
          //side_doors.leftDoorWrite(60); //left 90, right same as before
          break;
        case ultrasonic::left_center:
          //Serial.println("left-center");
          //side_doors.leftDoorWrite(30); //left 90, right same as before
          break;
        case ultrasonic::center:
          //Serial.println("center");
          //side_doors.doorsWrite(15);
          counter++;
          break;
        case ultrasonic::center_right:
          //Serial.println("center-right");
          //side_doors.rightDoorWrite(30);
          break;
        case ultrasonic::right:
          //Serial.println("right");
          //side_doors.rightDoorWrite(60); //right 90, left same as before
          break;
        case ultrasonic::left_right:
          //Serial.println("left-right");
          //side_doors.doorsWrite(30);  //left 90, right 90
          break;
        case ultrasonic::none:
          //Serial.println("none");
          //side_doors.doorsWrite(120); //left 120, right 60 (normal plushie collection position!)
          break; 
        default:
          //Serial.println("default");   
          break;    
      }
      if(counter == 3){
        counter = 0; 
        Serial3.write(1);
      }
}

// void calibrateDoors() {
//   side_doors.doorsClose();
//   delay(5000);
//   side_doors.doorsOpenM();
//   delay(5000);
//   side_doors.doorsTogether();
//   delay(5000);
//   side_doors.doorsOpenT();
//   delay(5000);  
// }