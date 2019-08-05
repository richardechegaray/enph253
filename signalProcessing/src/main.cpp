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
//enum commsMap {thanos, methanos, upRamp, collectPlushie, depositPlushie, stones, shutDown, firstTurnColl, driveStraightColl, lastTurnColl} message;

#define TX3 PB10
#define RX3 PB11
HardwareSerial Serial3 = HardwareSerial(RX3, TX3);

#define THANOS 0
#define METHANOS 1
#define ROLE THANOS

int currentMajorState;
void calibrateDoors();
void ultrasonicStateMachine();
int counter = 0;

void setup() {
  Serial.begin(115200);
  Serial3.begin(9600);
  currentDoorState = doorsOpen;
}

void loop() {  // MASTER
  /*currentMajorState = Serial3.read();
    
   switch (currentMajorState) {
    case 0: // upRamp
      if(currentDoorState!=doorsClosed){
        side_doors.doorsClose();
        currentDoorState = doorsClosed;
      }
      
      break;

    case 1: // plushieCollection
      #if (ROLE == THANOS)
        if(currentDoorState!=doorsOpen){
          side_doors.doorsOpenT();
          currentDoorState = doorsOpen;
        } 
      #elif (ROLE == METHANOS)
        if(currentDoorState!=doorsOpen){
          side_doors.doorsOpenM();
          currentDoorState = doorsOpen;
        }
      #endif
      //ultrasonicStateMachine();
      break;
    
    case 2: // plushieDeposit
      if(currentDoorState!=doorsTogether){
          side_doors.doorsTogether();
          currentDoorState = doorsTogether;
        }
      
      break;

    case 3: //stones
      if(currentDoorState!=doorsClosed){
        side_doors.doorsWrite(90);
        delay(500);
        side_doors.doorsClose();
        currentDoorState = doorsClosed;
      }
      break;

    case -1:
      break; 

    default:
      break;
   }*/
   ultrasonicStateMachine();
}

/*void loop(){
    distance = ultra.get_distance();
    Serial.println(distance);
    delay(200);
    //ultrasonicStateMachine();
    // loc = ultra.loc_of_obj(range);
    // Serial.print(ultra.zero);
    // Serial.print(ultra.one);
    // Serial.print(ultra.two);
    // Serial.print(ultra.three);
    // Serial.print(ultra.four);
    // Serial.print(ultra.five);
    // Serial.println(ultra.six);

    // Serial.print("distance of left most pin: ");
    // Serial.println(ultra.distance_zero);

    // Serial.print("Object at location: "); 
    // if (loc == 0)
    //     Serial.println("left"); 
    // else if (loc == 1) 
    //     Serial.println("left and center"); 
    // else if (loc == 2)
    //     Serial.println("center"); 
    // else if (loc == 3)
    //     Serial.println("center and right"); 
    // else if (loc == 4)
    //     Serial.println("right"); 
    // else if (loc == 5)
    //     Serial.println("left and right"); 
    // else if (loc == 6)
    //     Serial.println("all"); 
    // else if (loc == 7)
    //     Serial.println("no object detected");
}*/


void ultrasonicStateMachine(){
      loc = ultra.loc_of_obj(RANGE);
      /*switch(loc){
        case ultrasonic::left:
          side_doors.leftDoorWrite(55); //close less than the left_center case
          break;
        case ultrasonic::left_center:
          side_doors.leftDoorWrite(55); 
          break;
        default:
        if (ROLE == THANOS){
          // if(currentDoorState!=doorsOpen){
            //side_doors.doorsOpenT();
            side_doors.leftDoorWrite(90);
           // currentDoorState = doorsOpen;
          //}
        } else if(ROLE == METHANOS){
          // if(currentDoorState!=doorsOpen){
              side_doors.rightDoorWrite(90);
             // currentDoorState = doorsOpen;
            //}
        }
          break;
      }*/
      switch(loc){
       case ultrasonic::left:
          Serial.println("left");
          //side_doors.leftDoorWrite(60); //left 90, right same as before
          break;
        case ultrasonic::left_center:
          Serial.println("left-center");
          //side_doors.leftDoorWrite(30); //left 90, right same as before
          break;
        case ultrasonic::center:
          Serial.println("center");
          //side_doors.doorsWrite(15);
          counter++; 
          break;
        case ultrasonic::center_right:
          Serial.println("center-right");
          //side_doors.rightDoorWrite(30);
          break;
        case ultrasonic::right:
          Serial.println("right");
          //side_doors.rightDoorWrite(60); //right 90, left same as before
          break;
        case ultrasonic::left_right:
          Serial.println("left-right");
          //side_doors.doorsWrite(30);  //left 90, right 90
          break;
        case ultrasonic::none:
          Serial.println("none");
          //side_doors.doorsWrite(120); //left 120, right 60 (normal plushie collection position!)
          break; 
        default:
          Serial.println("default");   
          break;    
      }
      if(counter == 3){
        //collision avoidance
        Serial.println("counted 3");
        //send bit to driver
      }
}


/*void ultrasonicStateMachine_new(){
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
      #if (ROLE == THANOS)
        side_doors.doorsOpenT();
      #elif (ROLE == METHANOS)
        side_doors.doorsOpenM();
      #endif
      break;
  }
}*/

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