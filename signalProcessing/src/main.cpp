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
// #define ROLE METHANOS

#define COM_PLUSH_COLLECT_TIME 8
#define COM_PLUSH_DEPOSIT_TIME 26

int currentMajorState;
void calibrateDoors();
void ultrasonicStateMachine();
char role;

void setup() {
  Serial.begin(115200);
  Serial3.begin(9600);
  currentDoorState = doorsOpen;
}

//without communication:
/*void loop(){
  timeElapsed = (millis() - initialTime)/1000; // in seconds
  if (timeElapsed < COM_PLUSH_COLLECT_TIME)
    side_doors.doorsClose();
  else if (timeElapsed < COM_PLUSH_DEPOSIT_TIME){
    #if (ROLE == THANOS)
      side_doors.doorsOpenT();
    #elif (ROLE == METHANOS)
      side_doors.doorsOpenM();
    #endif
  }   
  // else if (timeElapsed < COM_STONES)
  //   side_doors.doorsTogether(); 
  else 
     side_doors.doorsTogether();
}*/

//with communication:
void loop() {  // MASTER
  char role = Serial3.read();
  if (role == 'm'){
    #define ROLE METHANOS
  } else if (role == 't'){
    #define ROLE THANOS  
  } else{

  }
  currentMajorState = Serial3.read();
    
   switch (currentMajorState) {
    case 0: // upRamp
      //Serial.println("upRamp");
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
     // Serial.println("collect");
      //ultrasonicStateMachine();
      break;
    
    case 2: // plushieDeposit
      //Serial.println("deposit");
      if(currentDoorState!=doorsTogether){
          side_doors.doorsTogether();
          currentDoorState = doorsTogether;
        }
      
      break;

    case 3: //stones
      //Serial.println("stones");
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
   }
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
      switch(loc){
        case ultrasonic::right:
          side_doors.rightDoorWrite(0); //close more than the Methanos angle, to see the change
          break;
        case ultrasonic::center_right:
          side_doors.rightDoorWrite(0); //close more than the Methanos angle, to see the change
          break;
      }
      /*switch(loc){
       case ultrasonic::left:
            side_doors.leftDoorWrite(60); //left 90, right same as before
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
            side_doors.doorsWrite(15); 
            break;
        case ultrasonic::center_right:
            //need to turn left until we read black tape again
            //Serial3.write(loc);
            side_doors.rightDoorWrite(30);
            break;
        case ultrasonic::right:
            side_doors.rightDoorWrite(60); //right 90, left same as before
            break;
        case ultrasonic::left_right:
            side_doors.doorsWrite(30);  //left 90, right 90
            break;
        case ultrasonic::all:
            //need to turn left (METHANOS) or right (THANOS) until we read black tape again
            //Serial3.write(loc);
            side_doors.doorsWrite(15);
            break;
        case ultrasonic::none:
            side_doors.doorsWrite(120); //left 120, right 60 (normal plushie collection position!)
            break;        
      }*/
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

void calibrateDoors() {
  side_doors.doorsClose();
  delay(5000);
  side_doors.doorsOpenM();
  delay(5000);
  side_doors.doorsTogether();
  delay(5000);
  side_doors.doorsOpenT();
  delay(5000);  
}