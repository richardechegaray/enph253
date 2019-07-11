#include <Arduino.h>
//#include <robot.cpp>
#include "WheelRotary.h"
#define PIN0 PA0
#define PIN1 PA1
//note: every .h file in include/ can make use of https://github.com/danieleff/STM32GENERIC/tree/master/STM32/libraries
//Robot sample_robot = Robot();

// void setup() {
//   sample_robot.state = up_ramp; //should state be a global field of the Robot class instead???
// }
// //Note: for each state, think about possible interrupts!!! in the ISR, whichever state we are in WILL affect which state we transition to
// void loop() {
//   //don't know if you would agree or not, but i thought of sparing one (master) function per state (of course there will be lots of interrupts interrupting:) the function) and letting the master function call other helper functions rather than calling multiple less significant functions that need each other one after another
//   //this could help us keep the switch state architecture simple?
//   switch (sample_robot.state){
//     case 0:
//       sample_robot.speed_optimized_pid();
//       break;
//     case 1:
//       sample_robot.regular_pid();
//       break;
//     case 2:
//       sample_robot.follow_IR();
//       break;
//     case 3:
//       //do
//       break;
//     case 4:
//       //do
//       break;
//     case 5:
//       //do
//       break;  
//   }
// }
WheelRotary wheelRotary = WheelRotary(PIN0, PIN1);
unsigned rot = 0;
void setup(){
  Serial.begin(115200); 
}

void loop() { 
  // rot = wheelRotary.countA(PIN0, PIN1);
  // Serial.print("rotation:");
  // Serial.println(rot); 

  // Serial.println();
  wheelRotary.countA(PIN0, PIN1);
  wheelRotary.countB(PIN0, PIN1);
  Serial.println(wheelRotary.counter/96);
}

