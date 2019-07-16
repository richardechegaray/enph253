#include <Arduino.h>

// ******************** ROTARY ENCODER ******************************************************************************************************
// #include <robot.cpp>
// #include "WheelRotaries.h"
// #define PIN0 PA0
// #define PIN1 PA1
// ******************************************************************************************************************************************

// ******************* INFRARED SENSOR ******************************************************************************************************
// #include "IRdecision.h"
// #include "IRsensor.h"
// #define PIN_LEFT PA0
// #define PIN_CENTER PA1
// #define PIN_RIGHT PA2
// #define PIN PA0
// ******************************************************************************************************************************************

// ****************** ULTRASONIC SENSOR *****************************************************************************************************
#include "ultrasonic.h"
#define TRIG PB10
#define ECHO PB11

ultrasonic ultra = ultrasonic(TRIG, ECHO);
int distance;
int range;
bool yesorno;

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("Setup done");
}

void loop() {
    // test 1: observe range that it can reliably detect
    // closest: it reads reliably from 11+ cm, can still handle up to 7 before it gets whack, but it'll give values off by a couple cm
    // farthest: over a metre!!! we gucci boys
    // put circuit on a ruler and pull back from object (robot, then pillar) until it's not working as expected

    // distance = ultra.get_distance();
    // Serial.print("Distance: ");
    // Serial.println(distance);
    // delay(1000);

    // test 2: see if it triggers when objects enter range
    // looked at range side to side: at 20cm distance, range right to left is only about 20cm
    range = 25;
    yesorno = ultra.is_there_obj(range);
    Serial.print("Object within range: ");
    Serial.println(yesorno); 
    delay(1000);

    // test 3: put ultrasonic on robot to see if we can detect pillars in front range without servo
    // range =  25;
    // yesorno = ultra.is_there_obj(range);
    // if (yesorno)
    //     // light an LED?
    // delay(1000)

}

// ******************* INFRARED SENSOR ******************************************************************************************************
// IRdecision decision = IRdecision(PIN_LEFT, PIN_CENTER, PIN_RIGHT, 10);
// IRsensor sensor = IRsensor(PIN, 1);

// void setup(){
//   Serial.begin(115200); 
//   delay(2000);
//   Serial.println("Setup done");
// }

// void loop() { 
//   sensor.corr();
//   float duration = sensor.duration;
//   float average = sensor.average;
//   float correlation = sensor.correlation;

//   Serial.print("duration: ");
//   Serial.println(duration*1000000);

//   Serial.print("average: ");
//   Serial.println(average);

//   Serial.print("correlation: ");
//   Serial.println(correlation);

//   delay(1000);

//   int max_pin;
//   max_pin = decision.strongest_signal();

//   float left_correlation, center_correlation, right_correlation;
//   left_correlation = decision.corrleft;
//   center_correlation = decision.corrcenter;
//   right_correlation = decision.corrright;

//   Serial.print("left: ");
//   Serial.println(left_correlation);

//   Serial.print("center: ");
//   Serial.println(center_correlation);

//   Serial.print("right: ");
//   Serial.println(right_correlation);

//   Serial.print("max correlating pin: ");
//   Serial.println(max_pin);
// }
// ******************************************************************************************************************************************

// ******************** ROTARY ENCODER ******************************************************************************************************
// WheelRotary wheelRotary = WheelRotary(PIN0, PIN1);
// unsigned rot = 0;

// void setup() {
//     Serial.begin(115200); 
//     delay(2000);
//     Serial.println("Setup done");
// }

// void loop() {
//   rot = wheelRotary.countA(PIN0, PIN1);
//   Serial.print("rotation:");
//   Serial.println(rot); 

//   Serial.println();
//   wheelRotary.countA(PIN0, PIN1);
//   wheelRotary.countB(PIN0, PIN1);
//   Serial.println(wheelRotary.counter/96);
// }
// ******************************************************************************************************************************************

// ************************* ROBOT **********************************************************************************************************
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
// ******************************************************************************************************************************************


