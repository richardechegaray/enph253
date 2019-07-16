#include <Arduino.h>
#include "IRdecision.h" 
/** PIN INITIALIZATIONS - START **/
#define FAR_LEFT PB12
#define LEFT_SENSOR PB13  
#define RIGHT_SENSOR PB14
#define FAR_RIGHT PB15

#define LEFT_MOTOR_FW PB_9 
#define LEFT_MOTOR_BW PB_8
#define RIGHT_MOTOR_FW PB_6
#define RIGHT_MOTOR_BW PB_7

#define LEFT_IR PA0
#define MID_IR PA1
#define RIGHT_IR PA2
/** PIN INITIALIZATIONS - END **/

/** GLOBAL VARIABLES - START **/
#define ON 1
#define OFF 0
float clockFreq = 100000;
float period = 1000;

float spinSpeed = 8*period/100; //make sure this isnt too fast
float targetSpeed = 20*period/100;
float targetSpeedPlus = 23*period/100;
float targetSpeedMinus = 17*period/100;

int leftValue, rightValue, farLeftValue, farRightValue;

float midThreshold = 0;
float closeThreshold = 0;

enum state { initialSpin, /*orient,*/ driving} currentState;
enum range { far, mid, close } currentDistance;


float leftIntensity, midIntensity, rightIntensity = -100; //initilialize to lowest possible value
float maxIntensity = 0; //set to reasonable 'max value'
//
float threshold = 0; //TBD !!!
//
/** GLOBAL VARIABLES - END **/


/** FUNCTION DEFINITIONS - START **/
float capSpeed(float speed);
void drive(float bwLeft, float fwLeft, float bwRight, float fwRight);
void irDrive(range distance);
/** FUNCTION DEFINITIONS - END **/


IRdecision decision = IRdecision(LEFT_IR, MID_IR, RIGHT_IR, 1);

void setup(){
  Serial.begin(115200); 
  Serial.println("Setup done");
  pinMode(LEFT_SENSOR, INPUT_PULLUP); 
  pinMode(RIGHT_SENSOR, INPUT_PULLUP); 
  pinMode(FAR_LEFT, INPUT_PULLUP);
  pinMode(FAR_RIGHT, INPUT_PULLUP);
        
  pinMode(LEFT_MOTOR_FW, OUTPUT);
  pinMode(LEFT_MOTOR_BW, OUTPUT);
  pinMode(RIGHT_MOTOR_FW,OUTPUT);
  pinMode(RIGHT_MOTOR_BW, OUTPUT);

  pwm_start(LEFT_MOTOR_FW, clockFreq, period, 0, 1); // initializing all motors
  pwm_start(LEFT_MOTOR_BW, clockFreq, period, 0, 1); 
  pwm_start(RIGHT_MOTOR_FW, clockFreq, period, 0, 1); 
  pwm_start(RIGHT_MOTOR_BW, clockFreq, period, 0, 1);   
  currentDistance = far;

}
/*** TEST 2b ***/
void loop() { 
 
}

/*** TEST 0 ***/  /*
void loop() { 
  // write down the values ! ! ! 
  delay(1000);

  int max_pin;
  max_pin = decision.strongest_signal();

  leftIntensity = decision.corrleft;
  midIntensity = decision.corrcenter;
  rightIntensity = decision.corrright;

  Serial.print("Left: ");
  Serial.println(leftIntensity);

  Serial.print("Middle: ");
  Serial.println(midIntensity);

  Serial.print("Right: ");
  Serial.println(rightIntensity);

  Serial.print("Max correlation pin: ");
  Serial.println(max_pin);
    
  Serial.println("-");

}  */

/*** TEST 1a ***/   /*
void loop() { 
 switch ( currentState ) { //state machine

    case initialSpin : //drive straight
      midIntensity = decision.corrcenter;
      drive(0, spinSpeed, spinSpeed, 0); // spin CW
      if (midIntensity > maxIntensity) 
        maxIntensity = midIntensity;
      
      if ((midIntensity < maxIntensity+threshold) && (midIntensity > maxIntensity-threshold))
        currentState = driving;
      break;

    case driving :
      drive(0, targetSpeed, 0, targetSpeed);
      break;

 }
}   */

/*** TEST 1b ***/   /*
void loop() { 
  switch ( currentState ) { //state machine
    case initialSpin : //drive straight
      midIntensity = decision.corrcenter;
      drive(0, spinSpeed, spinSpeed, 0); // spin CW
      if (midIntensity > maxIntensity) 
        maxIntensity = midIntensity;
      
      if ((midIntensity < maxIntensity+threshold) && (midIntensity > maxIntensity-threshold))
        currentState = driving;
      break;

    case driving :
      if (decision.strongest_signal() == LEFT_IR) 
        drive(0, targetSpeedPlus, 0, targetSpeedMinus);
      else if (decision.strongest_signal() == RIGHT_IR) 
        drive(0, targetSpeedMinus, 0, targetSpeedPlus);
      else
        drive(0, targetSpeed, 0, targetSpeed);
      break;
  }
}   */

/*** TEST 2b ***/    /*
void loop() { 
  switch ( currentState ) { //state machine
    case initialSpin : //drive straight
      midIntensity = decision.corrcenter;
      drive(0, spinSpeed, spinSpeed, 0); // spin CW
      if (midIntensity > maxIntensity) 
        maxIntensity = midIntensity;
      
      if ((midIntensity < maxIntensity+threshold) && (midIntensity > maxIntensity-threshold))
        currentState = driving;
      break;

    case driving :
      leftValue = digitalRead(LEFT_SENSOR);
      rightValue = digitalRead(RIGHT_SENSOR);
      farLeftValue = digitalRead(FAR_LEFT);
      farRightValue = digitalRead(FAR_RIGHT);
      midIntensity = decision.corrcenter;

      if (leftValue || rightValue || farLeftValue || farRightValue == ON)
        drive(0,0,0,0);
      else if (midIntensity<closeThreshold) 
        irDrive(close);
      else if (midIntensity<midThreshold)
        irDrive(mid);
      else
        irDrive(far);
      break;
  }
}   */

void irDrive(range distance) {
  float speed, speedPlus, speedMinus;

  if (distance == far) {
    speed = targetSpeed;
    speedPlus = targetSpeedPlus;
    speedMinus = targetSpeedMinus;
  } else if (distance == mid) {
    speed = targetSpeed/2;
    speedPlus = targetSpeedPlus/2;
    speedMinus = targetSpeedMinus/2;
  } else if (distance == close) {
    speed = targetSpeed/4;
    speedPlus = targetSpeedPlus/4;
    speedMinus = targetSpeedMinus/4;
  } else {
    speed = speedPlus = speedMinus = 0;
  }

  if (decision.strongest_signal() == LEFT_IR) 
    drive(0, speedPlus, 0, speedMinus);
  else if (decision.strongest_signal() == RIGHT_IR) 
    drive(0, speedMinus, 0, speedPlus);
  else
    drive(0, speed, 0, speed);
}

void drive(float bwLeft, float fwLeft, float bwRight, float fwRight) {
  fwLeft = capSpeed(fwLeft);
  fwRight = capSpeed(fwRight);
  bwLeft = capSpeed(bwLeft);
  bwRight = capSpeed(bwRight);

  pwm_start(LEFT_MOTOR_BW, clockFreq, period, bwLeft, 0); 
  pwm_start(LEFT_MOTOR_FW, clockFreq, period, fwLeft, 0); 
  pwm_start(RIGHT_MOTOR_BW, clockFreq, period, bwRight, 0); 
  pwm_start(RIGHT_MOTOR_FW, clockFreq, period, fwRight, 0); 
}

float capSpeed(float speed) {
  if (speed>period)
    return period;
  else if (speed<0)
  return period/10;
  else
    return speed;
}
