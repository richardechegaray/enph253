#include <Arduino.h>
#include "IRdecision.h" 
#include "ultrasonic.h"

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

#define TRIG PB10
#define ECHO PB11

/** PIN INITIALIZATIONS - END **/

/** GLOBAL VARIABLES - START **/
#define ON 1
#define OFF 0
float clockFreq = 100000;
float period = 1000;

float spinSpeed = 18*period/100; //make sure this isnt too fast
float targetSpeed = 25*period/100;
float targetSpeedPlus = 30*period/100;
float targetSpeedMinus = 20*period/100;

int leftValue, rightValue, farLeftValue, farRightValue;

float midThreshold = 2000;
float closeThreshold = 1100;

int detectionRange;

enum state { initialSpin, driving, avoid} currentState;
enum range { far, mid, close } currentDistance;


float leftIntensity, midIntensity, rightIntensity = -1; //initilialize to lowest possible value
int highestPin = -1;
float maxIntensity = 2000; //set to reasonable 'max value', at tape range

/** GLOBAL VARIABLES - END **/


/** FUNCTION DEFINITIONS - START **/
float capSpeed(float speed);
void drive(float bwLeft, float fwLeft, float bwRight, float fwRight);
void irDrive(range distance);
/** FUNCTION DEFINITIONS - END **/

ultrasonic ultra = ultrasonic(TRIG, ECHO);
IRdecision decision = IRdecision(LEFT_IR, MID_IR, RIGHT_IR, 1);

void setup(){
  Serial.begin(115200); 
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

void loop() { 
  detectionRange = 25;
  if (ultra.is_there_obj(detectionRange))
    currentState = avoid;

 switch ( currentState ) { //state machine

    case initialSpin : //drive straight
      highestPin = decision.strongest_signal();
      drive(spinSpeed, 0, 0, spinSpeed); // spin CCW
      
      if (highestPin == MID_IR)
        currentState = driving;

      break;

    case driving :
      // leftValue = digitalRead(LEFT_SENSOR);
      // rightValue = digitalRead(RIGHT_SENSOR);
      // farLeftValue = digitalRead(FAR_LEFT);
      // farRightValue = digitalRead(FAR_RIGHT);

      // if (leftValue || rightValue || farLeftValue || farRightValue == ON)
      //   drive(0,0,0,0);

      if (decision.strongest_signal() == LEFT_IR) 
        drive(0, targetSpeedMinus, 0, targetSpeedPlus);
      else if (decision.strongest_signal() == RIGHT_IR) 
        drive(0, targetSpeedPlus, 0, targetSpeedMinus);
      else
        drive(0, targetSpeed, 0, targetSpeed);
      break;

    case avoid:
      drive(targetSpeed*2, 0, 0, targetSpeed*2);
      delay(300);
      drive(0, targetSpeed*2, 0, targetSpeed*2);
      delay(600);
      currentState = initialSpin;
      break;
 }
}

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
    speed = 4*targetSpeed/3;
    speedPlus = 4*targetSpeedPlus/3;
    speedMinus = 4*targetSpeedMinus/3;
  } else if (distance == mid) {
    speed = targetSpeed;
    speedPlus = targetSpeedPlus;
    speedMinus = targetSpeedMinus;
  } else if (distance == close) {
    speed = targetSpeed/3;
    speedPlus = targetSpeedPlus/3;
    speedMinus = targetSpeedMinus/3;
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
