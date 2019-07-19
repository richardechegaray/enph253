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

#define LEFT_IR PA2
#define MID_IR PA1
#define RIGHT_IR PA0

#define TRIG PB10
#define ECHO PB11

/** PIN INITIALIZATIONS - END **/

/** GLOBAL VARIABLES - START **/
#define ON 1
#define OFF 0
float clockFreq = 100000;
float period = 1000;

float spinSpeed = 22*period/100; //make sure this isnt too fast, 20 is too slow
float targetSpeed = 25*period/100;
float targetSpeedPlus = 30*period/100;
float targetSpeedMinus = 20*period/100;

int leftValue, rightValue, farLeftValue, farRightValue;

float midThreshold = 5000; //past columns
float closeThreshold = 6000;  //quite close

int detectionRange;

enum state { initialSpin, drivingFar, drivingMiddle, drivingClose, /*avoid,*/ stop} currentState;
enum range { far, mid, close } currentDistance;


float leftIntensity, midIntensity, rightIntensity = -1; //initilialize to lowest possible value
int highestPin = -1;
float maxIntensity = 2000; //set to reasonable 'max value', at tape range
int number; //debuggin only
int strongestSignal;

/** GLOBAL VARIABLES - END **/


/** FUNCTION DEFINITIONS - START **/
float capSpeed(float speed);
void drive(float bwLeft, float fwLeft, float bwRight, float fwRight);
void irDrive(range distance);
/** FUNCTION DEFINITIONS - END **/

ultrasonic ultra = ultrasonic(TRIG, ECHO);
IRdecision decision = IRdecision(LEFT_IR, MID_IR, RIGHT_IR, 1); //1 kHz

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
  currentState = initialSpin;
}

void loop() {
  /*number = decision.strongest_signal();
  if (number == LEFT_IR) 
    Serial.println("Left!!!");
  else if (number == MID_IR) 
    Serial.println("Middle!!!");
  else if (number == RIGHT_IR) 
    Serial.println("Right!!!");
  else 
    Serial.println("error finding max");

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
  
  delay(1000);*/

 /**/switch ( currentState ) { //state machine

    case initialSpin : //drive straight
      highestPin = decision.strongest_signal();
      midIntensity = decision.corrcenter;
      drive(spinSpeed, 0, 0, spinSpeed); // spin CCW
      
      if ((highestPin == MID_IR) && (midIntensity > 100)) {
        drive(0,0,0,0);
        currentState = drivingFar;
      }
      break;

    case drivingFar :
      // detectionRange = 25; //ultrasonic
      // if ((ultra.is_there_obj(detectionRange)) && currentState == driving) {
      // currentState = avoid;  
      // }
      midIntensity = decision.corrcenter;
      leftValue = digitalRead(LEFT_SENSOR);
      rightValue = digitalRead(RIGHT_SENSOR);
      farLeftValue = digitalRead(FAR_LEFT);
      farRightValue = digitalRead(FAR_RIGHT);

      if (midIntensity > closeThreshold) {
        currentDistance = close; 
        currentState = drivingClose;
        irDrive(currentDistance);
      } else if (midIntensity > midThreshold) {
        currentDistance = mid;
        currentState = drivingMiddle;
        irDrive(currentDistance);
      } else
        irDrive(currentDistance);

      break;  

    case drivingMiddle :
      midIntensity = decision.corrcenter;
      leftValue = digitalRead(LEFT_SENSOR);
      rightValue = digitalRead(RIGHT_SENSOR);
      farLeftValue = digitalRead(FAR_LEFT);
      farRightValue = digitalRead(FAR_RIGHT);
     
      if (midIntensity > closeThreshold) {
        currentDistance = close; 
        currentState = drivingClose;
        irDrive(currentDistance);
      } else
        irDrive(currentDistance);
      
      break;

    case drivingClose :
      leftValue = digitalRead(LEFT_SENSOR);
      rightValue = digitalRead(RIGHT_SENSOR);
      farLeftValue = digitalRead(FAR_LEFT);
      farRightValue = digitalRead(FAR_RIGHT);

      if ((leftValue || rightValue || farLeftValue || farRightValue == ON) && (midIntensity > closeThreshold)) {
        currentState = stop;
        break;
      }

      irDrive(currentDistance);

      break;    

    // case avoid:
    //   drive(targetSpeed*2, 0, 0, targetSpeed*2);
    //   delay(300);
    //   drive(0, targetSpeed*2, 0, targetSpeed*2);
    //   delay(600);
    //   currentState = initialSpin;
    //   break;
    
    case stop:
      drive(0,0,0,0);
      break;
 }/**/
}

void irDrive(range distance) {
  float speed, speedPlus, speedMinus;
  int maxVal;

  if (distance == far) {
    speed = targetSpeed;
    speedPlus = 110*targetSpeedPlus/100;
    speedMinus = 110*targetSpeedMinus/100;
  } else if (distance == mid) {
    speed = targetSpeed;
    speedPlus = targetSpeedPlus;
    speedMinus = targetSpeedMinus;
  } else if (distance == close) {
    speed = 4*targetSpeed/5;
    speedPlus = 4*targetSpeedPlus/5;
    speedMinus = 4*targetSpeedMinus/5;
  } else {
    speed = speedPlus = speedMinus = 0;
  }

  maxVal = decision.strongest_signal();
  if (maxVal == LEFT_IR) 
    drive(0, speedMinus, 0, speedPlus);
  else if (maxVal == RIGHT_IR) 
    drive(0, speedPlus, 0, speedMinus);
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
