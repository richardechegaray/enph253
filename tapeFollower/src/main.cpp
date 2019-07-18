//right wheel tends to be faster when speeds are set to be equal
#include <Arduino.h>
#include <pid.h>
//40 seconds till u switch

#define FAR_LEFT PB12
#define LEFT_SENSOR PB13  
#define RIGHT_SENSOR PB14
#define FAR_RIGHT PB15

#define LEFT_MOTOR_FW PB_9 
#define LEFT_MOTOR_BW PB_8

#define RIGHT_MOTOR_FW PB_6
#define RIGHT_MOTOR_BW PB_7

#define BUFFER_SIZE 5
#define ON 1
#define OFF 0

float clockFreq = 100000;
float period = 1000;

float initialTime;
float timeElapsed;

float targetSpeed = 50*period/100;
float leftSpeed = targetSpeed;
float rightSpeed = targetSpeed;

float leftValue = 0.0;
float rightValue = 0.0;
float farLeftValue = 0.0;
float farRightValue = 0.0;

float error = 0.0;

float leftBuffer[BUFFER_SIZE];
float rightBuffer[BUFFER_SIZE];

int numberOfTurns = 0;

pid p_i_d;

enum majorState { upRamp, collectPlushie, depositPlushie/*, stones, shutDown*/ } currentMajorState;
enum pidState { onTrack, leftOff, rightOff, turnLeft, turnRight, white, malfunc} currentPidState, previousPidState;

float speedCapOff(float speed); //add for all functions
pidState getPidState(float left, float right, float farLeft, float farRight);
void drive(float bwLeft, float fwLeft, float bwRight, float fwRight);
void pidStateMachine();

void setup() {
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
    initialTime = millis();
    previousPidState = onTrack;
    p_i_d = pid();
}

void loop() {
  timeElapsed = (millis() - initialTime)/1000; // in seconds

  if (timeElapsed < 20)
    currentMajorState = upRamp;
  else if (timeElapsed < 45)
    currentMajorState = collectPlushie;
  else 
    currentMajorState = depositPlushie;

  leftValue = digitalRead(LEFT_SENSOR);
  rightValue = digitalRead(RIGHT_SENSOR);
  farLeftValue = digitalRead(FAR_LEFT);
  farRightValue = digitalRead(FAR_RIGHT);

  //Serial.println(numberOfTurns);

  // Serial.print((int)farLeftValue);
  // Serial.print((int)leftValue);
  // Serial.print((int)rightValue);
  // Serial.println((int)farRightValue);
  // delay(700);

  switch ( currentMajorState ) {
    case upRamp:
      currentPidState = getPidState(leftValue, rightValue, farLeftValue, farRightValue);
      pidStateMachine();
      //    if (numberOfTurns == 2) {
      //  currentMajorState = collectPlushie;
      //}
      break;

    case collectPlushie:
      currentPidState = getPidState(leftValue, rightValue, farLeftValue, farRightValue);
      pidStateMachine();

      // if (numberOfTurns == 4) {
      //   currentMajorState = depositPlushie;
      // }
      break;

    case depositPlushie:
      drive(0, 0, 0, 0);
      //Serial.println("done");
      break;

  } //state machine
}

//increase the first turn left delay
pidState getPidState(float left, float right, float farLeft, float farRight){
   //first check the branch cases to not miss any of the turns
  if (farRight == ON) {
    if (numberOfTurns < 2 && currentMajorState == upRamp) { //if its 0 or 1
      numberOfTurns++;
      if (numberOfTurns == 1) { //first turn
        delay(300); //up ramp delay
      }
      return turnRight;
    }
  }

  if ( currentMajorState == collectPlushie ) {
    if (farLeft == ON)
      return turnLeft; 
  }
  //   if ( farLeft == ON ) {
  //   if ((numberOfTurns < 4) && (numberOfTurns > 1)) { //if its 2 or 3
  //     numberOfTurns++;
  //     return turnLeft;
  //   }
  // }

  if ( (left == ON) && (right == ON) )
    return onTrack;
  else if ( (right == ON) && (left == OFF) )
    return leftOff;
  else if ( (right == OFF) && (left == ON) )
    return rightOff;
  else{
    return white;
  }    
}

void pidStateMachine() {
  switch ( currentPidState ) { //state machine

    case onTrack : //drive straight
      error = 0; 
      leftSpeed = targetSpeed + p_i_d.output_pid(error);
      rightSpeed = targetSpeed + p_i_d.output_pid(error);
      // Serial.println(error);
      // Serial.println(leftSpeed);
      // Serial.println(rightSpeed);
      // Serial.println();
      drive(0, leftSpeed, 0, rightSpeed);
      break;

    case leftOff : //turn right
      error = 1; 
      leftSpeed = targetSpeed + p_i_d.output_pid(error);
      rightSpeed = targetSpeed + p_i_d.output_pid(-error);
      // Serial.println(error);
      // Serial.println(leftSpeed);
      // Serial.println(rightSpeed);
      // Serial.println();
      drive(0, leftSpeed, 0, rightSpeed); //left needs to catch up, right side weaker
      break;

    case rightOff : //turn left
      error = 1; 
      leftSpeed = targetSpeed + p_i_d.output_pid(-error);
      rightSpeed = targetSpeed + p_i_d.output_pid(error);
      // Serial.println(error);
      // Serial.println(leftSpeed);
      // Serial.println(rightSpeed);
      // Serial.println();
      drive(0, leftSpeed, 0, rightSpeed); //left weaker, right needs to catch up 
      break;

    case white : //both sensors off tape
      error = 4;
      if(previousPidState == leftOff) {  // continue to turn right
        leftSpeed = targetSpeed + p_i_d.output_pid(error);
        rightSpeed = targetSpeed + p_i_d.output_pid(-error);
        drive(0, leftSpeed, 0, rightSpeed); //left needs to catch up, right side weaker
      } else if(previousPidState == rightOff) {  // continue to turn left
        leftSpeed = targetSpeed + p_i_d.output_pid(-error);
        rightSpeed = targetSpeed + p_i_d.output_pid(error);
        drive(0, leftSpeed, 0, rightSpeed); //left weaker, right needs to catch up
      } else if(previousPidState == turnLeft){ //continue to turn left
        leftSpeed = targetSpeed + p_i_d.output_pid(-error);
        rightSpeed = targetSpeed + p_i_d.output_pid(error);
        drive(0, leftSpeed, 0, rightSpeed);  
      } else if (previousPidState == turnRight){ //continue to turn right
        leftSpeed = targetSpeed + p_i_d.output_pid(error);
        rightSpeed = targetSpeed + p_i_d.output_pid(-error);
        drive(0, leftSpeed, 0, rightSpeed);  
      } else {
        drive(0, 0, 0, 0); // do nothing
        delay(2000); //if it stops this means it went from both on, to both off....
      }
      // Serial.println(error);
      // Serial.println(leftSpeed);
      // Serial.println(rightSpeed);
      // Serial.println();
      break;
    
    case turnLeft :
      if (numberOfTurns == 1)
        error = 3;
      else
        error = 9;
 
      leftSpeed = targetSpeed + p_i_d.output_pid(-error);
      rightSpeed = targetSpeed + p_i_d.output_pid(error);
      // Serial.println(error);
      // Serial.println(leftSpeed);
      // Serial.println(rightSpeed);
      // Serial.println();
      drive(0, leftSpeed, 0, rightSpeed); //left weaker, right needs to catch up 
      delay(250);  
      if (numberOfTurns == 1) {
        delay(750);
      } 

      break;

    case turnRight : //not doing yet
      if (numberOfTurns == 1) 
        error = 3;
      else
        error = 9;      
      
      leftSpeed = targetSpeed + p_i_d.output_pid(error);
      rightSpeed = targetSpeed + p_i_d.output_pid(-error);
      // Serial.println(error);
      // Serial.println(leftSpeed);
      // Serial.println(rightSpeed);
      // Serial.println();
      drive(0, leftSpeed, 0, rightSpeed); //left needs to catch up, right side weaker
      delay(250); 
     if (numberOfTurns == 1) {
       delay(750);
     } 
      break;

    default:
      drive(0, 0, 0, 0); //spin for 5 seconds
      // Serial.println("Default");
      delay(5000);  // this should never happen...
      break;  
  }

  if (currentPidState != onTrack && currentPidState != white)
    previousPidState = currentPidState;

  // if (numberOfTurns == 1) 
  //   previousPidState == rightOff; //continue to turn left on initial turn
}

void drive(float bwLeft, float fwLeft, float bwRight, float fwRight) { // DO NOT TRY TO RUN FW AND BW DIRECTION FOR ONE WHEEL AT A TIME
  fwLeft = speedCapOff(fwLeft);
  fwRight = speedCapOff(fwRight);

  // if (fwRight < 0) {
  //   bwRight = -fwRight;
  //   fwRight = 0;
  // }
  // if (fwLeft < 0) {
  //   bwLeft = -fwLeft;
  //   fwLeft = 0;
  // }

  bwLeft = speedCapOff(bwLeft);
  bwRight = speedCapOff(bwRight);

  pwm_start(LEFT_MOTOR_BW, clockFreq, period, bwLeft, 0); 
  pwm_start(LEFT_MOTOR_FW, clockFreq, period, fwLeft, 0); 
  pwm_start(RIGHT_MOTOR_BW, clockFreq, period, bwRight, 0); 
  pwm_start(RIGHT_MOTOR_FW, clockFreq, period, fwRight, 0); 
}

float speedCapOff(float speed) {
  if (speed>period)
    return period;
  else if (speed<0)
  return 12*period/100;
  else
    return speed;
}