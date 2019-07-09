#include <Wire.h>
#include <Arduino.h>
#include <pid.h>

#define LEFT_SENSOR PB13 
#define RIGHT_SENSOR PB14
#define FAR_RIGHT PB15
#define FAR_LEFT PB12

#define LEFT_MOTOR_FW PB_9 
#define LEFT_MOTOR_BW PB_8

#define RIGHT_MOTOR_FW PB_6
#define RIGHT_MOTOR_BW PB_7

#define BUFFER_SIZE 5
#define ON 1
#define OFF 0

float clockFreq = 100000;
float period = 1000;
float target_speed = period * 0.2;

float forwardSpeed = period/4;
float turnSpeedWeakSide = period/5;
float turnSpeedStrongSide = 3*period/5;

float leftValue = 0.0;
float rightValue = 0.0;

float leftError = 0.0;
float rightError = 0.0;

float leftBuffer[BUFFER_SIZE];
float rightBuffer[BUFFER_SIZE];

enum state { onTrack, leftOff, rightOff, turnLeft, turnRight, white, error } currentState, previousState;

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
    previousState = onTrack;
    //p_i_d = pid();
}

state getState(float left, float right){

  //if, turn logic right here, separate if statement

  if ( (left == ON) && (right == ON) )
    return onTrack;
  else if ( (right == ON) && (left == OFF) )
    return leftOff;
  else if ( (right == OFF) && (left == ON) )
    return rightOff;
  else if ( (left == OFF ) && (right == OFF) )
    return white;
  else {
    Serial.println("Error with digitalReading");
    return error;
  }
}

float read_mean(float arr[]){
    float sum = 0;
    for(unsigned i = 0; i< BUFFER_SIZE; i++){
        sum += arr[i];
    }
    return (float)sum/BUFFER_SIZE;
}

float reading(float arr[], int ir_sensor){
    for(unsigned i = 0; i < BUFFER_SIZE; i++){
        arr[i] = analogRead(ir_sensor);
    }
    return read_mean(arr);
}

void drive(float bwLeft, float fwLeft, float bwRight, float fwRight) {
  pwm_start(LEFT_MOTOR_BW, clockFreq, period, bwLeft, 0); 
  pwm_start(LEFT_MOTOR_FW, clockFreq, period, fwLeft, 0); 
  pwm_start(RIGHT_MOTOR_BW, clockFreq, period, bwRight, 0); 
  pwm_start(RIGHT_MOTOR_FW, clockFreq, period, fwRight, 0); 
}

void loop() {
  leftValue = digitalRead(LEFT_SENSOR);
  rightValue = digitalRead(RIGHT_SENSOR);

  currentState = getState(leftValue, rightValue);
  Serial.println(currentState);

  switch ( currentState ) { //state machine

    case onTrack : //drive straight
      drive(0, forwardSpeed, 0, forwardSpeed);
      break;

    case leftOff : //turn right
      drive(0, turnSpeedStrongSide, 0, turnSpeedWeakSide); //left needs to catch up, right side weaker
      break;

    case rightOff : //turn left
      drive(0, turnSpeedWeakSide, 0, turnSpeedStrongSide); //left weaker, right needs to catch up
      break;

    case white : //both sensors off tape
      if(previousState == leftOff)      // continue to turn right
        drive(0, turnSpeedStrongSide, 0, turnSpeedWeakSide); //left needs to catch up, right side weaker
      else if(previousState == rightOff)  // continue to turn left
        drive(0, turnSpeedWeakSide, 0, turnSpeedStrongSide); //left weaker, right needs to catch up
      else 
        drive(0, 0, 0, 0); // do nothing
      break;

    case error:
      drive(forwardSpeed, 0, 0, forwardSpeed); //spin counterclockwise
      delay(3000);
      break;

    default:
      drive(0, forwardSpeed, forwardSpeed, 0); //spin clockwise
      delay(3000);
      break;

    previousState = currentState;
  }

}

// void loop() {
//   leftValue = digitalRead(LEFT_SENSOR);
//   rightValue = digitalRead(RIGHT_SENSOR);

//   if ((leftValue && rightValue) == ON)
//     drive(0,forwardSpeed, 0, forwardSpeed);
//   else if (leftValue == ON)
//     drive(0,forwardSpeed, 0, 0);
//   else if (rightValue == ON)
//     drive(0, 0, 0, forwardSpeed);
//   else
//     drive(0,0,0,0);
// }