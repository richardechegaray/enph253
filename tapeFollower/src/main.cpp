#include <Wire.h>
#include <Arduino.h>
#include <pid.h>

#define LEFT_SENSOR PA3 
#define RIGHT_SENSOR PA5

#define LEFT_MOTOR_FW PA_6
#define LEFT_MOTOR_BW PB_1
#define RIGHT_MOTOR_FW PA_2
#define RIGHT_MOTOR_BW PB_0

#define BUFFER_SIZE 5
#define ON 1
#define OFF 0

float clockFreq = 100000;
float period = 1000;
float target_speed = period * 0.2;

float leftValue = 0.0;
float rightValue = 0.0;

float leftError = 0.0;
float rightError = 0.0;

float leftBuffer[BUFFER_SIZE];
float rightBuffer[BUFFER_SIZE];

enum state { onTrack, leftOff, rightOff, turnLeft, turnRight, error } currentState;

void setup() {
    Serial.begin(115200);
    pinMode(LEFT_SENSOR, INPUT); 
    pinMode(RIGHT_SENSOR, INPUT); 
        
    pinMode(LEFT_MOTOR_FW, OUTPUT);
    pinMode(LEFT_MOTOR_BW, OUTPUT);
    pinMode(RIGHT_MOTOR_FW,OUTPUT);
    pinMode(RIGHT_MOTOR_BW, OUTPUT);

    pwm_start(LEFT_MOTOR_FW, clockFreq, period, 0, 1); // initialize, make it do nothing
    pwm_start(LEFT_MOTOR_BW, clockFreq, period, 0, 1); //initialize, make it do nothing
    pwm_start(RIGHT_MOTOR_FW, clockFreq, period, 0, 1); //initialize, make it do nothing
    pwm_start(RIGHT_MOTOR_BW, clockFreq, period, 0, 1); 
    //p_i_d = pid();
}

state getState(float left, float right){
  if ( (left && right) == ON )
    return onTrack;
  else if ( (right == ON) && (left == OFF) )
    return leftOff;
  else if ( (right == OFF) && (left == ON) )
    return rightOff;
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

void loop() {
  leftValue = digitalRead(LEFT_SENSOR);
  rightValue = digitalRead(RIGHT_SENSOR);

  currentState = getState(leftValue, rightValue);

  switch ( currentState ) { //state machine

    case onTrack :
      pwm_start(LEFT_MOTOR_BW, clockFreq, period, 0, 0); // off
      pwm_start(RIGHT_MOTOR_BW, clockFreq, period, 0, 0); // off
      pwm_start(LEFT_MOTOR_FW, clockFreq, period, period/2, 0); // half speed
      pwm_start(RIGHT_MOTOR_FW, clockFreq, period, period/2, 0); // half speed
      break;

    case leftOff :
      pwm_start(LEFT_MOTOR_BW, clockFreq, period, 0, 0); // off
      pwm_start(RIGHT_MOTOR_BW, clockFreq, period, 0, 0); // off
      pwm_start(LEFT_MOTOR_FW, clockFreq, period, 4*period/5, 0); // left motor 4/5 speed, to catch up
      pwm_start(RIGHT_MOTOR_FW, clockFreq, period, period/5, 0); // right motor 1/5 speed
      break;

    case rightOff :
      pwm_start(LEFT_MOTOR_BW, clockFreq, period, 0, 0); // off
      pwm_start(RIGHT_MOTOR_BW, clockFreq, period, 0, 0); // off
      pwm_start(LEFT_MOTOR_FW, clockFreq, period, period/5, 0); // left motor 1/5 speed
      pwm_start(RIGHT_MOTOR_FW, clockFreq, period, 4*period/5, 0); // right motor 4/5 speed, to catch up
      break;
  }

}