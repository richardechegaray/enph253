// //right wheel tends to be faster when speeds are set to be equal

// #include <Wire.h>
// #include <Arduino.h>
// #include <pid.h>

// #define FAR_LEFT PB12
// #define LEFT_SENSOR PB13  
// #define RIGHT_SENSOR PB14
// #define FAR_RIGHT PB15

// #define LEFT_MOTOR_FW PA_3 
// #define LEFT_MOTOR_BW PA_2
// #define RIGHT_MOTOR_FW PA_0
// #define RIGHT_MOTOR_BW PA_1

// #define BUFFER_SIZE 5
// #define ON 1
// #define OFF 0

// float clockFreq = 100000;
// float period = 1000;

// float targetSpeed = period/2;
// float leftSpeed = targetSpeed;
// float rightSpeed = targetSpeed;

// // float forwardSpeed = period/6;
// // float turnSpeedWeakSide = period/7;
// // float turnSpeedStrongSide = period/4;

// float leftValue = 0.0;
// float rightValue = 0.0;
// float farLeft = 0.0;
// float farRight = 0.0;

// float error = 0.0;

// float leftBuffer[BUFFER_SIZE];
// float rightBuffer[BUFFER_SIZE];

// pid p_i_d;

// enum state { onTrack, leftOff, rightOff, turnLeft, turnRight, white, malfunc } currentState, previousState;

// float speedVerification(float speed); //add for all functions


// void setup() {
//     Serial.begin(115200);

//     pinMode(LEFT_SENSOR, INPUT_PULLUP); 
//     pinMode(RIGHT_SENSOR, INPUT_PULLUP); 
//     pinMode(FAR_LEFT, INPUT_PULLUP);
//     pinMode(FAR_RIGHT, INPUT_PULLUP);
        
//     pinMode(LEFT_MOTOR_FW, OUTPUT);
//     pinMode(LEFT_MOTOR_BW, OUTPUT);
//     pinMode(RIGHT_MOTOR_FW,OUTPUT);
//     pinMode(RIGHT_MOTOR_BW, OUTPUT);

//     pwm_start(LEFT_MOTOR_FW, clockFreq, period, 0, 1); // initializing all motors
//     pwm_start(LEFT_MOTOR_BW, clockFreq, period, 0, 1); 
//     pwm_start(RIGHT_MOTOR_FW, clockFreq, period, 0, 1); 
//     pwm_start(RIGHT_MOTOR_BW, clockFreq, period, 0, 1); 
//     previousState = onTrack;
//     p_i_d = pid();
// }

// state getState(float left, float right){
//   //if, turn logic right here, separate if statement

//   if ( (left == ON) && (right == ON) )
//     return onTrack;
//   else if ( (right == ON) && (left == OFF) )
//     return leftOff;
//   else if ( (right == OFF) && (left == ON) )
//     return rightOff;
//   else 
//     return white;
// }

// void drive(float bwLeft, float fwLeft, float bwRight, float fwRight) {
//   bwLeft = speedVerification(bwLeft);
//   fwLeft = speedVerification(fwLeft);
//   bwRight = speedVerification(bwRight);
//   fwRight = speedVerification(fwRight);
    
//   pwm_start(LEFT_MOTOR_BW, clockFreq, period, bwLeft, 0); 
//   pwm_start(LEFT_MOTOR_FW, clockFreq, period, fwLeft, 0); 
//   pwm_start(RIGHT_MOTOR_BW, clockFreq, period, bwRight, 0); 
//   pwm_start(RIGHT_MOTOR_FW, clockFreq, period, fwRight, 0); 
// }

// float speedVerification(float speed) {
//   if (speed<0)
//     return period/12;
//   else if (speed>period)
//     return period;
//   else 
//     return speed;
// }

// void loop() {
//   leftValue = digitalRead(LEFT_SENSOR);
//   rightValue = digitalRead(RIGHT_SENSOR);

//   currentState = getState(leftValue, rightValue);
  
//   //Serial.println(currentState);

//   switch ( currentState ) { //state machine

//     case onTrack : //drive straight
//       error = 0; 
//       leftSpeed = targetSpeed + p_i_d.output_pid(error);
//       rightSpeed = targetSpeed + p_i_d.output_pid(error);
//       Serial.println(error);
//       Serial.println(leftSpeed);
//       Serial.println(rightSpeed);
//       Serial.println();
//       drive(0, leftSpeed, 0, rightSpeed);
//       break;

//     case leftOff : //turn right
//       error = 1; 
//       leftSpeed = targetSpeed + p_i_d.output_pid(error);
//       rightSpeed = targetSpeed + p_i_d.output_pid(-error);
//       Serial.println(error);
//       Serial.println(leftSpeed);
//       Serial.println(rightSpeed);
//       Serial.println();
//       drive(0, leftSpeed, 0, rightSpeed); //left needs to catch up, right side weaker
//       break;

//     case rightOff : //turn left
//       error = 1; 
//       leftSpeed = targetSpeed + p_i_d.output_pid(-error);
//       rightSpeed = targetSpeed + p_i_d.output_pid(error);
//       Serial.println(error);
//       Serial.println(leftSpeed);
//       Serial.println(rightSpeed);
//       Serial.println();
//       drive(0, leftSpeed, 0, rightSpeed); //left weaker, right needs to catch up 
//       break;

//     case white : //both sensors off tape
//       error = 5;
//       if(previousState == leftOff) {  // continue to turn right
//         leftSpeed = targetSpeed + p_i_d.output_pid(error);
//         rightSpeed = targetSpeed + p_i_d.output_pid(-error);
//         drive(0, leftSpeed, 0, rightSpeed); //left needs to catch up, right side weaker
//       } else if(previousState == rightOff) {  // continue to turn left
//         leftSpeed = targetSpeed + p_i_d.output_pid(-error);
//         rightSpeed = targetSpeed + p_i_d.output_pid(error);
//         drive(0, leftSpeed, 0, rightSpeed); //left weaker, right needs to catch up
//       } else {
//         drive(0, 0, 0, 0); // do nothing
//         delay(2000); //if it stops this means it went from both on, to both off....
//       }
//       Serial.println(error);
//       Serial.println(leftSpeed);
//       Serial.println(rightSpeed);
//       Serial.println();
//       break;

//     default:
//       drive(0, period/3, period/3, 0); //spin for 5 seconds
//       Serial.println("Default");
//       delay(2000);  // this should never happen...
//       break;  

//   }

//   if (currentState != white) //if currentState is white, then don't update previousState so that we will keep on turning left or right to correct moving off the tape
//     previousState = currentState;

// }