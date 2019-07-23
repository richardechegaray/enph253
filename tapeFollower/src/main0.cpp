// right wheel tends to be faster when speeds are set to be equal
// METHANOS = 1 kHz, turn left twice, then right
// THANOS = 10 kHz, turn right twice, then left

#include <Arduino.h>
#include <pid.h>
#include "IRdecision.h" 
//40 seconds till u switch

#define FAR_LEFT PB12
#define LEFT_SENSOR PB13  
#define RIGHT_SENSOR PB14
#define FAR_RIGHT PB15

#define LEFT_MOTOR_FW PB_9 
#define LEFT_MOTOR_BW PB_8
#define RIGHT_MOTOR_FW PB_6
#define RIGHT_MOTOR_BW PB_7

#define ON 1
#define OFF 0

#define LEFT_IR PA2
#define MID_IR PA1
#define RIGHT_IR PA0

#define TX3 PB10
#define RX3 PB11
HardwareSerial Serial3 = HardwareSerial(RX3, TX3);

float clockFreq = 100000;
float period = 1000;

float initialTime;
float timeElapsed;

float spinSpeed = 22*period/100; //make sure this isnt too fast, 20 is too slow
float targetIrSpeed = 25*period/100;
float targetIrSpeedPlus = 30*period/100;
float targetIrSpeedMinus = 20*period/100;

float targetSpeed = 50*period/100;
float leftSpeed = targetSpeed;
float rightSpeed = targetSpeed;

float midThreshold = 5000; //past columns
float closeThreshold = 6000;  //quite close

float midIntensity = -1;
int highestPin = -1;
//int detectionRange; uSonic

float leftValue = 0.0;
float rightValue = 0.0;
float farLeftValue = 0.0;
float farRightValue = 0.0;

float error = 0.0;
int numberOfTurns = 0;

int turnInfo = 0; //info from the signalProcessing MCU (from the ultrasonic sensor)

enum majorState { upRamp, collectPlushie, depositPlushie, stones/*, shutDown*/ } currentMajorState;
enum pidState { onTrack, leftOff, rightOff, turnLeft, turnRight, white, malfunc} currentPidState, previousPidState;
enum irState { initialSpin, drivingFar, drivingMiddle, drivingClose, /*avoid,*/ stop} currentIrState;
enum range { far, mid, close } currentDistance;


pid p_i_d;

#define THANOS 0
#define METHANOS 1
#define ROLE THANOS

#if (ROLE == THANOS)
  IRdecision decision = IRdecision(LEFT_IR, MID_IR, RIGHT_IR, 10); //10 kHz
#elif (ROLE == METHANOS)
  IRdecision decision = IRdecision(LEFT_IR, MID_IR, RIGHT_IR, 1); //1 kHz
#endif


float speedCapOff(float speed); //add for all functions
pidState getPidState(float left, float right, float farLeft, float farRight);
void drive(float bwLeft, float fwLeft, float bwRight, float fwRight);
void pidStateMachine();
void irStateMachine();
void irDrive(range distance);


void setup() {
    Serial.begin(115200);
    Serial3.begin(115200);

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
    p_i_d = pid();
    initialTime = millis();
    previousPidState = onTrack;
    currentDistance = far;
    currentIrState = initialSpin;
}

void serialComm(){
    int check_available = Serial3.availableForWrite();
    while (!check_available)
        check_available = Serial3.availableForWrite();
    return;
}

void turn(float bwLeft, float fwLeft, float bwRight, float fwRight) { // DO NOT TRY TO RUN FW AND BW DIRECTION FOR ONE WHEEL AT A TIME

  fwLeft = speedCapOff(fwLeft);
  fwRight = speedCapOff(fwRight);
  bwLeft = speedCapOff(bwLeft);
  bwRight = speedCapOff(bwRight);

//   leftValue = digitalRead(LEFT_SENSOR);
//   rightValue = digitalRead(RIGHT_SENSOR);

  pwm_start(LEFT_MOTOR_BW, clockFreq, period, bwLeft, 0); 
  pwm_start(LEFT_MOTOR_FW, clockFreq, period, fwLeft, 0); 
  pwm_start(RIGHT_MOTOR_BW, clockFreq, period, bwRight, 0); 
  pwm_start(RIGHT_MOTOR_FW, clockFreq, period, fwRight, 0); 
}

void handleUltrasonicInfo(int turnInfo){
    switch(turnInfo){
        case 1: //left_center
            //need to turn right until we read black tape again
            //stop the right wheel and write to the left wheel
            turn(0, targetSpeed, 0, 0);
        case 2: //center
            //need to turn left (METHANOS) or right (THANOS) until we read black tape again
            if(ROLE == THANOS)
                drive(0, targetSpeed, 0, 0);
            else if(ROLE == METHANOS)
                drive(0, 0, 0, targetSpeed);
        case 3: //center_right
            //need to turn left until we read black tape again
            drive(0, 0, 0, targetSpeed);
        case 6: //all
            //need to turn left (METHANOS) or right (THANOS) until we read black tape again
            if(ROLE == THANOS)
                drive(0, targetSpeed, 0, 0);
            else if(ROLE == METHANOS)
                drive(0, 0, 0, targetSpeed);
    }
}

void loop() {
  timeElapsed = (millis() - initialTime)/1000; // in seconds

  if (timeElapsed < 13)
    currentMajorState = upRamp;
  else if (timeElapsed < 28)
    currentMajorState = collectPlushie;
  else 
    currentMajorState = depositPlushie;

  leftValue = digitalRead(LEFT_SENSOR);
  rightValue = digitalRead(RIGHT_SENSOR);
  farLeftValue = digitalRead(FAR_LEFT);
  farRightValue = digitalRead(FAR_RIGHT);

  // Serial.print((int)farLeftValue);
  // Serial.print((int)leftValue);
  // Serial.print((int)rightValue);
  // Serial.println((int)farRightValue);
  // delay(700);

  //send the current state to the signalProcessing MCU
  serialComm();
  Serial3.write(currentMajorState);

  switch ( currentMajorState ) { // state machine
    case upRamp:
      currentPidState = getPidState(leftValue, rightValue, farLeftValue, farRightValue);
      pidStateMachine();
      break;

    case collectPlushie:
      currentPidState = getPidState(leftValue, rightValue, farLeftValue, farRightValue);
      pidStateMachine();
      break;

    case depositPlushie:
      irStateMachine();
      break;

    case stones:
      drive(targetSpeed, 0, targetSpeed, 0);
      break;

  }
  turnInfo = Serial3.read(); //read info from the signalProcessing MCU to see if we need to prevent a collision
  handleUltrasonicInfo(turnInfo); //send the info
}

//increase the first turn left delay
pidState getPidState(float left, float right, float farLeft, float farRight){

  #if (ROLE == THANOS) 
    if (farLeft == ON) {
      if (numberOfTurns < 2 && currentMajorState == upRamp) { //if its 0 or 1
        numberOfTurns++;
        if (numberOfTurns == 1)  //first turn
          delay(300); //up ramp delay
        return turnLeft;
      }
    }

    if ( currentMajorState == collectPlushie ) {
      if (farRight == ON)
        return turnRight; 
    }
  #elif (ROLE == METHANOS) 
    if (farRight == ON) {
      if (numberOfTurns < 2 && currentMajorState == upRamp) { //if its 0 or 1
        numberOfTurns++;
        if (numberOfTurns == 1)  //first turn
          delay(300); //up ramp delay
        return turnRight;
      }
    }

    if ( currentMajorState == collectPlushie ) {
      if (farLeft == ON)
        return turnLeft; 
    }
  #endif

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

//modular
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
      error = 3; //4
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
        error = 2;  //3
      else
        error = 7; //9
 
      leftSpeed  = targetSpeed + p_i_d.output_pid(-error);
      rightSpeed = targetSpeed + p_i_d.output_pid(error);
      // Serial.println(error);
      // Serial.println(leftSpeed);
      // Serial.println(rightSpeed);
      // Serial.println();
      drive(0, leftSpeed, 0, rightSpeed); //left weaker, right needs to catch up 
      delay(200);  
      if (numberOfTurns == 1) 
        delay(800);

      break;

    case turnRight : //not doing yet
      if (numberOfTurns == 1) 
        error = 2;  //3
      else
        error = 7;  //9      
      
      leftSpeed = targetSpeed + p_i_d.output_pid(error);
      rightSpeed = targetSpeed + p_i_d.output_pid(-error);
      // Serial.println(error);
      // Serial.println(leftSpeed);
      // Serial.println(rightSpeed);
      // Serial.println();
      drive(0, leftSpeed, 0, rightSpeed); //left needs to catch up, right side weaker
      delay(200); 
      if (numberOfTurns == 1) 
        delay(800);
      
      break;

    default:
      drive(0, 0, 0, 0); //spin for 5 seconds
      delay(5000);  // this should never happen...
      break;  
  }

  if (currentPidState != onTrack && currentPidState != white)
    previousPidState = currentPidState;

  // if (numberOfTurns == 1) 
  //   previousPidState == rightOff; //continue to turn left on initial turn
}

//modular
void irStateMachine() {
  switch ( currentIrState ) { //state machine
    case initialSpin : //drive straight
      highestPin = decision.strongest_signal();
      midIntensity = decision.corrcenter;
      drive(spinSpeed, 0, 0, spinSpeed); // spin CCW
      
      if ((highestPin == MID_IR) && (midIntensity > 100)) {
        drive(0,0,0,0);
        currentIrState = drivingFar;
      }
      break;

    case drivingFar :
      
      midIntensity = decision.corrcenter;

      if (midIntensity > closeThreshold) {
        currentDistance = close; 
        currentIrState = drivingClose;
      } else if (midIntensity > midThreshold) {
        currentDistance = mid;
        currentIrState = drivingMiddle;
      } 
        
      irDrive(currentDistance);
      break;  

    case drivingMiddle :
      midIntensity = decision.corrcenter;
     
      if (midIntensity > closeThreshold) {
        currentDistance = close; 
        currentIrState = drivingClose;
      } 

     if (leftValue || rightValue || farLeftValue || farRightValue == ON) {
        currentIrState = stop;
        break;
      }      
      
      irDrive(currentDistance);
      break;

    case drivingClose :
      midIntensity = decision.corrcenter;
      leftValue = digitalRead(LEFT_SENSOR);
      rightValue = digitalRead(RIGHT_SENSOR);
      farLeftValue = digitalRead(FAR_LEFT);
      farRightValue = digitalRead(FAR_RIGHT);

      if (leftValue || rightValue || farLeftValue || farRightValue == ON) {
        currentIrState = stop;
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
      delay(1500);
      currentMajorState = stones;
      break;
  }
}

//modular
void drive(float bwLeft, float fwLeft, float bwRight, float fwRight) { // DO NOT TRY TO RUN FW AND BW DIRECTION FOR ONE WHEEL AT A TIME


  if ((currentPidState == turnLeft)||(currentPidState == turnRight)) {
    if (fwRight < 0) {
      bwRight = -fwRight;
      fwRight = 0;
    }
    if (fwLeft < 0) {
      bwLeft = -fwLeft;
      fwLeft = 0;
    }
  }

  fwLeft = speedCapOff(fwLeft);
  fwRight = speedCapOff(fwRight);
  bwLeft = speedCapOff(bwLeft);
  bwRight = speedCapOff(bwRight);

  pwm_start(LEFT_MOTOR_BW, clockFreq, period, bwLeft, 0); 
  pwm_start(LEFT_MOTOR_FW, clockFreq, period, fwLeft, 0); 
  pwm_start(RIGHT_MOTOR_BW, clockFreq, period, bwRight, 0); 
  pwm_start(RIGHT_MOTOR_FW, clockFreq, period, fwRight, 0); 
}

//modular
void irDrive(range distance) {
  float speed, speedPlus, speedMinus;

  if (distance == far) {
    speed = targetSpeed; // turn sharper if far away
    speedPlus = 110*targetIrSpeedPlus/100;
    speedMinus = 110*targetIrSpeedMinus/100;
  } else if (distance == mid) {
    speed = targetSpeed;
    speedPlus = targetIrSpeedPlus;
    speedMinus = targetIrSpeedMinus;
  } else if (distance == close) {
    speed = 4*targetSpeed/5;
    speedPlus = 4*targetIrSpeedPlus/5;
    speedMinus = 4*targetIrSpeedMinus/5;
  } else {
    speed = speedPlus = speedMinus = 0;
  }

  highestPin = decision.strongest_signal();
  if (highestPin == LEFT_IR) 
    drive(0, speedMinus, 0, speedPlus);
  else if (highestPin == RIGHT_IR) 
    drive(0, speedPlus, 0, speedMinus);
  else
    drive(0, speed, 0, speed);
}

//modular
float speedCapOff(float speed) {
  if (speed>period)
    return period;
  else if (speed<0)
  return 12*period/100;
  else
    return speed;
}