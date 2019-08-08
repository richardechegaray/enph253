#include <Arduino.h>
#include <pid.h>
#include "IRdecision.h" 
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <FreeMono9pt7b.h>
#define OLED_RESET -1 //for reset button
Adafruit_SSD1306 display(OLED_RESET);

#define FAR_LEFT PB12
#define STONE_LEFT PB13
#define LEFT_MID PA15
#define MID_MID PA12
#define RIGHT_MID PA11
#define STONE_RIGHT PB14
#define FAR_RIGHT PB15

#define LEFT_MOTOR_FW PA_3 
#define LEFT_MOTOR_BW PA_2
#define RIGHT_MOTOR_FW PA_0
#define RIGHT_MOTOR_BW PA_1

#define ON 1
#define OFF 0

#define LEFT_IR PB0
#define MID_IR PB1
#define RIGHT_IR PA7

#define TX3 PB10
#define RX3 PB11
HardwareSerial Serial3 = HardwareSerial(RX3, TX3);

#define KP_METER PA4
#define KD_METER PA5
#define RESET_BUTTON PB8
#define MODE_SWITCH PB5
#define MOOD_SWITCH PB4  //SHARP TURNS

//#define RAMP_TIME 15
// #define COLLECT_TIME 29
// #define SMALL_LOOP_TIME 17
// #define SMALL_COLLECT_TIME 24
// #define BIG_COLLECT_TIME 8
#define RAMP_TIME 15
#define SMALL_TURN_TIME_GM 18.3
#define SMALL_TURN_TIME_BM 14.8
#define SMALL_COLLECT_TIME_GM 26 //26.8
#define SMALL_COLLECT_TIME_BM 26.8
#define BIG_COLLECT_TIME 14

#define METHANOS_TURN_TIME_GM 0.7 //for driving past the first 2 pillars, sharp turn
#define THANOS_TURN_TIME_GM 0.7  // for driving past the first 2 pillars, sharp turn
#define METHANOS_TURN_TIME_BM 0.6
#define THANOS_TURN_TIME_BM 0.6

float clockFreq = 100000;
float period = 1000;

float initialTime;
float timeElapsed;
float collisionStartTime;   
float collisionTimeInterval;

int isThereCollision;

float spinSpeed = 24*period/100; //make sure this isnt too fast, had 22
float targetIrSpeed = 22*period/100;  //25
float targetIrSpeedPlus = 27*period/100;  //30
float targetIrSpeedMinus = 17*period/100;  //20

float targetSpeed = 60*period/100;
float leftSpeed = targetSpeed;
float rightSpeed = targetSpeed;
float collisionSpeed = 50*period/100;

float irStartThreshold;
float closeThreshold; //past columns

float midIntensity = -1;
int highestPin = -1;

int farLeftVal = 0;
int stoneLeftVal = 0;
int leftMidVal = 0;
int midMidVal = 0;
int rightMidVal = 0;
int stoneRightVal = 0;
int farRightVal = 0;

bool miniLoopDone;
bool miniStateDone;
bool mood;

float timeOut;
float time;

float error = 0.0;
int numberOfTurns;
uint8_t majState;

enum majorState { upRamp, collectPlushie, depositPlushie, shutDown } ;
enum pidState { onTape, offOnOn, offOffOn, onOnOff, onOffOff, white, turnLeft, turnRight } ;
enum irState { initialSpin, drivingFar, drivingClose, adjust, stop } ;
enum collisionState { firstTurn, driveStraight, lastTurn } ;

majorState currentMajorState, previousMajorState;
irState currentIrState;
pidState  currentPidState, previousPidState;
collisionState currentCollisionState;

pid p_i_d;

int role;
#define THANOS 0
#define METHANOS 1
IRdecision decision;

float speedCapOff(float speed); //add for all functions
pidState getPidState(int farLeft, int stoneLeft, int leftMid, int midMid, int rightMid, int stoneRight, int farRight);
void drive(float bwLeft, float fwLeft, float bwRight, float fwRight);
void pidStateMachine();
void irStateMachine();
void irDrive(irState currentIrState);
void updatePotVal();
void updateDisplay();
void cutAcross();
void collisionStateMachine();

void setup() {
    Serial.begin(115200);
    Serial3.begin(9600);

    pinMode(FAR_LEFT, INPUT_PULLUP); 
    pinMode(STONE_LEFT, INPUT_PULLUP); 
    pinMode(LEFT_MID, INPUT_PULLUP);
    pinMode(MID_MID, INPUT_PULLUP);
    pinMode(RIGHT_MID, INPUT_PULLUP); 
    pinMode(STONE_RIGHT, INPUT_PULLUP);
    pinMode(FAR_RIGHT, INPUT_PULLUP);
        
    pinMode(LEFT_MOTOR_FW, OUTPUT);
    pinMode(LEFT_MOTOR_BW, OUTPUT);
    pinMode(RIGHT_MOTOR_FW,OUTPUT);
    pinMode(RIGHT_MOTOR_BW, OUTPUT);

    pwm_start(LEFT_MOTOR_FW, clockFreq, period, 0, 1); // Initializing all motors
    pwm_start(LEFT_MOTOR_BW, clockFreq, period, 0, 1); 
    pwm_start(RIGHT_MOTOR_FW, clockFreq, period, 0, 1); 
    pwm_start(RIGHT_MOTOR_BW, clockFreq, period, 0, 1); 
    p_i_d = pid();
    numberOfTurns = 0;
    previousPidState = onTape;
    currentIrState = initialSpin;
    currentMajorState = upRamp;
    isThereCollision = OFF;
    previousMajorState = shutDown; //this cannot be initialized as upRamp because in the first run we need it to be different than currentMajorState for it to be communicated
    miniLoopDone = false;
    miniStateDone = false;
    // mood = false;
    currentCollisionState = firstTurn; 

    display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // OLED Display
    display.clearDisplay();
    display.setTextColor(WHITE);
    display.setFont(&FreeMono9pt7b);

    if (digitalRead(MODE_SWITCH)) {
      role = METHANOS; 
      decision = IRdecision(LEFT_IR, MID_IR, RIGHT_IR, 1); // default is 1kHz but we set this to the correct value in setup
      irStartThreshold = 400;
      closeThreshold = 1200; //past columns
    } else {
      role = THANOS;
      decision = IRdecision(LEFT_IR, MID_IR, RIGHT_IR, 10); // default is 1kHz but we set this to the correct value in setup
      irStartThreshold = 120;
      closeThreshold = 1200; //past columns   
    }
    majState = 0;

    if(digitalRead(MOOD_SWITCH) == ON){
      mood = true; //happy
    } else if(digitalRead(MOOD_SWITCH) == OFF){
      mood = false;
    }
    updateDisplay();
    initialTime = millis();
}
//1:happy

 void loop(){

  if (mood = true){ //HAPPY MODE
     if(miniStateDone == false){  //small loop
        timeElapsed = (millis() - initialTime)/1000;
        if(timeElapsed < RAMP_TIME)
          currentMajorState = upRamp;  
        else if(timeElapsed > SMALL_TURN_TIME_GM && timeElapsed < SMALL_TURN_TIME_GM+2){ //CHECK FOR THIS BEFORE TAPE FOLLOWING
          cutAcross(); //turn, drive straight, turn again, find tape
          majState  = (int)collectPlushie;
          if (role == METHANOS)
            majState |= 1UL << 4;  //set 5th bit to high if we are methanos
          else if (role == THANOS)
            majState &= ~(1UL << 4); //clears 5th bit if we are thanos
          Serial3.write(majState);
        } else if(timeElapsed < SMALL_COLLECT_TIME_GM)
            currentMajorState = collectPlushie;  //follow tape until we are in a position to look for IR
        else if(miniLoopDone == false)
            currentMajorState = depositPlushie; //deposit small loop plushies, with back-up and find tape
      }
      else if (miniStateDone == true){    //start the large loop
        timeElapsed = (millis() - initialTime)/1000; 
        if(timeElapsed < BIG_COLLECT_TIME)
          currentMajorState = collectPlushie; //start collecting again
        else
          currentMajorState = depositPlushie; //deposit big loop plushies, without back-up

        if (previousMajorState == shutDown)
          currentMajorState = shutDown;
      }
  } else if(mood = false){
    if(miniStateDone == false){  //small loop
        timeElapsed = (millis() - initialTime)/1000;
        if(timeElapsed < RAMP_TIME)
          currentMajorState = upRamp;  
        else if(timeElapsed > SMALL_TURN_TIME_BM && timeElapsed < SMALL_TURN_TIME_BM+2){ //CHECK FOR THIS BEFORE TAPE FOLLOWING
          cutAcross(); //turn, drive straight, turn again, find tape
          majState  = (int)collectPlushie;
          if (role == METHANOS)
            majState |= 1UL << 4;  //set 5th bit to high if we are methanos
          else if (role == THANOS)
            majState &= ~(1UL << 4); //clears 5th bit if we are thanos
          Serial3.write(majState);
        } else if(timeElapsed < SMALL_COLLECT_TIME_BM)
            currentMajorState = collectPlushie;  //follow tape until we are in a position to look for IR
        else if(miniLoopDone == false)
            currentMajorState = depositPlushie; //deposit small loop plushies, with back-up and find tape
    }
    else if (miniStateDone == true){    //start the large loop
      timeElapsed = (millis() - initialTime)/1000; 
      if(timeElapsed < BIG_COLLECT_TIME)
        currentMajorState = collectPlushie; //start collecting again
      else
        currentMajorState = depositPlushie; //deposit big loop plushies, without back-up

      if (previousMajorState == shutDown)
        currentMajorState = shutDown;
    }
  }   

  // COMMUNICATION
  if ((numberOfTurns > 0) && (currentMajorState == upRamp)) {
    majState  = (int)collectPlushie;
  }
  else 
    majState = (int)currentMajorState;

  if (role == METHANOS)
    majState |= 1UL << 4;  //set 5th bit to high if we are methanos
  else if (role == THANOS)
    majState &= ~(1UL << 4); //clears 5th bit if we are thanos
  Serial3.write(majState);

  farLeftVal = digitalRead(FAR_LEFT);
  stoneLeftVal = digitalRead(STONE_LEFT);
  leftMidVal = digitalRead(LEFT_MID);
  midMidVal = digitalRead(MID_MID);
  rightMidVal = digitalRead(RIGHT_MID);
  stoneRightVal = digitalRead(STONE_RIGHT);
  farRightVal = digitalRead(FAR_RIGHT); 

  switch ( currentMajorState ) { // state machine
    case upRamp :
      currentPidState = getPidState(farLeftVal, stoneLeftVal, leftMidVal, midMidVal, rightMidVal, stoneRightVal, farRightVal);
      pidStateMachine();
      break;

    case collectPlushie :
      //if ((miniStateDone == true) && (Serial3.available())) { //check if there's a collision in the big loop
      // if (miniStateDone == true) {
      // isThereCollision = Serial3.read();
      //   if (isThereCollision == 1) {
      //     collisionStateMachine();
      //     isThereCollision = 0;
      //     break;
      //   }
      // }
      currentPidState = getPidState(farLeftVal, stoneLeftVal, leftMidVal, midMidVal, rightMidVal, stoneRightVal, farRightVal);
      pidStateMachine();
      break;

    case depositPlushie :
      // if ((miniStateDone == true) && (Serial3.available())) {
      //   isThereCollision = Serial3.read();
      //   if (isThereCollision == 1) {
      //     collisionStateMachine();
      //     isThereCollision = OFF;
      //     break;
      //   }
      // }
 
      irStateMachine();
      break;

      case shutDown :
      drive(0,0,0,0);
      break;
  }
   previousMajorState = currentMajorState;
 }

//IR
/*void loop() {
    float number = decision.strongest_signal();
  if (number == LEFT_IR) 
    Serial.println("Left!!!");
  else if (number == MID_IR) 
    Serial.println("Middle!!!");
  else if (number == RIGHT_IR) 
    Serial.println("Right!!!");
  else 
    Serial.println("error finding max");

  float leftIntensity = decision.corrleft;
  midIntensity = decision.corrcenter;
  float rightIntensity = decision.corrright;

  Serial.print("Left: ");
  Serial.println(leftIntensity);
  Serial.print("Middle: ");
  Serial.println(midIntensity);
  Serial.print("Right: ");
  Serial.println(rightIntensity);
  Serial.print("Max correlation pin: ");
  delay(500);

  display.clearDisplay();
  display.setCursor(5, 20);
  display.print("Left: ");
  display.println(leftIntensity);
  display.setCursor(5, 40);
  display.print("Mid: ");
  display.println(midIntensity);
  display.setCursor(5, 60);
  display.print("Right: ");
  display.println(rightIntensity);
  display.display();
}*/

//QRDs
/*void loop() {
  farLeftVal = digitalRead(FAR_LEFT);
  stoneLeftVal = digitalRead(STONE_LEFT);
  leftMidVal = digitalRead(LEFT_MID);
  midMidVal = digitalRead(MID_MID);
  rightMidVal = digitalRead(RIGHT_MID);
  stoneRightVal = digitalRead(STONE_RIGHT);
  farRightVal = digitalRead(FAR_RIGHT);

  Serial.print(farLeftVal);
  Serial.print(" ");
  Serial.print(stoneLeftVal);
  Serial.print(" ");
  Serial.print(leftMidVal);
  Serial.print(midMidVal);
  Serial.print(rightMidVal);
  Serial.print(" ");
  Serial.print(stoneRightVal);
  Serial.print(" ");
  Serial.println(farRightVal);

  display.clearDisplay();
  display.setCursor(5, 20);
  display.print(farLeftVal);
  display.print(" ");
  display.print(stoneLeftVal);
  display.print(" ");
  display.print(leftMidVal);
  display.print(midMidVal);
  display.print(rightMidVal);
  display.print(" ");
  display.print(stoneRightVal);
  display.print(" ");
  display.println(farRightVal);
  display.setCursor(5, 40);
  currentPidState = getPidState(farLeftVal, stoneLeftVal, leftMidVal, midMidVal, rightMidVal, stoneRightVal, farRightVal);
  display.print(currentPidState);
  display.display();
  delay(200);
}*/

//Avoiding
/*void loop() {
    timeElapsed = (millis() - initialTime)/1000; // in seconds
  
    numberOfTurns = 5;
    currentMajorState = collectPlushie;

    if ((timeElapsed > 5) && (timeElapsed < 7)) {
      cutAcross();
    } else {
    farLeftVal = digitalRead(FAR_LEFT);
    stoneLeftVal = digitalRead(STONE_LEFT);
    leftMidVal = digitalRead(LEFT_MID);
    midMidVal = digitalRead(MID_MID);
    rightMidVal = digitalRead(RIGHT_MID);
    stoneRightVal = digitalRead(STONE_RIGHT);
    farRightVal = digitalRead(FAR_RIGHT);

    currentPidState = getPidState(farLeftVal, stoneLeftVal, leftMidVal, midMidVal, rightMidVal, stoneRightVal, farRightVal);
    pidStateMachine(); 
    }

   
}*/

//increase the first turn left delay
pidState getPidState(int farLeft, int stoneLeft, int leftMid, int midMid, int rightMid, int stoneRight, int farRight) {

  if (role == THANOS) {
    if ((farLeft == ON) && (currentMajorState == upRamp)) {
      numberOfTurns++;
      if (numberOfTurns == 1)  //first turn
        delay(400); //up ramp delay used to be 450
      return turnLeft;
    } else if ((currentMajorState == collectPlushie) && (farRight == ON)) {
        numberOfTurns++;
      return turnRight; 
    }
  } 
  else if (role == METHANOS) {
    if ((farRight == ON) && (currentMajorState == upRamp)) { 
      numberOfTurns++;
      if (numberOfTurns == 1)  //first turn
        delay(450); //up ramp delay, 400 at 60 pwm speed, 300 at 50 pwm
      return turnRight;
    }  else if ((currentMajorState == collectPlushie) && (farLeft == ON)) {
      numberOfTurns++;
      return turnLeft; 
    }
  }

  if ( (leftMid == OFF) && (midMid == ON) && (rightMid == OFF) )
    return onTape;
  else if ( (leftMid == OFF) && (midMid == ON) && (rightMid == ON) )
    return offOnOn;
  else if ( (leftMid == OFF) && (midMid == OFF) && (rightMid == ON) )
    return offOffOn;
   else if ( (leftMid == ON) && (midMid == ON) && (rightMid == OFF) )
    return onOnOff;
   else if ( (leftMid == ON) && (midMid == OFF) && (rightMid == OFF) )
    return onOffOff;         
  else
    return white;
     
}

//modular
void pidStateMachine() {
  switch ( currentPidState ) { //state machine
  
  case onTape : //drive straight
    error = 0; 
    leftSpeed = targetSpeed + p_i_d.output_pid(error);
    rightSpeed = targetSpeed + p_i_d.output_pid(error);
    drive(0, leftSpeed, 0, rightSpeed);
    break;

  case offOnOn : //turn right
    error = 2; //2
    leftSpeed = targetSpeed + p_i_d.output_pid(error);
    rightSpeed = targetSpeed + p_i_d.output_pid(-error);
    drive(0, leftSpeed, 0, rightSpeed); //left needs to catch up, right side weaker
    break;

  case onOnOff : //turn left
    error = 1; //2
    leftSpeed = targetSpeed + p_i_d.output_pid(-error);
    rightSpeed = targetSpeed + p_i_d.output_pid(error);
    drive(0, leftSpeed, 0, rightSpeed); //left weaker, right needs to catch up 
    break;

  case offOffOn : //turn right
    error = 3; 
    leftSpeed = targetSpeed + p_i_d.output_pid(error);
    rightSpeed = targetSpeed + p_i_d.output_pid(-error);
    drive(0, leftSpeed, 0, rightSpeed); //left needs to catch up, right side weaker
    break;

  case onOffOff : //turn left
    error = 3; 
    leftSpeed = targetSpeed + p_i_d.output_pid(-error);
    rightSpeed = targetSpeed + p_i_d.output_pid(error);
    drive(0, leftSpeed, 0, rightSpeed); //left weaker, right needs to catch up 
    break;

  case white : //both sensors off tape
    error = 6; //4
    if ((previousPidState == offOnOn) || (previousPidState == offOffOn)) { // continue to turn right
      leftSpeed = targetSpeed + p_i_d.output_pid(error);
      rightSpeed = targetSpeed + p_i_d.output_pid(-error);
      drive(0, leftSpeed, 0, rightSpeed); //left needs to catch up, right side weaker
    } else if ((previousPidState == onOnOff) || (previousPidState == onOffOff)) { // continue to turn left
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
    } 
    break;

  case turnLeft :
    if (numberOfTurns == 1)
    error = 5; //2
    else
    error = 8; //9
    leftSpeed = targetSpeed + p_i_d.output_pid(-error);
    rightSpeed = targetSpeed + p_i_d.output_pid(error);
    drive(0, leftSpeed, 0, rightSpeed); //left weaker, right needs to catch up 
    delay(300); 
    if (numberOfTurns == 1) 
    delay(700);
    break;

  case turnRight : 
    if (numberOfTurns == 1) 
    error = 5; //2
    else
    error = 8; //9 
    leftSpeed = targetSpeed + p_i_d.output_pid(error);
    rightSpeed = targetSpeed + p_i_d.output_pid(-error);
    drive(0, leftSpeed, 0, rightSpeed); //left needs to catch up, right side weaker
    delay(300); //thanos side is different
    if (numberOfTurns == 1) 
      delay(700);
    break;

  // case stoneOnRight : //not doing yet
  //   break; 

  // case stoneOnLeft : //not doing yet
  //   break; 

  default :
    drive(0, 0, 0, 0);
    delay(5000); // this should never happen...
    break; 

  }

  if (currentPidState != onTape && currentPidState != white)
    previousPidState = currentPidState;
}


void irStateMachine() {

  switch ( currentIrState ) { //state machine
    case initialSpin : //drive straight
      highestPin = decision.strongest_signal();  
      midIntensity = decision.corrcenter;

      if (role == THANOS)
        drive(0, spinSpeed, spinSpeed, 0); // spin CW
      else if (role == METHANOS)
        drive(spinSpeed, 0, 0, spinSpeed); // spin CCW 
      
      if ((highestPin == MID_IR) && (midIntensity > irStartThreshold)) {
        drive(0,0,0,0);
        currentIrState = drivingFar;
      }
      break;

    case drivingFar :
      // midIntensity = decision.corrcenter;
      // if (midIntensity > closeThreshold) {
      //   currentIrState = drivingClose;
      // } 
      // irDrive(currentIrState);
      drive(0, 5*targetIrSpeed/4, 0, 5*targetIrSpeed/4); //used to be targetIrSpeed
      delay(1500);
      currentIrState = drivingClose;
      break;  

    case drivingClose :
      midIntensity = decision.corrcenter;

      farLeftVal = digitalRead(FAR_LEFT);
      // leftMidVal = digitalRead(LEFT_MID);
      // midMidVal = digitalRead(MID_MID);
      // rightMidVal = digitalRead(RIGHT_MID);
      farRightVal = digitalRead(FAR_RIGHT);

      if ((farLeftVal == ON) || (farRightVal == ON)) {
        currentIrState = adjust;
        break;
      }
      // if ((farLeftVal == ON) || (leftMidVal  == ON) || (midMidVal  == ON) || (rightMidVal == ON) || (farRightVal == ON)) {
      //   currentIrState = adjust;
      //   break;
      // }
      irDrive(currentIrState);
      break;    

    case adjust :

      farLeftVal = digitalRead(FAR_LEFT);
      farRightVal = digitalRead(FAR_RIGHT);
      timeOut = millis();
      time = (millis() - timeOut)/1000;

      while (time < 1.5) {
        if ((farLeftVal == ON) && (farRightVal == ON)) {
          drive(0,0,0,0);
          currentIrState = stop;
          break;
        } else if (farRightVal == OFF){
          drive(0, 0, 0, targetIrSpeed);
        } else if (farLeftVal == OFF){
          drive(0, targetIrSpeed, 0, 0);
        }  
        farLeftVal = digitalRead(FAR_LEFT);
        farRightVal = digitalRead(FAR_RIGHT);
        time = (millis() - timeOut)/1000;
      }

      if (currentIrState != stop) { //drive forward to get off the tape
        currentIrState = drivingClose;
        drive(0, 5*targetIrSpeed/4, 0, 5*targetIrSpeed/4); //increased from targetIrSpeed
        delay(1000); 
      }

      // if ((farLeftVal == ON) && (farRightVal == ON)) {
      //   drive(0,0,0,0);
      //   currentIrState = stop;
      //   break;
      // } else if (farRightVal == OFF)
      //   drive(0, 0, 0, targetIrSpeed);
      // else if (farLeftVal == OFF)
      //   drive(0, targetIrSpeed, 0, 0);

      break;
    
    case stop :
      //drive(0,0,0,0);
      // if (bigLoop == true) {
      //   currentMajorState = shutDown;
      // }
      if(miniLoopDone == false) {
        miniLoopDone = true;
        //drive backwards until we find tape again
        if(role == THANOS){ //robot is picking the wrong path - this is to correct the direction
          drive(0, 0, spinSpeed, 0); // spin CW
          delay(1300);
        }
        drive(collisionSpeed, 0, collisionSpeed, 0);
        delay(1000);

        do { //drive backwards till you hit tape
          drive(4*collisionSpeed/5, 0, 4*collisionSpeed/5, 0); //used to be 50
          leftMidVal = digitalRead(LEFT_MID);
          midMidVal = digitalRead(MID_MID);
          rightMidVal = digitalRead(RIGHT_MID);
          if ((leftMidVal == ON) && (midMidVal == ON)) 
            rightMidVal = ON;
          else if ((rightMidVal == ON) && (midMidVal == ON)) 
            leftMidVal = ON;
        } while(!(leftMidVal == ON && midMidVal == ON && rightMidVal == ON));
        miniStateDone = true; //to go back to a new timer for the big loop
        currentIrState = initialSpin; //re-start IR mode
        majState = 0;
        if (mood == ON)
          majState |= 1UL << 6; //open the arms
        else if (mood == OFF)
          majState &= ~(1UL << 6); //close the arms
        Serial3.write(majState);
        majState = 0;
        initialTime = millis(); //reset the timer for the big loop
      } 
      else if (miniLoopDone == true) {
        drive(0,0,0,0);
        currentMajorState = shutDown;
      }
      break;
  }
}

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

void cutAcross() {
  float turnTime = 0;
  // if (role == THANOS) 
  //   turnTime = THANOS_TURN_TIME;
  // else if (role == METHANOS)
  //   turnTime = METHANOS_TURN_TIME;
  if((role == THANOS) && (mood == true)){
    turnTime = THANOS_TURN_TIME_GM;
  } else if((role == THANOS) && (mood == false)){
    turnTime = THANOS_TURN_TIME_BM;
  } else if((role == METHANOS) && (mood == true)){
    turnTime = METHANOS_TURN_TIME_GM;
  } else if((role == METHANOS) && (mood == false)){
    turnTime = METHANOS_TURN_TIME_BM;
  }

  numberOfTurns++;
  if (currentMajorState != depositPlushie) {
    switch( currentCollisionState ){
      case firstTurn :        
        collisionStartTime = millis(); 
        collisionTimeInterval = (millis() - collisionStartTime)/1000;

        while (collisionTimeInterval < turnTime) {
          if (role == THANOS)
            drive(0, collisionSpeed, collisionSpeed, 0); //drive cw
          else if (role == METHANOS)
            drive(collisionSpeed, 0, 0, collisionSpeed); //drive ccw

          collisionTimeInterval = (millis() - collisionStartTime)/1000;
        }
        currentCollisionState = driveStraight;
        break;
          
      case driveStraight :
        // drive(collisionSpeed, 0, collisionSpeed, 0);
        // delay(500);
        timeOut = millis();

        do {
          time = (millis() - timeOut)/1000;
          if ((time > 1.5) && (time < 1.55)) {
            majState = 0;
            majState |= 1UL << 5;  // = 32, 5th bit

            if (role == METHANOS) 
              majState |= 1UL << 4;  //set 5th bit to high if we are methanos
            else if (role == THANOS)
              majState &= ~(1UL << 4); //clears 5th bit if we are thanos
            Serial3.write(majState);
          }
          drive(0, collisionSpeed, 0, collisionSpeed);
          leftMidVal = digitalRead(LEFT_MID);
          midMidVal = digitalRead(MID_MID);
          rightMidVal = digitalRead(RIGHT_MID);
          if ((leftMidVal == ON) && (midMidVal == ON)) 
            rightMidVal = ON;
          else if ((rightMidVal == ON) && (midMidVal == ON)) 
            leftMidVal = ON;
        } while(!(leftMidVal == ON && midMidVal == ON && rightMidVal == ON));
        // leftMidVal = digitalRead(LEFT_MID);
        // midMidVal = digitalRead(MID_MID);
        // rightMidVal = digitalRead(RIGHT_MID);

        // while(!(leftMidVal == ON && midMidVal == ON && rightMidVal == ON)) {
        //   drive(0, collisionSpeed, 0, collisionSpeed);
        //   leftMidVal = digitalRead(LEFT_MID);
        //   midMidVal = digitalRead(MID_MID);
        //   rightMidVal = digitalRead(RIGHT_MID);
        //   if ((leftMidVal == ON) && (midMidVal == ON)) 
        //     rightMidVal = ON;
        //   else if ((rightMidVal == ON) && (midMidVal == ON)) 
        //     leftMidVal = ON;
        // }
        currentCollisionState = lastTurn;
        break;

      case lastTurn :
        // if (role == THANOS) 
        //   currentPidState = turnRight;
        // else if (role == METHANOS) 
        //   currentPidState = turnLeft;
        // pidStateMachine();
        do {
          if (role == THANOS)
            drive(0,  25*period/100, 25*period/100, 0); //drive cw, used to be 50, then 30..
          else if (role == METHANOS)
            drive(25*period/100, 0, 0, 25*period/100); //drive ccw           
          midMidVal = digitalRead(MID_MID);
        } while(!( midMidVal == ON));

        currentCollisionState = firstTurn; 
        currentPidState = onTape;
        if(role == THANOS){   // might want to go back to regular pid  ? ? ?
          previousPidState = turnLeft; //to go back to the tape (it passes the tape)
        } else if(role == METHANOS){
          previousPidState = turnRight;
        }
        pidStateMachine();
        break;     
    }
  }
  else {
    drive(0,0,0,0);
    delay(3000);
  }
 
}

void collisionStateMachine() {
  float turnTime = 0.6;

  if (currentMajorState != depositPlushie) {
    switch( currentCollisionState ){
      case firstTurn :        
        collisionStartTime = millis(); 
        collisionTimeInterval = (millis() - collisionStartTime)/1000;

        while (collisionTimeInterval < turnTime) {
          if (role == THANOS)
            drive(0, collisionSpeed, collisionSpeed, 0); //drive cw
          else if (role == METHANOS)
            drive(collisionSpeed, 0, 0, collisionSpeed); //drive ccw

          collisionTimeInterval = (millis() - collisionStartTime)/1000;
        }
        currentCollisionState = driveStraight;
        break;
          
      case driveStraight :
        // drive(collisionSpeed, 0, collisionSpeed, 0);
        // delay(500);
        timeOut = millis();

        do {
          time = (millis() - timeOut)/1000;
          if ((time > 1.5) && (time < 1.55)) {
            majState = 0;
            majState |= 1UL << 5;  // = 32, 5th bit

            if (role == METHANOS) 
              majState |= 1UL << 4;  //set 5th bit to high if we are methanos
            else if (role == THANOS)
              majState &= ~(1UL << 4); //clears 5th bit if we are thanos
            Serial3.write(majState);
          }
          drive(0, collisionSpeed, 0, collisionSpeed);
          leftMidVal = digitalRead(LEFT_MID);
          midMidVal = digitalRead(MID_MID);
          rightMidVal = digitalRead(RIGHT_MID);
          if ((leftMidVal == ON) && (midMidVal == ON)) 
            rightMidVal = ON;
          else if ((rightMidVal == ON) && (midMidVal == ON)) 
            leftMidVal = ON;
        } while(!(leftMidVal == ON && midMidVal == ON && rightMidVal == ON));
   
        currentCollisionState = lastTurn;
        break;

      case lastTurn :
        do {
          if (role == THANOS)
            drive(0,  25*period/100, 25*period/100, 0); //drive cw, used to be 50, then 30..
          else if (role == METHANOS)
            drive(25*period/100, 0, 0, 25*period/100); //drive ccw           
          midMidVal = digitalRead(MID_MID);
        } while(!( midMidVal == ON));

        currentCollisionState = firstTurn; 
        currentPidState = onTape;
        if(role == THANOS){   // might want to go back to regular pid  ? ? ?
          previousPidState = turnLeft; //to go back to the tape (it passes the tape)
        } else if(role == METHANOS){
          previousPidState = turnRight;
        }
        pidStateMachine();
        break;     
    }
  }
  else {
    if (role == THANOS)
      drive(collisionSpeed, 0, 0, collisionSpeed); //turn left
    else if (role == METHANOS)
      drive(0, collisionSpeed, collisionSpeed, 0); //turn right
    delay(300);

    drive(0, targetIrSpeed, 0, targetIrSpeed);
    delay(750);
    currentIrState = initialSpin;
  }
 
}

//modular
void irDrive(irState currentIrState) {
  float speed, speedPlus, speedMinus;

  if (currentIrState == drivingFar) {
    speed = targetIrSpeed; // turn sharper if far away
    speedPlus = targetIrSpeedPlus;
    speedMinus = targetIrSpeedMinus;
  } else if (currentIrState == drivingClose) {
    speed = targetIrSpeed; //used to be 4/5
    speedPlus = targetIrSpeedPlus;
    speedMinus = targetIrSpeedMinus;
  } else {
    speed = 0;
    speedPlus = 0;
    speedMinus = 0;
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

void updatePotVal(){
  p_i_d.kp = 80+100*analogRead(KP_METER)/1023;
  p_i_d.kd = 160+220*analogRead(KD_METER)/1023;
 }

 void updateDisplay() {
    //updatePotVal();
    display.clearDisplay();
    display.setCursor(5, 20);
    display.print("kp: ");
    display.println(p_i_d.kp);
    display.setCursor(5, 40);
    display.print("kd: ");
    display.println(p_i_d.kd);
    display.setCursor(5, 60);
    if (role == THANOS)
      display.println("Thanos D:");
    else if (role == METHANOS)
      display.println("Methanos :D");
    display.display();
 }