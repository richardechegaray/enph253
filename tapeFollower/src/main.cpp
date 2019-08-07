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
#define MOOD_SWITCH PB4

//#define RAMP_TIME 15
// #define COLLECT_TIME 29
// #define SMALL_LOOP_TIME 17
// #define SMALL_COLLECT_TIME 24
// #define BIG_COLLECT_TIME 8
#define RAMP_TIME 15
#define SMALL_TURN_TIME 17.5
#define SMALL_COLLECT_TIME 24
#define BIG_COLLECT_TIME 14

#define QUARTER_TURN_TIME 0.7

float clockFreq = 100000;
float period = 1000;

float initialTime;
float timeElapsed;
float collisionStartTime;   
float collisionTimeInterval;

int isThereCollision;

float spinSpeed = 22*period/100; //make sure this isnt too fast, 20 is too slow
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
// bool bigLoop;
bool miniStateDone;

float timeOut;
float time;

float error = 0.0;
int numberOfTurns;
uint8_t majState;

enum majorState { upRamp, collectPlushie, depositPlushie, stones, shutDown } ;
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

    // pinMode(KP_KD_BUTTON, INPUT_PULLUP);
        
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
    // bigLoop = false;
    miniStateDone = false;
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
    updateDisplay();
    initialTime = millis();
}

 void loop(){
  if(miniStateDone == false){
    timeElapsed = (millis() - initialTime)/1000;
    if(timeElapsed < RAMP_TIME){
      currentMajorState = upRamp;  
    } else if(timeElapsed > SMALL_TURN_TIME && timeElapsed < SMALL_TURN_TIME+2){ //CHECK FOR THIS BEFORE TAPE FOLLOWING
      collisionStateMachine(); //turn, drive straight, turn again, find tape
    } else if(timeElapsed < SMALL_COLLECT_TIME){
      currentMajorState = collectPlushie;  //follow tape until we are in a position to look for IR
    } else if(miniLoopDone == false){
      currentMajorState = depositPlushie; //deposit small loop plushies, with back-up and find tape
    }
  } else{
    //start the large loop
    timeElapsed = (millis() - initialTime)/1000; 
    if(timeElapsed < BIG_COLLECT_TIME){
      currentMajorState = collectPlushie; //start collecting again
    } else{
      currentMajorState = depositPlushie; //deposit big loop plushies, without back-up
    } 
  }

  farLeftVal = digitalRead(FAR_LEFT);
  stoneLeftVal = digitalRead(STONE_LEFT);
  leftMidVal = digitalRead(LEFT_MID);
  midMidVal = digitalRead(MID_MID);
  rightMidVal = digitalRead(RIGHT_MID);
  stoneRightVal = digitalRead(STONE_RIGHT);
  farRightVal = digitalRead(FAR_RIGHT); 

  switch ( currentMajorState ) { // state machine
    case upRamp:
      currentPidState = getPidState(farLeftVal, stoneLeftVal, leftMidVal, midMidVal, rightMidVal, stoneRightVal, farRightVal);
      pidStateMachine();
      break;

    case collectPlushie:
      // isThereCollision = Serial3.read();
      // if (isThereCollision == 1) {
      //   collisionStateMachine();
      //   isThereCollision = OFF;
      //   break;
      // }
      currentPidState = getPidState(farLeftVal, stoneLeftVal, leftMidVal, midMidVal, rightMidVal, stoneRightVal, farRightVal);
      pidStateMachine();
      break;

    case depositPlushie:
      // isThereCollision = Serial3.read();
      // if (isThereCollision == 1) {
      //   collisionStateMachine();
      //   isThereCollision = OFF;
      //   break;
      // }
      irStateMachine();
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
      collisionStateMachine();
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

/*void loop() {  // SLAVE
  timeElapsed = (millis() - initialTime)/1000; // in seconds

  if (timeElapsed < RAMP_TIME)
    currentMajorState = upRamp;
  else if (timeElapsed < COLLECT_TIME)
    currentMajorState = collectPlushie;
  else if (stonePart == true)
    currentMajorState = stones;
  else 
    currentMajorState = depositPlushie;

  //if ((timeElapsed > SMALL_LOOP_TIME) && (timeElapsed < SMALL_LOOP_TIME+2))
  //   collisionStateMachine();

  if ((numberOfTurns > 0) && (currentMajorState == upRamp))
    majState  = (int)collectPlushie;
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
    case upRamp:
      currentPidState = getPidState(farLeftVal, stoneLeftVal, leftMidVal, midMidVal, rightMidVal, stoneRightVal, farRightVal);
      pidStateMachine();
      break;

    case collectPlushie:
      isThereCollision = Serial3.read();
      if (isThereCollision == 1) {
        collisionStateMachine();
        isThereCollision = OFF;
        break;
      }
      currentPidState = getPidState(farLeftVal, stoneLeftVal, leftMidVal, midMidVal, rightMidVal, stoneRightVal, farRightVal);
      pidStateMachine();
      break;

    case depositPlushie:
      isThereCollision = Serial3.read();
      if (isThereCollision == 1) {
        collisionStateMachine();
        isThereCollision = OFF;
        break;
      }
      irStateMachine();
      break;

    case stones:
      drive(0, 0, 0, 0);
      break;
  }
   previousMajorState = currentMajorState;
 }*/  

//increase the first turn left delay
pidState getPidState(int farLeft, int stoneLeft, int leftMid, int midMid, int rightMid, int stoneRight, int farRight) {

  if (role == THANOS) {
    if (farLeft == ON) {
      if (currentMajorState == upRamp) { //if its 0 or 1
        numberOfTurns++;
        if (numberOfTurns == 1)  //first turn
          delay(400); //up ramp delay used to be 450
        return turnLeft;
      }
    }

    if ( currentMajorState == collectPlushie ) {
      if (farRight == ON) {
        numberOfTurns++;
        return turnRight; 
      }
    }
  } else if (role == METHANOS) {
    if (farRight == ON) {
      if (currentMajorState == upRamp) { //if its 0 or 1 //got rid of number of turns <2
        numberOfTurns++;
        if (numberOfTurns == 1)  //first turn
          delay(450); //up ramp delay, 400 at 60 pwm speed, 300 at 50 pwm
        return turnRight;
      }
    }

    if ( currentMajorState == collectPlushie ) {
      if (farLeft == ON) {
        numberOfTurns++;
        return turnLeft; 
      }
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
      drive(0, targetIrSpeed, 0, targetIrSpeed);
      delay(3000);
      currentIrState = drivingClose;
      break;  

    case drivingClose :
      midIntensity = decision.corrcenter;

      farLeftVal = digitalRead(FAR_LEFT);
      leftMidVal = digitalRead(LEFT_MID);
      midMidVal = digitalRead(MID_MID);
      rightMidVal = digitalRead(RIGHT_MID);
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

      while (time < 1) {
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

      if (currentIrState != stop) {
        currentIrState = drivingClose;
        drive(0, targetIrSpeed, 0, targetIrSpeed);
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
      if(miniLoopDone == false){
        //drive backwards until we find tape again
        drive(collisionSpeed, 0, collisionSpeed, 0);
        delay(1000);
        do {
          drive(collisionSpeed, 0, collisionSpeed, 0);
          leftMidVal = digitalRead(LEFT_MID);
          midMidVal = digitalRead(MID_MID);
          rightMidVal = digitalRead(RIGHT_MID);
          if ((leftMidVal == ON) && (midMidVal == ON)) 
            rightMidVal = ON;
          else if ((rightMidVal == ON) && (midMidVal == ON)) 
            leftMidVal = ON;
        } while(!(leftMidVal == ON && midMidVal == ON && rightMidVal == ON));
        miniLoopDone = true;
        miniStateDone = true; //to go back to a new timer for the big loop
        currentIrState = initialSpin; //re-start IR mode
        initialTime = millis(); //reset the timer for the big loop
      } 
      else{
        drive(0,0,0,0);
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

void collisionStateMachine() {
  numberOfTurns++;
  if (currentMajorState != depositPlushie) {
    switch( currentCollisionState ){
      case firstTurn:
        collisionStartTime = millis(); 
        collisionTimeInterval = (millis() - collisionStartTime)/1000;
        while (collisionTimeInterval < QUARTER_TURN_TIME) {
          if (role == THANOS)
            drive(0, collisionSpeed, collisionSpeed, 0); //drive cw
          else if (role == METHANOS)
            drive(collisionSpeed, 0, 0, collisionSpeed); //drive ccw

          collisionTimeInterval = (millis() - collisionStartTime)/1000;
        }
        currentCollisionState = driveStraight;
        break;
          
      case driveStraight:
        // drive(collisionSpeed, 0, collisionSpeed, 0);
        // delay(500);
        do {
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

      case lastTurn:
        // if (role == THANOS) 
        //   currentPidState = turnRight;
        // else if (role == METHANOS) 
        //   currentPidState = turnLeft;
        // pidStateMachine();
        do {
          if (role == THANOS)
            drive(0, collisionSpeed, collisionSpeed, 0); //drive cw
          else if (role == METHANOS)
            drive(collisionSpeed, 0, 0, collisionSpeed); //drive ccw           
          midMidVal = digitalRead(MID_MID);
        } while(!( midMidVal == ON));

        currentCollisionState = firstTurn; 
        currentPidState = onTape;
        if(role == THANOS){
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

//modular
void irDrive(irState currentIrState) {
  float speed, speedPlus, speedMinus;

  if (currentIrState == drivingFar) {
    speed = targetIrSpeed; // turn sharper if far away
    speedPlus = targetIrSpeedPlus;
    speedMinus = targetIrSpeedMinus;
  } else if (currentIrState == drivingClose) {
    speed = 4*targetIrSpeed/5;
    speedPlus = 4*targetIrSpeedPlus/5;
    speedMinus = 4*targetIrSpeedMinus/5;
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