// right wheel tends to be faster when speeds are set to be equal
// METHANOS = 1 kHz, turn left twice, then right
// THANOS = 10 kHz, turn right twice, then left

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
#define LEFT_MID PB14
#define MID_MID PB15
#define RIGHT_MID PA15
#define STONE_RIGHT PA12
#define FAR_RIGHT PA11

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

#define KP_METER PA5
#define KD_METER PA4
#define KP_KD_BUTTON PB8
//first switch b4, SWIRCH 2 = b5
#define MODE_SWITCH PB5

#define RAMP_TIME 15
#define COLLECT_TIME 29

// #define COM_PLUSH_COLLECT_TIME 8
// #define COM_PLUSH_DEPOSIT_TIME 26
// #define COM_STONES 40

float clockFreq = 100000;
float period = 1000;

float initialTime;
float timeElapsed;

float spinSpeed = 22*period/100; //make sure this isnt too fast, 20 is too slow
float targetIrSpeed = 22*period/100;  //25
float targetIrSpeedPlus = 27*period/100;  //30
float targetIrSpeedMinus = 17*period/100;  //20

float targetSpeed = 60*period/100;
float leftSpeed = targetSpeed;
float rightSpeed = targetSpeed;

float irStartThreshold = 900;
float midThreshold = 1000; //past columns
float closeThreshold = 1550;  //quite close

float midIntensity = -1;
int highestPin = -1;

int farLeftVal = 0;
int stoneLeftVal = 0;
int leftMidVal = 0;
int midMidVal = 0;
int rightMidVal = 0;
int stoneRightVal = 0;
int farRightVal = 0;

bool stonePart;
// bool irDefined;

float error = 0.0;
int numberOfTurns;

enum majorState { upRamp, collectPlushie, depositPlushie, stones, shutDown } ;
enum pidState { onTape, offOnOn, offOffOn, onOnOff, onOffOff, white, turnLeft, turnRight, stoneOnRight, stoneOnLeft } ;
enum irState { initialSpin, drivingFar, drivingClose, /*avoid,*/ stop} ;

majorState currentMajorState, previousMajorState;
irState currentIrState;
pidState  currentPidState, previousPidState;

pid p_i_d;

int role;
#define THANOS 0
#define METHANOS 1

IRdecision decision = IRdecision(LEFT_IR, MID_IR, RIGHT_IR, 1); // default is 1kHz but we set this to the correct value in setup

float speedCapOff(float speed); //add for all functions
pidState getPidState(int farLeft, int stoneLeft, int leftMid, int midMid, int rightMid, int stoneRight, int farRight);
void drive(float bwLeft, float fwLeft, float bwRight, float fwRight);
void pidStateMachine();
void irStateMachine();
void irDrive(irState currentIrState);
void updatePotVal();
void updateDisplay();

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

    pinMode(KP_KD_BUTTON, INPUT_PULLUP);
    // attachInterrupt(digitalPinToInterrupt(KP_KD_BUTTON), updatePotVal, CHANGE);
        
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
    previousMajorState = shutDown; //this cannot be initialized as upRamp because in the first run we need it to be different than currentMajorState for it to be communicated
    // irDefined = false;
    stonePart = false;

    display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // OLED Display
    display.clearDisplay();
    display.setTextColor(WHITE);
    display.setFont(&FreeMono9pt7b);

    if (digitalRead(MODE_SWITCH)) {
      role = METHANOS; 
      Serial3.write("m");
      decision.setMode(1);
    } else {
      role = THANOS;
      Serial3.write("t");
      decision.setMode(10);
    }

    updateDisplay();
    initialTime = millis();
}

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
  delay(1000);
}*/

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

  delay(200);
}*/


 void loop() {  // SLAVE
  timeElapsed = (millis() - initialTime)/1000; // in seconds

  if (timeElapsed < RAMP_TIME)
    currentMajorState = upRamp;
  else if (timeElapsed < COLLECT_TIME)
    currentMajorState = collectPlushie;
  else if (stonePart == true)
    currentMajorState = stones;
  else 
    currentMajorState = depositPlushie;
  
  if ((numberOfTurns > 0) && (currentMajorState == upRamp)){
    //Serial.println(collectPlushie);
    Serial3.write(collectPlushie);
  } else {
    //Serial.println((int)currentMajorState);
    Serial3.write((int)currentMajorState);
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
      currentPidState = getPidState(farLeftVal, stoneLeftVal, leftMidVal, midMidVal, rightMidVal, stoneRightVal, farRightVal);
      pidStateMachine();
      break;

    case depositPlushie:
      irStateMachine();
      break;

    case stones:
      drive(0, 0, 0, 0);
      break;
  }
  previousMajorState = currentMajorState;
 }
 

//increase the first turn left delay
pidState getPidState(int farLeft, int stoneLeft, int leftMid, int midMid, int rightMid, int stoneRight, int farRight) {

  if (role == THANOS) {
    if (farLeft == ON) {
      if (currentMajorState == upRamp) { //if its 0 or 1
        numberOfTurns++;
        if (numberOfTurns == 1)  //first turn
          delay(450); //up ramp delay
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

//enum pidState { onTape, offOnOn, offOffOn, onOnOff, onOffOff, white, turnLeft, turnRight, stoneOnRight, stoneOnLeft } ;

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
    error = 2; 
    leftSpeed = targetSpeed + p_i_d.output_pid(error);
    rightSpeed = targetSpeed + p_i_d.output_pid(-error);
    // Serial.println(error);
    // Serial.println(leftSpeed);
    // Serial.println(rightSpeed);
    // Serial.println();
    drive(0, leftSpeed, 0, rightSpeed); //left needs to catch up, right side weaker
    break;

  case onOnOff : //turn left
    error = 2; 
    leftSpeed = targetSpeed + p_i_d.output_pid(-error);
    rightSpeed = targetSpeed + p_i_d.output_pid(error);
    // Serial.println(error);
    // Serial.println(leftSpeed);
    // Serial.println(rightSpeed);
    // Serial.println();
    drive(0, leftSpeed, 0, rightSpeed); //left weaker, right needs to catch up 
    break;

  case offOffOn : //turn right
    error = 4; 
    leftSpeed = targetSpeed + p_i_d.output_pid(error);
    rightSpeed = targetSpeed + p_i_d.output_pid(-error);
    // Serial.println(error);
    // Serial.println(leftSpeed);
    // Serial.println(rightSpeed);
    // Serial.println();
    drive(0, leftSpeed, 0, rightSpeed); //left needs to catch up, right side weaker
    break;

  case onOffOff : //turn left
    error = 4; 
    leftSpeed = targetSpeed + p_i_d.output_pid(-error);
    rightSpeed = targetSpeed + p_i_d.output_pid(error);
    // Serial.println(error);
    // Serial.println(leftSpeed);
    // Serial.println(rightSpeed);
    // Serial.println();
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
    // Serial.println(error);
    // Serial.println(leftSpeed);
    // Serial.println(rightSpeed);
    // Serial.println();
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
    // Serial.println(error);
    // Serial.println(leftSpeed);
    // Serial.println(rightSpeed);
    // Serial.println();
    drive(0, leftSpeed, 0, rightSpeed); //left needs to catch up, right side weaker
    delay(300); //thanos side is different
    if (numberOfTurns == 1) 
      delay(700);
    break;

  case stoneOnRight : //not doing yet
    drive(0,0,0,0);
    delay(5000);
    break; 

  case stoneOnLeft : //not doing yet
    drive(0,0,0,0);
    delay(5000);
    break; 

  default :
    drive(0, 0, 0, 0); //spin for 5 seconds
    delay(5000); // this should never happen...
    break; 

  }

  if (currentPidState != onTape && currentPidState != white)
    previousPidState = currentPidState;
}


void irStateMachine() {

  /*float number = decision.strongest_signal();
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
  delay(1000);*/
  switch ( currentIrState ) { //state machine
    case initialSpin : //drive straight
      highestPin = decision.strongest_signal();  
      midIntensity = decision.corrcenter;
      // drive(0,0,0,0);
      // delay(3000); //help door stop without forces
      if (role == THANOS)
        drive(0, spinSpeed, spinSpeed, 0); // spin CW
      else if (role == METHANOS)
        drive(spinSpeed, 0, 0, spinSpeed); // spin CCW 
      
      if ((highestPin == MID_IR) && (midIntensity > irStartThreshold)) {
        drive(0,0,0,0);
        currentIrState = drivingFar;
      }
      //displayIR(); 
      break;

    case drivingFar :
      midIntensity = decision.corrcenter;
      if (midIntensity > closeThreshold) {
        currentIrState = drivingClose;
      } 
      irDrive(currentIrState);
      // displayIR();
      break;  

    case drivingClose :
      midIntensity = decision.corrcenter;

      farLeftVal = digitalRead(FAR_LEFT);
      stoneLeftVal = digitalRead(STONE_LEFT);
      leftMidVal = digitalRead(LEFT_MID);
      midMidVal = digitalRead(MID_MID);
      rightMidVal = digitalRead(RIGHT_MID);
      stoneRightVal = digitalRead(STONE_RIGHT);
      farRightVal = digitalRead(FAR_RIGHT);

      if ((farLeftVal == ON) || (leftMidVal  == ON) || (midMidVal  == ON) || (rightMidVal == ON) || (farRightVal == ON)) {
        currentIrState = stop;
        break;
      }
      irDrive(currentIrState);
      // displayIR();
      break;    
    
    case stop :
      drive(0,0,0,0);
      stonePart = true;
      delay(500);
      // currentMajorState = stones;
      // displayIR();
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
    updatePotVal();
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


// void displayIR(){
//   display.clearDisplay();
//   display.setCursor(0,20);
//   display.print("L");
//   display.println(decision.corrleft); //left
//   display.setCursor(30,50);
//   display.print("M");
//   display.println(decision.corrcenter); //mid
//   display.setCursor(63,20);
//   display.print("R");
//   display.println(decision.corrright); //right
//   display.display();
// }
