// right wheel tends to be faster when speeds are set to be equal
// METHANOS = 1 kHz, turn left twice, then right
// THANOS = 10 kHz, turn right twice, then left

#include <Arduino.h>
#include <pid.h>
#include "IRdecision.h" 
// #include <Wire.h>
// #include <Adafruit_SSD1306.h>
// #include <FreeMono9pt7b.h>
// #define OLED_RESET -1 //for reset button
// Adafruit_SSD1306 display(OLED_RESET);

#define FAR_LEFT PB12
#define LEFT_SENSOR PB13  
#define RIGHT_SENSOR PB14
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

#define RAMP_TIME 15
#define COLLECT_TIME 27

// #define COM_PLUSH_COLLECT_TIME 8
// #define COM_PLUSH_DEPOSIT_TIME 26
// #define COM_STONES 40

// #define KP_POTMETER PA0
// #define KD_POTMETER PA1

float clockFreq = 100000;
float period = 1000;

float initialTime;
float timeElapsed;

float spinSpeed = 22*period/100; //make sure this isnt too fast, 20 is too slow
float targetIrSpeed = 25*period/100;
float targetIrSpeedPlus = 30*period/100;
float targetIrSpeedMinus = 20*period/100;

float targetSpeed = 60*period/100;
float leftSpeed = targetSpeed;
float rightSpeed = targetSpeed;

float irStartThreshold = 900;
float midThreshold = 1000; //past columns
float closeThreshold = 1550;  //quite close

float midIntensity = -1;
int highestPin = -1;

int leftValue = 0;
int rightValue = 0;
int farLeftValue = 0;
int farRightValue = 0;
bool stonePart;
bool irDefined;

float error = 0.0;
int numberOfTurns;

enum majorState { upRamp, collectPlushie, depositPlushie, stones, shutDown } ;
majorState currentMajorState, previousMajorState;
enum pidState { onTrack, leftOff, rightOff, turnLeft, turnRight, white, malfunc} currentPidState, previousPidState;
enum irState { initialSpin, drivingFar, drivingClose, /*avoid,*/ stop} currentIrState;

pid p_i_d;
// float kp_reading;
// float kd_reading;

#define THANOS 0
#define METHANOS 1
#define ROLE METHANOS

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
void irDrive(irState currentIrState);
// void displayPID();
// void displayRefl();
// void displayIR();

void setup() {
    Serial.begin(115200);
    Serial3.begin(9600);

    pinMode(LEFT_SENSOR, INPUT_PULLUP); 
    pinMode(RIGHT_SENSOR, INPUT_PULLUP); 
    pinMode(FAR_LEFT, INPUT_PULLUP);
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
    previousPidState = onTrack;
    currentIrState = initialSpin;
    currentMajorState = upRamp;
    previousMajorState = shutDown; //this cannot be initialized as upRamp because in the first run we need it to be different than currentMajorState for it to be communicated
    irDefined = false;
    stonePart = false;
    // kd_reading = p_i_d.kd; // Potentiometers
    // kp_reading = p_i_d.kp;

    // display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // OLED Display
    // display.clearDisplay();
    // display.setTextColor(WHITE);
    // display.setFont(&FreeMono9pt7b);

    initialTime = millis();
}

// void loop() {
//   // leftValue = digitalRead(LEFT_SENSOR);
//   // rightValue = digitalRead(RIGHT_SENSOR);
//   // farLeftValue = digitalRead(FAR_LEFT);
//   // farRightValue = digitalRead(FAR_RIGHT);
//   int leftMost = digitalRead(PA11);
//   int mid = digitalRead(PA12);
//   int rightMost = digitalRead(PA15);

//   // Serial.print((int)farLeftValue);
//   // Serial.print((int)leftValue);
//   // Serial.print((int)rightValue);
//   // Serial.println((int)farRightValue);

//   Serial.print(leftMost);
//   Serial.print(mid);
//   Serial.println(rightMost);

//   delay(750);

// }

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

  //  if (timeElapsed < 3)//RAMP_TIME)
  //   //data = 0;
  //    currentMajorState = upRamp;
  // else if (timeElapsed < 6)//COLLECT_TIME)
  //   //  data = 1;
  //   currentMajorState = collectPlushie;//collectPlushie;
  // else if (timeElapsed < 9)
  //   //  data = 2;
  //   currentMajorState = depositPlushie;
  // else 
  //   //  data = 3;
  //  currentMajorState = stones;
  
  if ((numberOfTurns > 0) && (currentMajorState == upRamp)){
    Serial.println(collectPlushie);
    Serial3.write(collectPlushie);
  } else {
    Serial.println((int)currentMajorState);
    Serial3.write((int)currentMajorState);
  }

  // if (numberOfTurns == 1){                 //NEWEST ONE
  //   Serial3.write((int)collectPlushie);
  // } else if(currentMajorState != previousMajorState){
  //   if (currentMajorState == depositPlushie) {
  //     // while (!(Serial3.available())) {
  //     //   drive(0,0,0,0); //before spin
  //     //   Serial3.flush();
  //     // }
  //     drive(0,0,0,0);
  //     Serial3.flush(); //might have to comment
  //     Serial3.write((int)depositPlushie); 
  //   } 
  //   Serial3.write((int)currentMajorState); 
  // }

  // if (numberOfTurns == 1) {
  //   Serial3.write((int)collectPlushie);
  // } else if (currentMajorState != previousMajorState) {
  //   if (currentMajorState == depositPlushie) {
  //     drive(0,0,0,0);
  //     delay(2000);
  //   }
  //   Serial3.flush();
  //   Serial3.write((int)currentMajorState);
  // }

  // if (timeElapsed < COM_PLUSH_COLLECT_TIME)
  //   Serial3.write((int)upRamp); 
  // else if (timeElapsed < COM_PLUSH_DEPOSIT_TIME)
  //   Serial3.write((int)collectPlushie); 
  // else if (timeElapsed < COM_STONES)
  //   Serial3.write((int)depositPlushie); 
  // else 
  //   Serial3.write((int)stones); 
  

  /*if (numberOfTurns == 1){     //magic
    Serial3.write((int)collectPlushie);
  } else if(currentMajorState == depositPlushie){
    if (butterfly == false) {
      butterfly = true;
      drive(0,0,0,0);
    }
    Serial3.write((int)currentMajorState); 
  } else {
      Serial3.write((int)currentMajorState); 
  }*/

  leftValue = digitalRead(LEFT_SENSOR);
  rightValue = digitalRead(RIGHT_SENSOR);
  farLeftValue = digitalRead(FAR_LEFT);
  farRightValue = digitalRead(FAR_RIGHT);

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
      drive(0, 0, 0, 0);
      break;
  }
  previousMajorState = currentMajorState;
}
 

//increase the first turn left delay
pidState getPidState(float left, float right, float farLeft, float farRight){

  #if (ROLE == THANOS) 
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
  #elif (ROLE == METHANOS) 
    if (farRight == ON) {
      if (currentMajorState == upRamp) { //if its 0 or 1 //got rid of number of turns <2
        numberOfTurns++;
        if (numberOfTurns == 1)  //first turn
          delay(450); //up ramp delay,  400 at 60 pwm speed, 300 at 50 pwm
        return turnRight;
      }
    }

    if ( currentMajorState == collectPlushie ) {
      if (farLeft == ON) {
        numberOfTurns++;
        return turnLeft; 
      }
    }
  #endif

  if ( (left == ON) && (right == ON) )
    return onTrack;
  else if ( (right == ON) && (left == OFF) )
    return leftOff;
  else if ( (right == OFF) && (left == ON) )
    return rightOff;
  else
    return white;
     
}

//modular
void pidStateMachine() {

  //update kp and kd values if we modified with the potentiometer
  // kp_reading = analogRead(KP_POTMETER);
  // if(kp_reading != p_i_d.kp){
  //   p_i_d.kp = map(kp_reading, 0, 1023, 0, 500); 
  // }
  // kd_reading = analogRead(KD_POTMETER);
  // if(kd_reading != p_i_d.kd){
  //   p_i_d.kd = map(kd_reading, 0, 1023, 0, 500);
  // }
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
      } 
      // Serial.println(error);
      // Serial.println(leftSpeed);
      // Serial.println(rightSpeed);
      // Serial.println();
      break;
    
    case turnLeft :
      if (numberOfTurns == 1)
        error = 2.5;  //2
      else
        error = 4; //9
 
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
        error = 2.5;  //2
      else
        error = 4;  //9      
      
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

  // displayPID();
  // displayRefl(); //these display 'global values'
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
      #if (ROLE == THANOS)
        drive(0, spinSpeed, spinSpeed, 0); // spin CW
      #elif (ROLE == METHANOS)
        drive(spinSpeed, 0, 0, spinSpeed); // spin CCW 
      #endif
      
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
      leftValue = digitalRead(LEFT_SENSOR);
      rightValue = digitalRead(RIGHT_SENSOR);
      farLeftValue = digitalRead(FAR_LEFT);
      farRightValue = digitalRead(FAR_RIGHT);

      if ((leftValue == ON) || (rightValue  == ON) || (farLeftValue  == ON) || (farRightValue == ON)) {
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

// void displayPID(){
//   display.clearDisplay();
//   display.setCursor(5,20);
//   display.print("kp:");
//   display.println(p_i_d.kp);
//   display.setCursor(5,40);
//   display.print("kd:");
//   display.println(p_i_d.kd);
//   display.display();
// }
// void displayRefl(){
//   display.clearDisplay();
//   display.setCursor(3,20);
//   display.print("FL:");
//   display.println(farLeftValue); //farleft
//   display.setCursor(65,20);
//   display.print("FR:");
//   display.println(farRightValue); //farright
//   display.setCursor(3, 50);
//   display.print("L:");
//   display.println(leftValue); //midleft
//   display.setCursor(65, 50);
//   display.print("R:");
//   display.println(rightValue); //midright
//   display.display();  
// }
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
