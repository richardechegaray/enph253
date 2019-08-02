// // right wheel tends to be faster when speeds are set to be equal
// // METHANOS = 1 kHz, turn left twice, then right
// // THANOS = 10 kHz, turn right twice, then left

// #include <Arduino.h>
// #include <pid.h>
// #include "IRdecision.h" 
// // #include <Wire.h>
// // #include <Adafruit_SSD1306.h>
// // #include <FreeMono9pt7b.h>
// // #define OLED_RESET -1 //for reset button
// // Adafruit_SSD1306 display(OLED_RESET);

// #define FAR_LEFT PB12
// #define LEFT_SENSOR PB13  
// #define RIGHT_SENSOR PB14
// #define FAR_RIGHT PB15

// #define LEFT_MOTOR_FW PA_3 
// #define LEFT_MOTOR_BW PA_2
// #define RIGHT_MOTOR_FW PA_0
// #define RIGHT_MOTOR_BW PA_1

// #define ON 1
// #define OFF 0

// #define LEFT_IR PB0
// #define MID_IR PB1
// #define RIGHT_IR PA7

// #define TX3 PB10
// #define RX3 PB11
// HardwareSerial Serial3 = HardwareSerial(RX3, TX3);

// #define RAMP_TIME 15
// #define COLLECT_TIME 27

// // #define KP_POTMETER PA0
// // #define KD_POTMETER PA1

// #define QUARTER_TURN_TIME 2 //90 degrees turn time (seconds)

// float clockFreq = 100000;
// float period = 1000;

// float initialTime;
// float timeElapsed;

// float spinSpeed = 22*period/100; //make sure this isnt too fast, 20 is too slow
// float targetIrSpeed = 25*period/100;
// float targetIrSpeedPlus = 30*period/100;
// float targetIrSpeedMinus = 20*period/100;

// float targetSpeed = 60*period/100;
// float leftSpeed = targetSpeed;
// float rightSpeed = targetSpeed;

// float irStartThreshold = 900;
// float midThreshold = 1000; //past columns
// float closeThreshold = 1550;  //quite close

// float midIntensity = -1;
// int highestPin = -1;

// float leftValue = 0.0;
// float rightValue = 0.0;
// float farLeftValue = 0.0;
// float farRightValue = 0.0;
// bool irDefined;

// float error = 0.0;
// int numberOfTurns = 0;

// enum majorState { upRamp, collectPlushie, depositPlushie, stones, avoidCollision/*, shutDown*/ } currentMajorState, previousMajorState;
// enum pidState { onTrack, leftOff, rightOff, turnLeft, turnRight, white, malfunc} currentPidState, previousPidState;
// enum irState { initialSpin, drivingFar, drivingClose, /*avoid,*/ stop} currentIrState;
// enum collisionState { firstTurn, driveStraight, lastTurn } currentCollisionState;

// pid p_i_d;
// // float kp_reading;
// // float kd_reading;

// #define THANOS 0
// #define METHANOS 1
// #define ROLE METHANOS

// #if (ROLE == THANOS)
//   IRdecision decision = IRdecision(LEFT_IR, MID_IR, RIGHT_IR, 10); //10 kHz
// #elif (ROLE == METHANOS)
//   IRdecision decision = IRdecision(LEFT_IR, MID_IR, RIGHT_IR, 1); //1 kHz
// #endif

// int collision = 0; //the bit we receive from the signalProcessing MCU to indicate if we need to make a 90 degree turn or not

// float speedCapOff(float speed); //add for all functions
// pidState getPidState(float left, float right, float farLeft, float farRight);
// void drive(float bwLeft, float fwLeft, float bwRight, float fwRight);
// void pidStateMachine();
// void irStateMachine();
// void irDrive(irState currentIrState);
// void collisionStateMachine();
// // void displayPID();
// // void displayRefl();
// // void displayIR();

// void setup() {
//     Serial.begin(115200);
//     Serial3.begin(9600);

//     pinMode(LEFT_SENSOR, INPUT_PULLUP); 
//     pinMode(RIGHT_SENSOR, INPUT_PULLUP); 
//     pinMode(FAR_LEFT, INPUT_PULLUP);
//     pinMode(FAR_RIGHT, INPUT_PULLUP);
        
//     pinMode(LEFT_MOTOR_FW, OUTPUT);
//     pinMode(LEFT_MOTOR_BW, OUTPUT);
//     pinMode(RIGHT_MOTOR_FW,OUTPUT);
//     pinMode(RIGHT_MOTOR_BW, OUTPUT);

//     pwm_start(LEFT_MOTOR_FW, clockFreq, period, 0, 1); // Initializing all motors
//     pwm_start(LEFT_MOTOR_BW, clockFreq, period, 0, 1); 
//     pwm_start(RIGHT_MOTOR_FW, clockFreq, period, 0, 1); 
//     pwm_start(RIGHT_MOTOR_BW, clockFreq, period, 0, 1); 
//     p_i_d = pid();
//     previousPidState = onTrack;
//     currentIrState = initialSpin;
//     currentMajorState = upRamp;
//     previousMajorState = stones; //this cannot be initialized as upRamp because in the first run we need it to be different than currentMajorState for it to be communicated
//     irDefined = false;
//     currentCollisionState = firstTurn; 

//     // kd_reading = p_i_d.kd; // Potentiometers
//     // kp_reading = p_i_d.kp;

//     // display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // OLED Display
//     // display.clearDisplay();
//     // display.setTextColor(WHITE);
//     // display.setFont(&FreeMono9pt7b);

//     initialTime = millis();
// }

// void loop() {  // SLAVE
//   timeElapsed = (millis() - initialTime)/1000; // in seconds
//   if (collision == 1)
//     currentMajorState = avoidCollision;
//   else if (timeElapsed < RAMP_TIME)
//     currentMajorState = upRamp;
//   else if (timeElapsed < COLLECT_TIME)
//     currentMajorState = collectPlushie;
//   else if (currentIrState == stop)
//     currentMajorState = stones;
//   else {
//     if (irDefined == false) {
//       irDefined = true;
//       currentMajorState = depositPlushie;
//     }
//   }

//   if (currentMajorState != previousMajorState) {
//     while (!Serial3.availableForWrite()) {}
//     Serial3.write((int)currentMajorState); 
//   }


//   leftValue = digitalRead(LEFT_SENSOR);
//   rightValue = digitalRead(RIGHT_SENSOR);
//   farLeftValue = digitalRead(FAR_LEFT);
//   farRightValue = digitalRead(FAR_RIGHT);

//   switch ( currentMajorState ) { // state machine
//     case upRamp:
//       currentPidState = getPidState(leftValue, rightValue, farLeftValue, farRightValue);
//       pidStateMachine();
//       break;

//     case collectPlushie:
//       currentPidState = getPidState(leftValue, rightValue, farLeftValue, farRightValue);
//       pidStateMachine();
//       break;

//     case depositPlushie:
//       irStateMachine();
//       break;

//     case stones:
//       drive(0, 0, 0, 0);
//       break;

//     case avoidCollision:
//         collisionStateMachine();
//         break;
//   }
//   previousMajorState = currentMajorState;

//   /*check for collision avoidance: */
//   collision = Serial3.read();
// }
 

// //increase the first turn left delay
// pidState getPidState(float left, float right, float farLeft, float farRight){

//   #if (ROLE == THANOS) 
//     if (farLeft == ON) {
//       if (currentMajorState == upRamp) { //if its 0 or 1
//         numberOfTurns++;
//         if (numberOfTurns == 1)  //first turn
//           delay(400); //up ramp delay
//         return turnLeft;
//       }
//     }

//     if ( currentMajorState == collectPlushie ) {
//       if (farRight == ON)
//         return turnRight; 
//     }
//   #elif (ROLE == METHANOS) 
//     if (farRight == ON) {
//       if (currentMajorState == upRamp) { //if its 0 or 1 //got rid of number of turns <2
//         numberOfTurns++;
//         if (numberOfTurns == 1)  //first turn
//           delay(400); //up ramp delay, used to be 300
//         return turnRight;
//       }
//     }

//     if ( currentMajorState == collectPlushie ) {
//       if (farLeft == ON)
//         return turnLeft; 
//     }
//   #endif

//   if ( (left == ON) && (right == ON) )
//     return onTrack;
//   else if ( (right == ON) && (left == OFF) )
//     return leftOff;
//   else if ( (right == OFF) && (left == ON) )
//     return rightOff;
//   else
//     return white;
     
// }

// //modular
// void pidStateMachine() {

//   //update kp and kd values if we modified with the potentiometer
//   // kp_reading = analogRead(KP_POTMETER);
//   // if(kp_reading != p_i_d.kp){
//   //   p_i_d.kp = map(kp_reading, 0, 1023, 0, 500); 
//   // }
//   // kd_reading = analogRead(KD_POTMETER);
//   // if(kd_reading != p_i_d.kd){
//   //   p_i_d.kd = map(kd_reading, 0, 1023, 0, 500);
//   // }
//   switch ( currentPidState ) { //state machine

//     case onTrack : //drive straight
//       error = 0; 
//       leftSpeed = targetSpeed + p_i_d.output_pid(error);
//       rightSpeed = targetSpeed + p_i_d.output_pid(error);
//       // Serial.println(error);
//       // Serial.println(leftSpeed);
//       // Serial.println(rightSpeed);
//       // Serial.println();
//       drive(0, leftSpeed, 0, rightSpeed);
//       break;

//     case leftOff : //turn right
//       error = 1; 
//       leftSpeed = targetSpeed + p_i_d.output_pid(error);
//       rightSpeed = targetSpeed + p_i_d.output_pid(-error);
//       // Serial.println(error);
//       // Serial.println(leftSpeed);
//       // Serial.println(rightSpeed);
//       // Serial.println();
//       drive(0, leftSpeed, 0, rightSpeed); //left needs to catch up, right side weaker
//       break;

//     case rightOff : //turn left
//       error = 1; 
//       leftSpeed = targetSpeed + p_i_d.output_pid(-error);
//       rightSpeed = targetSpeed + p_i_d.output_pid(error);
//       // Serial.println(error);
//       // Serial.println(leftSpeed);
//       // Serial.println(rightSpeed);
//       // Serial.println();
//       drive(0, leftSpeed, 0, rightSpeed); //left weaker, right needs to catch up 
//       break;

//     case white : //both sensors off tape
//       error = 3; //4
//       if(previousPidState == leftOff) {  // continue to turn right
//         leftSpeed = targetSpeed + p_i_d.output_pid(error);
//         rightSpeed = targetSpeed + p_i_d.output_pid(-error);
//         drive(0, leftSpeed, 0, rightSpeed); //left needs to catch up, right side weaker
//       } else if(previousPidState == rightOff) {  // continue to turn left
//         leftSpeed = targetSpeed + p_i_d.output_pid(-error);
//         rightSpeed = targetSpeed + p_i_d.output_pid(error);
//         drive(0, leftSpeed, 0, rightSpeed); //left weaker, right needs to catch up
//       } else if(previousPidState == turnLeft){ //continue to turn left
//         leftSpeed = targetSpeed + p_i_d.output_pid(-error);
//         rightSpeed = targetSpeed + p_i_d.output_pid(error);
//         drive(0, leftSpeed, 0, rightSpeed);  
//       } else if (previousPidState == turnRight){ //continue to turn right
//         leftSpeed = targetSpeed + p_i_d.output_pid(error);
//         rightSpeed = targetSpeed + p_i_d.output_pid(-error);
//         drive(0, leftSpeed, 0, rightSpeed);  
//       } 
//       // Serial.println(error);
//       // Serial.println(leftSpeed);
//       // Serial.println(rightSpeed);
//       // Serial.println();
//       break;
    
//     case turnLeft :
//       if (numberOfTurns == 1)
//         error = 2.5;  //2
//       else
//         error = 4; //9
 
//       leftSpeed  = targetSpeed + p_i_d.output_pid(-error);
//       rightSpeed = targetSpeed + p_i_d.output_pid(error);
//       // Serial.println(error);
//       // Serial.println(leftSpeed);
//       // Serial.println(rightSpeed);
//       // Serial.println();
//       drive(0, leftSpeed, 0, rightSpeed); //left weaker, right needs to catch up 
//       delay(200);  
//       if (numberOfTurns == 1) 
//         delay(800);

//       break;

//     case turnRight : //not doing yet
//       if (numberOfTurns == 1) 
//         error = 2.5;  //2
//       else
//         error = 4;  //9      
      
//       leftSpeed = targetSpeed + p_i_d.output_pid(error);
//       rightSpeed = targetSpeed + p_i_d.output_pid(-error);
//       // Serial.println(error);
//       // Serial.println(leftSpeed);
//       // Serial.println(rightSpeed);
//       // Serial.println();
//       drive(0, leftSpeed, 0, rightSpeed); //left needs to catch up, right side weaker
//       delay(200); 
//       if (numberOfTurns == 1) 
//         delay(800);
      
//       break;

//     default:
//       drive(0, 0, 0, 0); //spin for 5 seconds
//       delay(5000);  // this should never happen...
//       break;  
//   }

//   if (currentPidState != onTrack && currentPidState != white)
//     previousPidState = currentPidState;

//   // displayPID();
//   // displayRefl(); //these display 'global values'
// }

// void irStateMachine() {
//   /*float number = decision.strongest_signal();
//   if (number == LEFT_IR) 
//     Serial.println("Left!!!");
//   else if (number == MID_IR) 
//     Serial.println("Middle!!!");
//   else if (number == RIGHT_IR) 
//     Serial.println("Right!!!");
//   else 
//     Serial.println("error finding max");

//   float leftIntensity = decision.corrleft;
//   midIntensity = decision.corrcenter;
//   float rightIntensity = decision.corrright;

//   Serial.print("Left: ");
//   Serial.println(leftIntensity);
//   Serial.print("Middle: ");
//   Serial.println(midIntensity);
//   Serial.print("Right: ");
//   Serial.println(rightIntensity);
//   Serial.print("Max correlation pin: ");
  
//   delay(1000);*/
//   switch ( currentIrState ) { //state machine
//     case initialSpin : //drive straight
//       highestPin = decision.strongest_signal();
//       midIntensity = decision.corrcenter;
//       drive(0, spinSpeed, spinSpeed, 0); // spin CW if methanos
      
//       if ((highestPin == MID_IR) && (midIntensity > irStartThreshold)) {
//         drive(0,0,0,0);
//         currentIrState = drivingFar;
//       }
//       //displayIR(); 
//       break;

//     case drivingFar :
//       midIntensity = decision.corrcenter;
//       if (midIntensity > closeThreshold) {
//         currentIrState = drivingClose;
//       } 
//       irDrive(currentIrState);
//       // displayIR();
//       break;  

//     case drivingClose :
//       midIntensity = decision.corrcenter;
//       leftValue = digitalRead(LEFT_SENSOR);
//       rightValue = digitalRead(RIGHT_SENSOR);
//       farLeftValue = digitalRead(FAR_LEFT);
//       farRightValue = digitalRead(FAR_RIGHT);

//       if ((leftValue == ON) || (rightValue  == ON) || (farLeftValue  == ON) || (farRightValue == ON)) {
//         currentIrState = stop;
//         break;
//       }
//       irDrive(currentIrState);
//       // displayIR();
//       break;    
    
//     case stop :
//       drive(0,0,0,0);
//       delay(500);
//       // currentMajorState = stones;
//       // displayIR();
//       break;
//   }
// }

// void drive(float bwLeft, float fwLeft, float bwRight, float fwRight) { // DO NOT TRY TO RUN FW AND BW DIRECTION FOR ONE WHEEL AT A TIME

//   if ((currentPidState == turnLeft)||(currentPidState == turnRight)) {
//     if (fwRight < 0) {
//       bwRight = -fwRight;
//       fwRight = 0;
//     }
//     if (fwLeft < 0) {
//       bwLeft = -fwLeft;
//       fwLeft = 0;
//     }
//   }

//   fwLeft = speedCapOff(fwLeft);
//   fwRight = speedCapOff(fwRight);
//   bwLeft = speedCapOff(bwLeft);
//   bwRight = speedCapOff(bwRight);

//   // pwm_start(LEFT_MOTOR_BW, clockFreq, period, bwLeft, 0); 
//   // pwm_start(LEFT_MOTOR_FW, clockFreq, period, fwLeft, 0); 
//   // pwm_start(RIGHT_MOTOR_BW, clockFreq, period, bwRight, 0); 
//   // pwm_start(RIGHT_MOTOR_FW, clockFreq, period, fwRight, 0); 
// }

// //modular
// void irDrive(irState currentIrState) {
//   float speed, speedPlus, speedMinus;

//   if (currentIrState == drivingFar) {
//     speed = targetIrSpeed; // turn sharper if far away
//     speedPlus = targetIrSpeedPlus;
//     speedMinus = targetIrSpeedMinus;
//   } else if (currentIrState == drivingClose) {
//     speed = 4*targetIrSpeed/5;
//     speedPlus = 4*targetIrSpeedPlus/5;
//     speedMinus = 4*targetIrSpeedMinus/5;
//   } else {
//     speed = 0;
//     speedPlus = 0;
//     speedMinus = 0;
//   }

//   highestPin = decision.strongest_signal();
//   if (highestPin == LEFT_IR) 
//     drive(0, speedMinus, 0, speedPlus);
//   else if (highestPin == RIGHT_IR) 
//     drive(0, speedPlus, 0, speedMinus);
//   else
//     drive(0, speed, 0, speed);
// }

// //modular
// float speedCapOff(float speed) {
//   if (speed>period)
//     return period;
//   else if (speed<0)
//     return 12*period/100;
//   else
//     return speed;
// }

// void collisionStateMachine(){
//     #if (ROLE == THANOS)
//         int turn_error = 7; //turn right (when error is positive)
//     #elif (ROLE == METHANOS)
//         int turn_error = -7; //turn left
//     #endif

//     switch( currentCollisionState ){
//         case firstTurn:
//             //turn 90*
//             leftSpeed = targetSpeed + p_i_d.output_pid(turn_error);
//             rightSpeed = targetSpeed + p_i_d.output_pid(-turn_error);
//             float start = millis(); //would this interfere with the other millis?
//             float interval = (millis() - start)/1000;
//             while (interval != QUARTER_TURN_TIME){
//                 drive(0, leftSpeed, 0, rightSpeed);
//                 interval = (millis() - start)/1000;
//             }
//             currentCollisionState = driveStraight;
//             break;
        
//         case driveStraight:
//             //drive straight (0 error)
//             leftValue = digitalRead(LEFT_SENSOR);
//             rightValue = digitalRead(RIGHT_SENSOR);
//             leftSpeed = targetSpeed;
//             rightSpeed = targetSpeed;
//             while (!(leftValue == ON && rightValue == ON)){ //while both are not high!
//                 drive(0, leftSpeed, 0, rightSpeed);
//                 leftValue = digitalRead(LEFT_SENSOR);
//                 rightValue = digitalRead(RIGHT_SENSOR);
//             }
//             currentCollisionState = lastTurn;
//             break;
        
//         case lastTurn:
//             //turn 90* again and then follow tape regularly (leave this function)
//             leftSpeed = targetSpeed + p_i_d.output_pid(turn_error);
//             rightSpeed = targetSpeed + p_i_d.output_pid(-turn_error);
//             start = millis();
//             interval = (millis() - start)/1000;
//             while (interval != QUARTER_TURN_TIME){
//                 drive(0, leftSpeed, 0, rightSpeed);
//                 interval = (millis() - start)/1000;
//             }
//             collision = 0;
//             currentCollisionState = firstTurn; 
//             //when we set collision=0, we should not come back to this function-- so for the next collision detection, we set currentCollState to firstTurn once again
//             break;  
//     }   
// }

// // void displayPID(){
// //   display.clearDisplay();
// //   display.setCursor(5,20);
// //   display.print("kp:");
// //   display.println(p_i_d.kp);
// //   display.setCursor(5,40);
// //   display.print("kd:");
// //   display.println(p_i_d.kd);
// //   display.display();
// // }
// // void displayRefl(){
// //   display.clearDisplay();
// //   display.setCursor(3,20);
// //   display.print("FL:");
// //   display.println(farLeftValue); //farleft
// //   display.setCursor(65,20);
// //   display.print("FR:");
// //   display.println(farRightValue); //farright
// //   display.setCursor(3, 50);
// //   display.print("L:");
// //   display.println(leftValue); //midleft
// //   display.setCursor(65, 50);
// //   display.print("R:");
// //   display.println(rightValue); //midright
// //   display.display();  
// // }
// // void displayIR(){
// //   display.clearDisplay();
// //   display.setCursor(0,20);
// //   display.print("L");
// //   display.println(decision.corrleft); //left
// //   display.setCursor(30,50);
// //   display.print("M");
// //   display.println(decision.corrcenter); //mid
// //   display.setCursor(63,20);
// //   display.print("R");
// //   display.println(decision.corrright); //right
// //   display.display();
// // }
