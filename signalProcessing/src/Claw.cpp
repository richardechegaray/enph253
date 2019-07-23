#include "Claw.h"

Touch Claw::touch_sensors;
Servo Claw::claw_servo;
float Claw::clockFreq;
float Claw::period;
float Claw::speed;

Claw::Claw(){
    touch_sensors = Touch(); //create the 3 touch sensor instances
    claw_servo = Servo();
    clockFreq = 100000;
    period = 1000;
    claw_servo.attach(CLAW_SERVO);
    claw_servo.write(OPEN_ANGLE);
    pinMode(LEAD_SCREW_UP, OUTPUT);
    pinMode(LEAD_SCREW_DOWN, OUTPUT);
    pinMode(CLAW_QRD, INPUT_PULLUP);
    pwm_start(LEAD_SCREW_UP, clockFreq, period, 0, 1);
    pwm_start(LEAD_SCREW_DOWN, clockFreq, period, 0, 1);
    Serial.begin(9600);
}
void Claw::getStone(){
    //we got info from the ultrasonic sensor that we are close enough to the pillar (we need to drive close enough)

    //if claw touch sensor does not read high (we haven't touched the pillar), keep moving forward -- send info to the drive MCU
    while(touch_sensors.getClawTouch() == LOW){
        Serial3.print("move forward");
    }

    //as soon as claw touch sensor reads high, send info to the drive MCU to stop the wheels
    Serial3.print("stop");


    //now that we have touched the pillar, start moving the lead screw up 
    bool endOfPillar = false; 
    while(touch_sensors.getTopTouch() == LOW){
        if(touch_sensors.getClawTouch() == LOW){
            endOfPillar = true;
            break;
        }
        //raise the lead screw
        pwm_start(LEAD_SCREW_UP, clockFreq, period, speed, 1);
        pwm_start(LEAD_SCREW_DOWN, clockFreq, period, 0, 0);
    }
    if(endOfPillar){
        
        //stop the lead screw
        pwm_start(LEAD_SCREW_UP, clockFreq, period, 0, 1);
        pwm_start(LEAD_SCREW_DOWN, clockFreq, period, 0, 0);

        //close the claw
        claw_servo.write(CLOSE_ANGLE);

        //move the lead screw a bit upwards, if we can, before backing off the pillar for deposit
        if(touch_sensors.getTopTouch() == LOW){
            pwm_start(LEAD_SCREW_UP, clockFreq, period, speed/10, 1);
            pwm_start(LEAD_SCREW_DOWN, clockFreq, period, 0, 0);
            delay(1000);
            pwm_start(LEAD_SCREW_UP, clockFreq, period, 0, 1);
            pwm_start(LEAD_SCREW_DOWN, clockFreq, period, 0, 0);   
        }

        //tell the drive MCU that it can back-off and turn for deposit
        Serial3.println("stone collected");

    } else{
        //meaning that top read high so we should stop raising the lead screw, but the claw still couldn't reach to the top of the pillar: error case
        pwm_start(LEAD_SCREW_UP, clockFreq, period, 0, 1);
        pwm_start(LEAD_SCREW_DOWN, clockFreq, period, 0, 0);
    }
}
void Claw::depositStone(){
    //motors should be stopped, ready for deposit

    //lower the lead screw until the bottom sensor is touching
    while(touch_sensors.getBottomTouch() == LOW){
        pwm_start(LEAD_SCREW_UP, clockFreq, period, 0, 1);
        pwm_start(LEAD_SCREW_DOWN, clockFreq, period, speed, 0);   
    } 

    //read from the reflectance sensor, and move if intensity is not "high" enough (and keep repeating this)
    //move which way?
    //considerations: 
    int reflectance = analogRead(CLAW_QRD);
    while(reflectance < DROP_REFLECTANCE_BEGIN || reflectance > DROP_REFLECTANCE_END){
        Serial3.println("move");
        reflectance = analogRead(CLAW_QRD);
    }
    //reflectance is in the range
    claw_servo.write(OPEN_ANGLE);

    Serial3.println("turn around");   
}
/*
done - the bottom touch sensor will be touching the robot
done - we will detect the pillar with the claw touch sensor, we will start lifting the screw such that the bottom touch sensor wont be touching the robot anymore
done - bottom: to not wreck the motors
done - deposit: we want to lead screw to go close to the robot (read high from bottom)
done - once we get the stone, move the lead screw up a bit more, backoff the robot, turn
done - gauntlet: all the way down until bottom sensor is touching again, look at reflectance sensor, open the servo
done - top: if we ever go too high, we would touch the platform to stop the motors
done - the top and the bottom touch sensors define the limits that we can move the dc motor on the lead screw (top: platform, bottom: robot)
done - so check that we dont go over the top touch sensor when grabbing the stone, and check that we dont go over the bottom touch sensor when depositing the stone
done - additionally use the bottom to go as low as we can for deposit
*/