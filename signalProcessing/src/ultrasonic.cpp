#include <Arduino.h>
#include <Servo.h>
#include "ultrasonic.h"

#define CM 0.034 // this is 0.034 cm/microsecond speed of sound
#define SERVO_PIN PB14 // needs to be a digital pin
#define CENTER 80

ultrasonic::ultrasonic(int pin_trig, int pin_echo):
trig(pin_trig),
echo(pin_echo)
{
    pinMode(trig, OUTPUT); 
    pinMode(echo, INPUT);
    myservo.attach(SERVO_PIN);
    myservo.write(CENTER); //center
    delay(500);
}

int ultrasonic::get_distance(){
    // set trig pin to low to clear it
    digitalWrite(trig, LOW);
    delayMicroseconds(2);

    // set trig pin to high to send out 10microsecond blast
    digitalWrite(trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig, LOW);

    // use pulseIn function described in header file to capture duration of the echo pin reading
    duration = pulseIn(echo, HIGH);

    // multiply by CM to convert to centimetres, and divide by two, because burst travels there and back
    distance = duration*CM/2;

    return distance;
}

bool ultrasonic::is_there_obj(int min_range, int max_range){
    int distance_to_obs = get_distance();
    if (distance_to_obs < max_range && distance_to_obs > min_range){
        return true;
    }
    else {
        return false;
    }
}

enum ultrasonic::location ultrasonic::loc_of_obj_collection(bool role){
    //left ---------->     right
    int angle_range[] = {CENTER+60, CENTER+40, CENTER+20, CENTER, CENTER-20, CENTER-40, CENTER-60};  
    int start;
    int end;
    if(role){
        //methanos: scan right
        start = 3;
        end = 6;        
    } else{
        start = 0;
        end = 3;
    }
    int obj_detected [] = {0, 0, 0, 0, 0, 0, 0};

    for (int i = start; i <= end; i++){
        myservo.write(angle_range[i]);
        if (i == 0){
            delay(250);
        }
        if (is_there_obj(MIN_RANGE, MAX_RANGE)){
            obj_detected[i] = 1;
        }
        delay(200);
    }
    if(obj_detected[3]){
        return center;
    } else if((obj_detected[1] || obj_detected[2]) && (obj_detected[4] || obj_detected[5])){
        return left_right;
    } else if(obj_detected[1] || obj_detected[2]){
        return left_center;
    } else if(obj_detected[4] || obj_detected[5]){
        return center_right;
    } else if(obj_detected[0]){
        return left;
    } else if(obj_detected[6]){
        return right;
    } else{
        return none;
    }    
}

enum ultrasonic::location ultrasonic::loc_of_obj_deposit(){
    int angle_range[] = {CENTER+45, CENTER, CENTER-45};
    int obj_detected [] = {0, 0, 0};

    for (int i = 0; i < sizeof(angle_range)/sizeof(int); i++){
        myservo.write(angle_range[i]);
        if (i == 0){
            delay(250);
        }
        if (is_there_obj(MIN_RANGE, MAX_RANGE)){
            obj_detected[i] = 1;
        }
        delay(300);
    }

    if(obj_detected[0] && obj_detected[1] && obj_detected[2]){
        return all;
    } else if(obj_detected[0] && obj_detected[1]){
        return left_center;
    } else if(obj_detected[1] && obj_detected[2]){
        return center_right;
    } else if(obj_detected[0] && obj_detected[2]){
        return left_right;
    } else if(obj_detected[1]){
        return center;    
    } else if(obj_detected[0]){
        return left;
    } else if(obj_detected[2]){
        return right;
    } else{
        return none;
    }
}

/*
enum ultrasonic::points ultrasonic::checkLocation(int range){
    int angle_range [] = {150, 130, 110, 90, 70, 50, 30};
    int obj_detected [] = {0, 0, 0, 0, 0, 0, 0};

    for (int i = 0; i < 7; i++){
        myservo.write(angle_range[i]);
        if (i == 0){
            delay(250);
            //distance_zero = get_distance();
        }

        if (is_there_obj(range)){
            obj_detected[i] = 1;
        }
        delay(250);
    }

    //check based on bit priority in here so that we dont need to return an array or anything
    if(obj_detected[3]){
        return c80;
    }
    if(obj_detected[2] || obj_detected[4]){
        if(obj_detected[2] && !obj_detected[4]){
            return l60;
        }
        else if(!obj_detected[2] && obj_detected[4]){
            return r60;
        }
        else {
            return lr60;
        }
    }
    if(obj_detected[1] || obj_detected[5]){
        if(obj_detected[1] && !obj_detected[5]){
            return l40;
        }
        else if(!obj_detected[1] && obj_detected[5]){
            return r40;
        }
        else {
            return lr40;
        }
    }
    if(obj_detected[0] || obj_detected[6]){
        if(obj_detected[0] && !obj_detected[6]){
            return l20;
        }
        else if(!obj_detected[0] && obj_detected[6]){
            return r20;
        }
        else {
            return lr20;
        }
    }
    return lr0;

    if (obj_detected[0] || obj_detected[1]){
        loc = left;
        if ((obj_detected[2]) || (obj_detected[3]) || (obj_detected[4])){
            loc = left_center;
            if ((obj_detected[5] || obj_detected[6])){
                loc = all;
            }
        }
        else if ((obj_detected[5] || obj_detected[6])){
            loc = left_right;
        }
    }
    else if ((obj_detected[2]) || (obj_detected[3]) || (obj_detected[4])){
        loc = center;
        if ((obj_detected[5] || obj_detected[6])){
            loc = center_right;
        }
    }
    else if ((obj_detected[5] || obj_detected[6])){
        loc = right;
    }
    else {
        loc = none;
    }
}*/

