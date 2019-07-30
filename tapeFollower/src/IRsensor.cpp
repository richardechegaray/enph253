#include <Arduino.h>
#include "IRsensor.h"
#define SIZE 100

float raw_sample [SIZE];
float waveform [2*SIZE];

IRsensor::IRsensor(){
    return;
}

IRsensor::IRsensor(int set_pin, int set_mode){
    pin = set_pin;
    pinMode(pin, INPUT);

    sample_size = SIZE; // the amount of values to collect into array for reading
    correlation = 0;

    mode = set_mode;
    if (mode == 1){
        power = 3;
    }
    else if (mode == 10){
        power = 4;
    }
    else {
        Serial.println("Error: incorrect mode");
    }
}

void IRsensor::sample(int pin){
    for (int i = 0; i < sample_size; i++){
        raw_sample[i] = analogRead(pin);
    }
}

void IRsensor::corr(){
    time_start = micros();
    sample(pin);
    time_end = micros();
    duration = (time_end - time_start)/(pow(10,6));
    time_per_sample = duration/sample_size;

    // find average of samples 
    float sum_sample = 0.0;
    for (int i = 0; i < sample_size; i++){
        sum_sample += raw_sample[i];
    }
    average = sum_sample/sample_size;

    // subtract every value in the array by that to centre it
    for (int i = 0; i < sample_size; i++){
        raw_sample[i] = raw_sample[i] - average;
    }

    // generate mode array
    for (int i = 0; i < (2*sample_size); i++){
        waveform[i] = sin(2*PI*pow(10,power)*i*time_per_sample);
    }
    // compare sample array to the generated array , correlation = IRsamples*waveform
    float corr_values[sample_size] = {};
    for (int m = 0; m < (sample_size); m++){
        for (int n = 0; n < (sample_size); n++){
            corr_values[m] += raw_sample[n]*waveform[m+n];
        }
    }

    float highest_val = -1000.0;

    for (float x : corr_values){
        if (x > highest_val){
            highest_val = x;
        }
    }

    correlation = highest_val;
}

    // Serial.println("raw:");
    // for (int i = 0; i < 10; i++){
    //     Serial.println(raw_sample[i]);
    // }

    // Serial.println("wave:");
    // for (int i = 0; i < 10; i++){
    //     Serial.println(waveform[i]);
    // }

    // Serial.println("corr_values:");
    // for (int i = 0; i < 10; i++){
    //     Serial.println(corr_values[i]);
    // }


    // for (int i = 0; i < (2*sample_size); i++){
    //     correlation += corr_values[i];
    // }