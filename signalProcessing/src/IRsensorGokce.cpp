// #include <Arduino.h>
// #include "IRSensor.h"

// IRSensor::IRSensor(int pin_, int mode_){
//     pin = pin_;
//     mode = mode_;
// }
// void IRSensor::ir_sample(int ir_pin){
//     //disable interrupts
//     int sum, average = 0;
//     uint32_t t0 = millis();	
//     for(int i=0; i<NUMBER_OF_SAMPLES; i++){
//         sample[i] = analogRead(ir_pin); 
//         sum += sample[i];  
//     }
//     uint32_t t1 = millis();
//     sample_ms = t1 - t0;
//     average = sum/NUMBER_OF_SAMPLES;
//     for(int i=0; i<NUMBER_OF_SAMPLES; i++){
//         sample[i] -= average;
//     }
// }
// int IRSensor::sample_intensity(){
//     int sq_sum=0;
//     for(int i=0; i<NUMBER_OF_SAMPLES; i++){
//         sq_sum += sq(sample[i]);
//     }
//     return sqrt(sq_sum/NUMBER_OF_SAMPLES);
// }
// void IRSensor::reference_signal(){
//     int dt = sample_ms/NUMBER_OF_SAMPLES; //time per sample
//     if(mode == 0){
//         int freq_ms = ONE_KHZ/1000;
//         for(int i=0; i<NUMBER_OF_SAMPLES*2; i++){
//             onek[i] = sin(2*PI*freq_ms*i*dt);
//         }
//     } else{
//         int freq_ms = TEN_KHZ/1000;
//         for(int i=0; i<NUMBER_OF_SAMPLES*2; i++){
//             tenk[i] = sin(2*PI*freq_ms*i*dt);
//         }
//     }
// }
// void IRSensor::sample_frequency(){
//     //spin 360 degrees
//     motor.spin();
//     int max = 0;
//     int corr;
//     if(mode == 0){
//         for(int i=0; i<NUMBER_OF_SAMPLES; i++){
//             corr = 0;
//             for(int j=0; j<NUMBER_OF_SAMPLES; j++){
//                 corr = sample[i]*onek[i+j];
//             }
//             if(corr > max){
//                 max = corr;
//             }
//         }
//     }
//     if(mode == 1){
//         for(int i=0; i<NUMBER_OF_SAMPLES; i++){
//             corr = 0;
//             for(int j=0; j<NUMBER_OF_SAMPLES; j++){
//                 corr = sample[i]*tenk[i+j];
//             }
//             if(corr > max){
//                 max = corr;
//             }
//         }
//     }
//     //now we have max; spin 360 degrees for the 2nd time and stop when you find max
//     int tolerance = 1;
//     motor.spin();
//     if(mode == 0){
//         for(int i=0; i<NUMBER_OF_SAMPLES; i++){
//             corr = 0;
//             for(int j=0; j<NUMBER_OF_SAMPLES; j++){
//                 corr = sample[i]*onek[i+j];
//             }
//             if((corr >= max-tolerance) && (corr <= max+tolerance)){
//                 motor.stop();
//                 motor.goforward();
//             }
//         }
//     }
//     if(mode == 1){
//         for(int i=0; i<NUMBER_OF_SAMPLES; i++){
//             corr = 0;
//             for(int j=0; j<NUMBER_OF_SAMPLES; j++){
//                 corr = sample[i]*tenk[i+j];
//             }
//             if((corr >= max-tolerance) && (corr <= max+tolerance)){
//                 motor.stop();
//                 motor.goforward();
//             }
//         }
//     }
// }
// void IRSensor::depositPlushies(){
//     //hard-coded intensity
//     int intensity = 0;
//     int tolerance = 1;
//     if(mode == 0){
//         for(int i=0; i<NUMBER_OF_SAMPLES; i++){
//             corr = 0;
//             for(int j=0; j<NUMBER_OF_SAMPLES; j++){
//                 corr = sample[i]*onek[i+j];
//             }
//             if((corr >= intensity-tolerance) && (corr <= intensity+tolerance)){
//                 motor.stop();
//                 deposit();
//             }
//         }
//     }
//     if(mode == 1){
//         for(int i=0; i<NUMBER_OF_SAMPLES; i++){
//             corr = 0;
//             for(int j=0; j<NUMBER_OF_SAMPLES; j++){
//                 corr = sample[i]*tenk[i+j];
//             }
//             if((corr >= intensity-tolerance) && (corr <= intensity+tolerance)){
//                 motor.stop();
//                 deposit();
//             }
//         }
//     }
// }