// unsigned int WheelRotaries::countA(int pina, int pinb){
//     aState = digitalRead(pina);
//     if (aState != aLastState){     
//         if (digitalRead(pinb) != aState) { 
//             counter ++;
//         } else {
//             counter --;
//         }
//     }
//     aLastState = aState;
//     return counter;
// }
// unsigned int WheelRotaries::countB(int pina, int pinb){
//     bState = digitalRead(pinb);
//     if (bState != bLastState){
//         if (digitalRead(pina) != bState) { 
//             counter --;
//         } else {
//             counter ++;
//         }    
//     }
//     bLastState = bState;
//     return counter;
// }