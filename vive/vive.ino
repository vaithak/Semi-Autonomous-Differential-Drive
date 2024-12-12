// Use vive.h to read and print the Vive data

#include "vive.h"

void setup() {
    Serial.begin(115200);
    vive_setup();
}

void loop() {
    ViveUpdate();
}

/*
 * Sample Vive code
 */
// #include "vive510.h"

// #define SIGNALPIN1 34 // pin receiving signal from Vive circuit
// #define SIGNALPIN2 7 // pin receiving signal from Vive circuit

// Vive510 vive1(SIGNALPIN1);
// Vive510 vive2(SIGNALPIN2);

// #define FREQ 1 // in Hz
// void setup() {
//   Serial.begin(115200);
//   pinMode(LED_BUILTIN,OUTPUT);

//   vive1.begin();
//   vive2.begin();
//   Serial.println("Vive trackers= started");
// }
                 
// uint32_t med3filt(uint32_t a, uint32_t b, uint32_t c) {
//   uint32_t middle;
//   if ((a <= b) && (a <= c))
//     middle = (b <= c) ? b : c;  
//   else if ((b <= a) && (b <= c))
//     middle = (a <= c) ? a : c;
//   else    middle = (a <= b) ? a : b;
//   return middle;
// }
                               
// void loop() {  
//   static uint16_t x1,y1,x2,y2;
    
//   // Process vive1 data
//   if (vive1.status() == VIVE_RECEIVING) {
//     static uint16_t x0, y0, oldx1, oldx2, oldy1, oldy2;
//     oldx2 = oldx1; oldy2 = oldy1;
//     oldx1 = x0;     oldy1 = y0;
    
//     x0 = vive1.xCoord();
//     y0 = vive1.yCoord();
    
//     x1 = med3filt(x0, oldx1, oldx2);
//     y1 = med3filt(y0, oldy1, oldy2);
//     Serial.print("\nVive1 status: ");
//     Serial.print(vive1.status());
//     Serial.print(", X coord: ");
//     Serial.print(x1);
//     Serial.print(", Y coord: ");
//     Serial.print(y1);
    
//     digitalWrite(LED_BUILTIN,HIGH);
//     if (x1 > 8000 || y1 > 8000 || x1 < 1000 || y1 < 1000) {
//       x1=0; y1=0;
//       digitalWrite(LED_BUILTIN,LOW);
//     }
//   }
//   else {
//     digitalWrite(LED_BUILTIN,LOW);
//     x1=0;
//     y1=0; 
//     vive1.sync(5); 
//   }

//   // Process vive2 data
//   if (vive2.status() == VIVE_RECEIVING) {
//     static uint16_t x0, y0, oldx1, oldx2, oldy1, oldy2;
//     oldx2 = oldx1; oldy2 = oldy1;
//     oldx1 = x0;     oldy1 = y0;
    
//     x0 = vive2.xCoord();
//     y0 = vive2.yCoord();
    
//     x2 = med3filt(x0, oldx1, oldx2);
//     y2 = med3filt(y0, oldy1, oldy2);
//     Serial.print("\nVive2 status: ");
//     Serial.print(vive2.status());
//     Serial.print(", X coord: ");
//     Serial.print(x2);
//     Serial.print(", Y coord: ");
//     Serial.print(y2);
    
//     if (x2 > 8000 || y2 > 8000 || x2 < 1000 || y2 < 1000) {
//       x2=0; y2=0;
//     }
//   }
//   else {
//     x2=0;
//     y2=0; 
//     vive2.sync(5); 
//   }
    
//   delay(100);
// }
