#include <RH_ASK.h>
#include <SPI.h>
#include <Arduino.h>

RH_ASK driver(2000,11,50);
const int num_data=202;   //9+12*16+1
float val[num_data];
uint8_t datalen[num_data];

int count=0;

void setup() {
  Serial.begin(9600);
  driver.init();
  
  for(int i=1;i<=16;i++){
  Serial.print("IMU");
  Serial.print(i);
  Serial.print("_gx; gy; gz; ax; ay; az; mx; my; mz; pitch; roll; heading;");
  }
  Serial.print("Little Finger;Little Finger Metacarpal;Ring Finger;Ring Finger Metacarpal;Middle Finger Metacarpal;Middle Finger;Fore Finger;Fore Finger Metacarpal;Thumb;");
  Serial.println("\n"); 
 // Serial.println("begin");
}

void loop() {
for(int i=0;i<num_data;){
 datalen[i] = sizeof(val[i]);
  while (driver.recv((uint8_t*)&val[i], &datalen[i]))
  {

    //   if(val[i]=0){
    //    Serial.print("\n");
    //    i=0;
    //   } 
    //Serial.print("The angle from the first sensor is: ");
    Serial.print(val[i]);
    Serial.print(";");
    //delay(100);
    i++;
  }
}
Serial.print("\n");
}