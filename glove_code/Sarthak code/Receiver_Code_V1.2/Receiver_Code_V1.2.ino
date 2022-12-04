#include <RH_ASK.h>
#include <SPI.h>

RH_ASK driver(2000,11,50);

int count=0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  driver.init();
  Serial.println("Little Finger;Little Finger Metacarpal;Ring Finger;Ring Finger Metacarpal;Middle Finger Metacarpal;Middle Finger;Fore Finger;Fore Finger Metacarpal;Thumb;");
  Serial.println();
  Serial.println(); 
}

void loop() {

  float val1;
  uint8_t datalen1 = sizeof(val1);
  if (driver.recv((uint8_t*)&val1, &datalen1))
  {
    //Serial.print("The angle from the first sensor is: ");
    Serial.print(val1);
    Serial.print(";");
    //delay(100);
    count++;
  }

  
  float val2;
  uint8_t datalen2 = sizeof(val2);
  if (driver.recv((uint8_t*)&val2, &datalen2))
  {
    //Serial.print("The angle from the second sensor is: ");
    Serial.print(val2);
    Serial.print(";");
    //delay(100);
    count++;
  }
  

  float val3;
  uint8_t datalen3 = sizeof(val3);
  if (driver.recv((uint8_t*)&val3, &datalen3))
  {
    //Serial.print("The angle from the third sensor is: ");
    Serial.print(val3);
    Serial.print(";");
    //delay(100);
    count++;
  }
  

  float val4;
  uint8_t datalen4 = sizeof(val4);
  if (driver.recv((uint8_t*)&val4, &datalen4))
  {
    //Serial.print("The angle from the fourth sensor is: ");
    Serial.print(val4);
    Serial.print(";");
    //delay(100);
    count++;
  }
  

  float val5;
  uint8_t datalen5 = sizeof(val5);
  if (driver.recv((uint8_t*)&val5, &datalen5))
  {
    //Serial.print("The angle from the fifth sensor is: ");
    Serial.print(val5);
    Serial.print(";");
    //delay(100);
    count++;
  }
  

  float val6;
  uint8_t datalen6 = sizeof(val6);
  if (driver.recv((uint8_t*)&val6, &datalen6))
  {
    //Serial.print("The angle from the sixth sensor is: ");
    Serial.print(val6);
    Serial.print(";");
    //delay(100);
    count++;
  }
  

  float val7;
  uint8_t datalen7 = sizeof(val7);
  if (driver.recv((uint8_t*)&val7, &datalen7))
  {
    //Serial.print("The angle from the seventh sensor is: ");
    Serial.print(val7);
    Serial.print(";");
    //delay(100);
    count++;
  }
  

  float val8;
  uint8_t datalen8 = sizeof(val8);
  if (driver.recv((uint8_t*)&val8, &datalen8))
  {
    //Serial.print("The angle from the seventh sensor is: ");
    Serial.print(val8);
    Serial.print(";");
    //delay(100);
    count++;
  }
 

  float val9;
  uint8_t datalen9 = sizeof(val9);
  if (driver.recv((uint8_t*)&val9, &datalen9))
  {
    //Serial.print("The angle from the ninth sensor is: ");
    Serial.print(val9);
    Serial.print(";");
    //delay(100);
    count++;
  }

  if(count==9){
    Serial.println();
    count=0;
  }

}
