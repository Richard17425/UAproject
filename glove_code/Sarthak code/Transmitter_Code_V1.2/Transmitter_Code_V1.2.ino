#include <RH_ASK.h>
#include <SPI.h>


int sen1=A0;
int sen2=A1;
int sen3=A2;
int sen4=A3;
int sen5=A10;
int sen6=A11;
int sen7=A12;
int sen8=A13;
int sen9=A14;


float zeroVal1=0;
float zeroVal2=0;
float zeroVal3=0;
float zeroVal4=0;
float zeroVal5=0;
float zeroVal6=0;
float zeroVal7=0;
float zeroVal8=0;
float zeroVal9=0;


float ninetyVal1=0;
float ninetyVal2=0;
float ninetyVal3=0;
float ninetyVal4=0;
float ninetyVal5=0;
float ninetyVal6=0;
float ninetyVal7=0;
float ninetyVal8=0;
float ninetyVal9=0;


int loops=100;


int initalDelay=500;
int delayTime=100;


int VCC=5.15;


int button1=4;
int button2=8;
int ledPin=12;


int delayDebounce=50;


float sum1=0;
float sum2=0;
float sum3=0;
float sum4=0;
float sum5=0;
float sum6=0;
float sum7=0;
float sum8=0;
float sum9=0;


float val0;
float val1;
float val2;
float val3;
float val4;
float val5;
float val6;
float val7;
float val8;
float val9;


int checkZero=LOW;
int checkNinety=LOW;


float data0;
float data1;
float data2;
float data3;
float data4;
float data5;
float data6;
float data7;
float data8;
float data9;


RH_ASK driver(2000,11,50);


void setup() {
 Serial.begin(9600);
  
  pinMode(sen1,INPUT);
  pinMode(sen2,INPUT);
  pinMode(sen3,INPUT);
  pinMode(sen4,INPUT);
  pinMode(sen5,INPUT);
  pinMode(sen6,INPUT);
  pinMode(sen7,INPUT);
  pinMode(sen8,INPUT);
  pinMode(sen9,INPUT);

  
  pinMode(button1,INPUT);
  pinMode(button2,INPUT);
  pinMode(ledPin,OUTPUT);
  

  driver.init();

  if (!driver.init()){
    Serial.println("init failed");
  }
  
}

void loop() {
  
//////////////////////////the button1(yellow one) been pushed/////////////////////////////////

if(digitalRead(button1)==LOW && digitalRead(button2)==HIGH && checkZero==LOW){
    delay(delayDebounce);
   
    if(digitalRead(button1)==LOW && digitalRead(button2)==HIGH){
  
  digitalWrite(ledPin,HIGH);
  delay(initalDelay);
  
  for(int i=0;i<loops;i++){
    zeroVal1=zeroVal1 + (analogRead(sen1)*VCC)/1023.0;
    zeroVal2=zeroVal2 + (analogRead(sen2)*VCC)/1023.0;
    zeroVal3=zeroVal3 + (analogRead(sen3)*VCC)/1023.0;
    zeroVal4=zeroVal4 + (analogRead(sen4)*VCC)/1023.0;
    zeroVal5=zeroVal5 + (analogRead(sen5)*VCC)/1023.0;
    zeroVal6=zeroVal6 + (analogRead(sen6)*VCC)/1023.0;
    zeroVal7=zeroVal7 + (analogRead(sen7)*VCC)/1023.0;
    zeroVal8=zeroVal8 + (analogRead(sen8)*VCC)/1023.0;
    zeroVal9=zeroVal9 + (analogRead(sen9)*VCC)/1023.0;
    //delay(delayTime);
  }

    zeroVal1=zeroVal1/loops;
    zeroVal2=zeroVal2/loops;
    zeroVal3=zeroVal3/loops;
    zeroVal4=zeroVal4/loops;
    zeroVal5=zeroVal5/loops;
    zeroVal6=zeroVal6/loops;
    zeroVal7=zeroVal7/loops;
    zeroVal8=zeroVal8/loops;
    zeroVal9=zeroVal9/loops;
    
   checkZero=HIGH;  
   Serial.println("The zero angle value has been set.");
   Serial.println(zeroVal7);
  
//   const char *msg1 = "The zero degree angle value has been set.";
//   driver.send((uint8_t *)msg1, strlen(msg1));
//   driver.waitPacketSent();
//   //Serial.println("Hello World");
//   //delay(100);

    digitalWrite(ledPin,LOW);
 }
}

//////////////////////////the button2(green one) been pushed/////////////////////////////////

if(digitalRead(button2)==LOW && digitalRead(button1)==HIGH && checkNinety==LOW){
    delay(delayDebounce);
    if(digitalRead(button2)==LOW && digitalRead(button1)==HIGH){
  
 
  digitalWrite(ledPin,HIGH);
  delay(initalDelay);
  
  for(int i=0;i<loops;i++){
    ninetyVal1=ninetyVal1 + (analogRead(sen1)*VCC)/1023.0;
    ninetyVal2=ninetyVal2 + (analogRead(sen2)*VCC)/1023.0;
    ninetyVal3=ninetyVal3 + (analogRead(sen3)*VCC)/1023.0;
    ninetyVal4=ninetyVal4 + (analogRead(sen4)*VCC)/1023.0;
    ninetyVal5=ninetyVal5 + (analogRead(sen5)*VCC)/1023.0;
    ninetyVal6=ninetyVal6 + (analogRead(sen6)*VCC)/1023.0;
    ninetyVal7=ninetyVal7 + (analogRead(sen7)*VCC)/1023.0;
    ninetyVal8=ninetyVal8 + (analogRead(sen8)*VCC)/1023.0;
    ninetyVal9=ninetyVal9 + (analogRead(sen9)*VCC)/1023.0;
    delay(delayTime);
  }

    ninetyVal1=ninetyVal1/loops;
    ninetyVal2=ninetyVal2/loops;
    ninetyVal3=ninetyVal3/loops;
    ninetyVal4=ninetyVal4/loops;
    ninetyVal5=ninetyVal5/loops;
    ninetyVal6=ninetyVal6/loops;
    ninetyVal7=ninetyVal7/loops;
    ninetyVal8=ninetyVal8/loops;
    ninetyVal9=ninetyVal9/loops;
  
  checkNinety=HIGH; 
  Serial.println("The ninety angle value has been set.");
  
//  const char *msg2 = "The ninety degree angle value has been set.";
//  driver.send((uint8_t *)msg2, strlen(msg2));
//  driver.waitPacketSent();
//  //Serial.println("Hello World");
//  //delay(100);

    digitalWrite(ledPin,LOW); 
 }
}

//////////////////////////two buttons been pushed/////////////////////////////////

if(digitalRead(button1)==LOW && digitalRead(button2)==LOW){
  delay(delayDebounce);
  if(digitalRead(button1)==LOW && digitalRead(button2)==LOW){
    if(checkZero==HIGH && checkNinety==HIGH){

  //Serial.println("started collecting data");
  digitalWrite(ledPin,HIGH);
  delay(70);
  digitalWrite(ledPin,LOW);
  delay(70);
  digitalWrite(ledPin,HIGH);
  delay(70);
  digitalWrite(ledPin,LOW);
  delay(70);
  digitalWrite(ledPin,HIGH);
  delay(70);
  digitalWrite(ledPin,LOW);
  delay(70);
  digitalWrite(ledPin,HIGH);
  delay(70);
  digitalWrite(ledPin,LOW);
  delay(70);
  digitalWrite(ledPin,HIGH);
  delay(70);
  digitalWrite(ledPin,LOW);
  delay(70);
  digitalWrite(ledPin,HIGH);
  delay(70);
  digitalWrite(ledPin,LOW);
  delay(70);
  digitalWrite(ledPin,HIGH);
  delay(70);
  digitalWrite(ledPin,LOW);
  delay(70);
  digitalWrite(ledPin,HIGH);
  delay(70);
  digitalWrite(ledPin,LOW);
  while(1!=0){
  sum1=0;
  sum2=0;
  sum3=0;
  sum4=0;
  sum5=0;
  sum6=0;
  sum7=0;
  sum8=0;
  sum9=0;

  delay(initalDelay);
//for(int i=0;i<loops;i++){
    sum1=sum1 + (analogRead(sen1)*VCC)/1023.0;
    sum2=sum2 + (analogRead(sen2)*VCC)/1023.0;
    sum3=sum3 + (analogRead(sen3)*VCC)/1023.0;
    sum4=sum4 + (analogRead(sen4)*VCC)/1023.0;
    sum5=sum5 + (analogRead(sen5)*VCC)/1023.0;
    sum6=sum6 + (analogRead(sen6)*VCC)/1023.0;
    sum7=sum7 + (analogRead(sen7)*VCC)/1023.0;
    sum8=sum8 + (analogRead(sen8)*VCC)/1023.0;
    sum9=sum9 + (analogRead(sen9)*VCC)/1023.0;
    //delay(delayTime);
//  }


val0=mapf(sum1,zeroVal1,ninetyVal1,0,90.0);
  data0=val0;
//  Serial.print("The angle from the first sensor is: ");
//  Serial.println(data0);
  driver.send((uint8_t *)&data0, sizeof(data0));
  driver.waitPacketSent();
//  Serial.println("sent");
//  delay(100);

val1=mapf(sum1,zeroVal1,ninetyVal1,0,90.0);
  data1=val1;
  Serial.print("The angle from the first sensor is: ");
  Serial.println(data1);
  driver.send((uint8_t *)&data1, sizeof(data1));
  driver.waitPacketSent();
//  delay(100);
  
val2=mapf(sum2,zeroVal2,ninetyVal2,0,90.0);
  data2=val2;
  Serial.print("The angle from the second sensor is: ");
  Serial.println(data2);
  driver.send((uint8_t *)&data2, sizeof(data2));
  driver.waitPacketSent();
//  delay(100);
  
val3=mapf(sum3,zeroVal3,ninetyVal3,0,90.0);
  data3=val3;
  Serial.print("The angle from the third sensor is: ");
  Serial.println(data3);
  driver.send((uint8_t *)&data3, sizeof(data3));
  driver.waitPacketSent();
//  delay(100);
  
val4=mapf(sum4,zeroVal4,ninetyVal4,0,90.0);
  data4=val4;
  Serial.print("The angle from the fourth sensor is: ");
  Serial.println(data4);
  driver.send((uint8_t *)&data4, sizeof(data4));
  driver.waitPacketSent();
//  delay(100);
  
val5=mapf(sum5,zeroVal5,ninetyVal5,0,90.0);
  data5=val5;
  Serial.print("The angle from the fifth sensor is: ");
  Serial.println(data5);
  driver.send((uint8_t *)&data5, sizeof(data5));
  driver.waitPacketSent();
//  delay(100);
  
val6=mapf(sum6,zeroVal6,ninetyVal6,0,90.0);
  data6=val6;
  Serial.print("The angle from the sixth sensor is: ");
  Serial.println(data6);
  driver.send((uint8_t *)&data6, sizeof(data6));
  driver.waitPacketSent();
//  delay(100);
  
val7=mapf(sum7,zeroVal7,ninetyVal7,0,90.0);
  data7=val7;
  Serial.print("The angle from the seventh sensor is: ");
  Serial.println(data7);
  driver.send((uint8_t *)&data7, sizeof(data7));
  driver.waitPacketSent();
//  delay(100);
  
val8=mapf(sum8,zeroVal8,ninetyVal8,0,90.0);
  data8=val8;
  Serial.print("The angle from the eigth sensor is: ");
  Serial.println(data8);
  driver.send((uint8_t *)&data8, sizeof(data8));
  driver.waitPacketSent();
//  delay(100);
  
val9=mapf(sum9,zeroVal9,ninetyVal9,0,90.0);
  data9=val9;
  Serial.print("The angle from the ninth sensor is: ");
  Serial.println(data9);
  driver.send((uint8_t *)&data9, sizeof(data9));
  driver.waitPacketSent();
//  delay(100);

  delay(5);
    }
    }
  }
}
}


float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
