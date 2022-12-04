#include <RH_ASK.h>
#include <SPI.h>
#include <LSM9DS1_Registers.h>
#include <LSM9DS1_Types.h>
#include <SparkFunLSM9DS1.h>
#include <Wire.h>
#include <stdio.h> 

LSM9DS1 imu[16];

#define TCAADDR  0x70 
#define TCAADDR2 0x71   //enable the A0 Pin on the second TCA
#define num 16     //the number of the IMUs

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

float imu_gx[16];
float imu_gy[16];
float imu_gz[16];
float imu_ax[16];
float imu_ay[16];
float imu_az[16];
float imu_mx[16];
float imu_my[16];
float imu_mz[16];


int loops=100;


int initalDelay=500;
int delayTime=50;     //100;


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

////////////////use to select the tca multiplexers used///////////////////

void tcaselect (uint8_t i) { 

  if (i > num-1 ) return ;
  if (i<8){
  Wire.beginTransmission(TCAADDR); 
  Wire.write(1 << i); 
  Wire.endTransmission(); 
  Serial.print(i);
  Serial.print(": ");
  }
  
  if (7<i<16){
  Wire.beginTransmission(TCAADDR2); 
  Wire.write(1 << i); 
  Wire.endTransmission(); 
  Serial.print(i);
  Serial.print(": ");
  }
}

////////////////////////////
// Sketch Output Settings //
////////////////////////////
#define PRINT_CALCULATED  // PRINT_RAW
#define PRINT_SPEED 1  //5 // 100 ms between prints
static unsigned long lastPrint = 0;   // Keep track of print time
#define DECLINATION -8.58             // Declination (degrees) in Boulder, CO.
void printGyro1();
void printAccel1();
void printMag1();
void printAttitude(float ax, float ay, float az, float mx, float my, float mz);


void setup() {
 Serial.begin(2000000);
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

Wire.begin();
for(int n=0;n<num;n++){
  tcaselect(n);
  #define LSM9DS1_M   0x1E // Would be 0x1C if SDO_M is LOW
  #define LSM9DS1_AG  0x6B // Would be 0x6A if SDO_AG is LOW
imu[n].settings.device.commInterface = IMU_MODE_I2C; // Set mode to I2C
imu[n].settings.device.mAddress = LSM9DS1_M; // Set mag address to 0x1E
imu[n].settings.device.agAddress = LSM9DS1_AG; // Set ag address to 0x6B

  if (imu[n].begin() == false) // with no arguments, this uses default addresses (AG:0x6B, M:0x1E) and i2c port (Wire).
  {
    Serial.println("Failed to communicate with the LSM9DS1.");
    Serial.println("Double-check wiring.");
    Serial.println("Default settings in this sketch will " \
                   "work for an out of the box LSM9DS1 " \
                   "Breakout, but may need to be modified " \
                   "if the board jumpers are.");
    while (1);
  }
 }

  driver.init();
  if (!driver.init()){
    Serial.println("Driver init failed");
  }
    Serial.println(""); 
   Serial.println("Push the yellow button to check");
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
    delay(delayTime);
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
   Serial.println("");  
   Serial.println("The zero angle value has been set.");
   Serial.println("Push the green button to check");
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
    digitalWrite(ledPin,LOW); 
  Serial.println("Push the two buttons to begin.");
 }
}
////////////////////////////////////////////////////////////////////////////////
/***************** two buttons been pushed,start collecting data **************/
////////////////////////////////////////////////////////////////////////////////

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

    sum1=sum1 + (analogRead(sen1)*VCC)/1023.0;
    sum2=sum2 + (analogRead(sen2)*VCC)/1023.0;
    sum3=sum3 + (analogRead(sen3)*VCC)/1023.0;
    sum4=sum4 + (analogRead(sen4)*VCC)/1023.0;
    sum5=sum5 + (analogRead(sen5)*VCC)/1023.0;
    sum6=sum6 + (analogRead(sen6)*VCC)/1023.0;
    sum7=sum7 + (analogRead(sen7)*VCC)/1023.0;
    sum8=sum8 + (analogRead(sen8)*VCC)/1023.0;
    sum9=sum9 + (analogRead(sen9)*VCC)/1023.0;


/////////////////////////// IMU data read and send////////////////////////

  tcaselect(0);
if ( imu[0].gyroAvailable() )
    imu[0].readGyro();
  if ( imu[0].accelAvailable() )
    imu[0].readAccel();
  if ( imu[0].magAvailable() )
    imu[0].readMag();   //Read and print the data

  if ((lastPrint + PRINT_SPEED) < millis())
  {
    printGyro1(0);  // Print "G: gx, gy, gz"
    printAccel1(0); // Print "A: ax, ay, az"
    printMag1(0);   // Print "M: mx, my, mz"
    printAttitude(imu[0].ax, imu[0].ay, imu[0].az,
                  -imu[0].my, -imu[0].mx, imu[0].mz);
    Serial.println();
    lastPrint = millis(); // Update lastPrint time
  }

  tcaselect(1);
if ( imu[1].gyroAvailable() )
    imu[1].readGyro();
  if ( imu[1].accelAvailable() )
    imu[1].readAccel();
  if ( imu[1].magAvailable() )
    imu[1].readMag();

  if ((lastPrint + PRINT_SPEED) < millis())
  {
    printGyro1(1);  // Print "G: gx, gy, gz"
    printAccel1(1); // Print "A: ax, ay, az"
    printMag1(1);   // Print "M: mx, my, mz"
    printAttitude(imu[1].ax, imu[1].ay, imu[1].az,
                  -imu[1].my, -imu[1].mx, imu[1].mz);
    Serial.println();
    lastPrint = millis(); // Update lastPrint time
  }

  tcaselect(2);
if ( imu[2].gyroAvailable() )
    imu[2].readGyro();
  if ( imu[2].accelAvailable() )
    imu[2].readAccel();
  if ( imu[2].magAvailable() )
    imu[2].readMag();

  if ((lastPrint + PRINT_SPEED) < millis())
  {
    printGyro1(2);  // Print "G: gx, gy, gz"
    printAccel1(2); // Print "A: ax, ay, az"
    printMag1(2);   // Print "M: mx, my, mz"
    printAttitude(imu[2].ax, imu[2].ay, imu[2].az,
                  -imu[2].my, -imu[2].mx, imu[2].mz);
    Serial.println();
    lastPrint = millis(); // Update lastPrint time
  }


    tcaselect(3);
if ( imu[3].gyroAvailable() )
    imu[3].readGyro();
  if ( imu[3].accelAvailable() )
    imu[3].readAccel();
  if ( imu[3].magAvailable() )
    imu[3].readMag();

  if ((lastPrint + PRINT_SPEED) < millis())
  {
    printGyro1(3);  // Print "G: gx, gy, gz"
    printAccel1(3); // Print "A: ax, ay, az"
    printMag1(3);   // Print "M: mx, my, mz"
    printAttitude(imu[3].ax, imu[3].ay, imu[3].az,
                  -imu[3].my, -imu[3].mx, imu[3].mz);
    Serial.println();
    lastPrint = millis(); // Update lastPrint time
  }


    tcaselect(4);
if ( imu[4].gyroAvailable() )
    imu[4].readGyro();
  if ( imu[4].accelAvailable() )
    imu[4].readAccel();
  if ( imu[4].magAvailable() )
    imu[4].readMag();

  if ((lastPrint + PRINT_SPEED) < millis())
  {
    printGyro1(4);  // Print "G: gx, gy, gz"
    printAccel1(4); // Print "A: ax, ay, az"
    printMag1(4);   // Print "M: mx, my, mz"
    printAttitude(imu[4].ax, imu[4].ay, imu[4].az,
                  -imu[4].my, -imu[4].mx, imu[4].mz);
    Serial.println();
    lastPrint = millis(); // Update lastPrint time
  }


    tcaselect(5);
if ( imu[5].gyroAvailable() )
    imu[5].readGyro();
  if ( imu[5].accelAvailable() )
    imu[5].readAccel();
  if ( imu[5].magAvailable() )
    imu[5].readMag();

  if ((lastPrint + PRINT_SPEED) < millis())
  {
    printGyro1(5);  // Print "G: gx, gy, gz"
    printAccel1(5); // Print "A: ax, ay, az"
    printMag1(5);   // Print "M: mx, my, mz"
    printAttitude(imu[5].ax, imu[5].ay, imu[5].az,
                  -imu[5].my, -imu[5].mx, imu[5].mz);
    Serial.println();
    lastPrint = millis(); // Update lastPrint time
  }


    tcaselect(6);
if ( imu[6].gyroAvailable() )
    imu[6].readGyro();
  if ( imu[6].accelAvailable() )
    imu[6].readAccel();
  if ( imu[6].magAvailable() )
    imu[6].readMag();

  if ((lastPrint + PRINT_SPEED) < millis())
  {
    printGyro1(6);  // Print "G: gx, gy, gz"
    printAccel1(6); // Print "A: ax, ay, az"
    printMag1(6);   // Print "M: mx, my, mz"
    printAttitude(imu[6].ax, imu[6].ay, imu[6].az,
                  -imu[6].my, -imu[6].mx, imu[6].mz);
    Serial.println();
    lastPrint = millis(); // Update lastPrint time
  }

    tcaselect(7);
if ( imu[7].gyroAvailable() )
    imu[7].readGyro();
  if ( imu[7].accelAvailable() )
    imu[7].readAccel();
  if ( imu[7].magAvailable() )
    imu[7].readMag();

  if ((lastPrint + PRINT_SPEED) < millis())
  {
    printGyro1(7);  // Print "G: gx, gy, gz"
    printAccel1(7); // Print "A: ax, ay, az"
    printMag1(7);   // Print "M: mx, my, mz"
    printAttitude(imu[7].ax, imu[7].ay, imu[7].az,
                  -imu[7].my, -imu[7].mx, imu[7].mz);
    Serial.println();
    lastPrint = millis(); // Update lastPrint time
  }


    tcaselect(8);
if ( imu[8].gyroAvailable() )
    imu[8].readGyro();
  if ( imu[8].accelAvailable() )
    imu[8].readAccel();
  if ( imu[8].magAvailable() )
    imu[8].readMag();

  if ((lastPrint + PRINT_SPEED) < millis())
  {
    printGyro1(8);  // Print "G: gx, gy, gz"
    printAccel1(8); // Print "A: ax, ay, az"
    printMag1(8);   // Print "M: mx, my, mz"
    printAttitude(imu[8].ax, imu[8].ay, imu[8].az,
                  -imu[8].my, -imu[8].mx, imu[8].mz);
    Serial.println();
    lastPrint = millis(); // Update lastPrint time
  }


    tcaselect(9);
if ( imu[9].gyroAvailable() )
    imu[9].readGyro();
  if ( imu[9].accelAvailable() )
    imu[9].readAccel();
  if ( imu[9].magAvailable() )
    imu[9].readMag();

  if ((lastPrint + PRINT_SPEED) < millis())
  {
    printGyro1(9);  // Print "G: gx, gy, gz"
    printAccel1(9); // Print "A: ax, ay, az"
    printMag1(9);   // Print "M: mx, my, mz"
    printAttitude(imu[9].ax, imu[9].ay, imu[9].az,
                  -imu[9].my, -imu[9].mx, imu[9].mz);
    Serial.println();
    lastPrint = millis(); // Update lastPrint time
  }


    tcaselect(10);
if ( imu[10].gyroAvailable() )
    imu[10].readGyro();
  if ( imu[10].accelAvailable() )
    imu[10].readAccel();
  if ( imu[10].magAvailable() )
    imu[10].readMag();

  if ((lastPrint + PRINT_SPEED) < millis())
  {
    printGyro1(10);  // Print "G: gx, gy, gz"
    printAccel1(10); // Print "A: ax, ay, az"
    printMag1(10);   // Print "M: mx, my, mz"
    printAttitude(imu[10].ax, imu[10].ay, imu[10].az,
                  -imu[10].my, -imu[10].mx, imu[10].mz);
    Serial.println();
    lastPrint = millis(); // Update lastPrint time
  }


    tcaselect(11);
if ( imu[11].gyroAvailable() )
    imu[11].readGyro();
  if ( imu[11].accelAvailable() )
    imu[11].readAccel();
  if ( imu[11].magAvailable() )
    imu[11].readMag();

  if ((lastPrint + PRINT_SPEED) < millis())
  {
    printGyro1(11);  // Print "G: gx, gy, gz"
    printAccel1(11); // Print "A: ax, ay, az"
    printMag1(11);   // Print "M: mx, my, mz"
    printAttitude(imu[11].ax, imu[11].ay, imu[11].az,
                  -imu[11].my, -imu[11].mx, imu[11].mz);
    Serial.println();
    lastPrint = millis(); // Update lastPrint time
  }


    tcaselect(12);
if ( imu[12].gyroAvailable() )
    imu[12].readGyro();
  if ( imu[12].accelAvailable() )
    imu[12].readAccel();
  if ( imu[12].magAvailable() )
    imu[12].readMag();

  if ((lastPrint + PRINT_SPEED) < millis())
  {
    printGyro1(12);  // Print "G: gx, gy, gz"
    printAccel1(12); // Print "A: ax, ay, az"
    printMag1(12);   // Print "M: mx, my, mz"
    printAttitude(imu[12].ax, imu[12].ay, imu[12].az,
                  -imu[12].my, -imu[12].mx, imu[12].mz);
    Serial.println();
    lastPrint = millis(); // Update lastPrint time
  }

    tcaselect(13);
if ( imu[13].gyroAvailable() )
    imu[13].readGyro();
  if ( imu[13].accelAvailable() )
    imu[13].readAccel();
  if ( imu[13].magAvailable() )
    imu[13].readMag();

  if ((lastPrint + PRINT_SPEED) < millis())
  {
    printGyro1(13);  // Print "G: gx, gy, gz"
    printAccel1(13); // Print "A: ax, ay, az"
    printMag1(13);   // Print "M: mx, my, mz"
    printAttitude(imu[13].ax, imu[13].ay, imu[13].az,
                  -imu[13].my, -imu[13].mx, imu[13].mz);
    Serial.println();
    lastPrint = millis(); // Update lastPrint time
  }


    tcaselect(14);
if ( imu[14].gyroAvailable() )
    imu[14].readGyro();
  if ( imu[14].accelAvailable() )
    imu[14].readAccel();
  if ( imu[14].magAvailable() )
    imu[14].readMag();

  if ((lastPrint + PRINT_SPEED) < millis())
  {
    printGyro1(14);  // Print "G: gx, gy, gz"
    printAccel1(14); // Print "A: ax, ay, az"
    printMag1(14);   // Print "M: mx, my, mz"
    printAttitude(imu[14].ax, imu[14].ay, imu[14].az,
                  -imu[14].my, -imu[14].mx, imu[14].mz);
    Serial.println();
    lastPrint = millis(); // Update lastPrint time
  }


    tcaselect(15);
if ( imu[15].gyroAvailable() )
    imu[15].readGyro();
  if ( imu[15].accelAvailable() )
    imu[15].readAccel();
  if ( imu[15].magAvailable() )
    imu[15].readMag();

  if ((lastPrint + PRINT_SPEED) < millis())
  {
    printGyro1(15);  // Print "G: gx, gy, gz"
    printAccel1(15); // Print "A: ax, ay, az"
    printMag1(15);   // Print "M: mx, my, mz"
    printAttitude(imu[15].ax, imu[15].ay, imu[15].az,
                  -imu[15].my, -imu[15].mx, imu[15].mz);
    Serial.println();
    lastPrint = millis(); // Update lastPrint time
  }


/* int t=0;

for(;t<num;t++){
  tcaselect(t);
if ( imu[t].gyroAvailable() )
    imu[t].readGyro();
  if ( imu[t].accelAvailable() )
    imu[t].readAccel();
  if ( imu[t].magAvailable() )
    imu[t].readMag();

  if ((lastPrint + PRINT_SPEED) < millis())
  {
    printGyro1(t);  // Print "G: gx, gy, gz"
    printAccel1(t); // Print "A: ax, ay, az"
    printMag1(t);   // Print "M: mx, my, mz"
    printAttitude(imu[t].ax, imu[t].ay, imu[t].az,
                  -imu[t].my, -imu[t].mx, imu[t].mz);
//    Serial.println();
    lastPrint = millis(); // Update lastPrint time
  }
 
 // delay(5); 
} */

////////////// Serial print and send data to the receiver ///////////

val1=mapf(sum1,zeroVal1,ninetyVal1,0,90.0);
  data1=val1;
  //Serial.print("The angle from the first sensor is: ");
  Serial.print(data1);
  Serial.print(" ");
  
val2=mapf(sum2,zeroVal2,ninetyVal2,0,90.0);
  data2=val2;
  //Serial.print("The angle from the second sensor is: ");
  Serial.print(data2);
  Serial.print(" ");
val3=mapf(sum3,zeroVal3,ninetyVal3,0,90.0);
  data3=val3;
  //Serial.print("The angle from the third sensor is: ");
  Serial.print(data3);
 Serial.print(" "); 
val4=mapf(sum4,zeroVal4,ninetyVal4,0,90.0);
  data4=val4;
  //Serial.print("The angle from the fourth sensor is: ");
  Serial.print(data4);
  Serial.print(" ");
val5=mapf(sum5,zeroVal5,ninetyVal5,0,90.0);
  data5=val5;
 // Serial.print("The angle from the fifth sensor is: ");
  Serial.print(data5);
  Serial.print(" ");
val6=mapf(sum6,zeroVal6,ninetyVal6,0,90.0);
  data6=val6;
  //Serial.print("The angle from the sixth sensor is: ");
  Serial.print(data6);
  Serial.print(" ");
val7=mapf(sum7,zeroVal7,ninetyVal7,0,90.0);
  data7=val7;
  //Serial.print("The angle from the seventh sensor is: ");
  Serial.print(data7);
  Serial.print(" ");
val8=mapf(sum8,zeroVal8,ninetyVal8,0,90.0);
  data8=val8;
  //Serial.print("The angle from the eigth sensor is: ");
  Serial.print(data8);
  Serial.print(" ");
val9=mapf(sum9,zeroVal9,ninetyVal9,0,90.0);
  data9=val9;
  //Serial.print("The angle from the ninth sensor is: ");
  Serial.print(data9);
  Serial.println(" ");
//delay(5);
    }
    }
  }
}
}


float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

///////////////// functions for the IMUs /////////////////////////
//more details:
//https://learn.sparkfun.com/tutorials/9dof-sensor-stick-hookup-guide/all


void printGyro1(int select)
{
//Serial.print("G: ");
#ifdef PRINT_CALCULATED   //use these datas

imu_gx[select]=imu[select].calcGyro(imu[select].gx);
imu_gy[select]=imu[select].calcGyro(imu[select].gy);
imu_gz[select]=imu[select].calcGyro(imu[select].gz);

  Serial.print(imu_gx[select], 2);
  Serial.print(" ");
  Serial.print(imu_gy[select], 2);
  Serial.print(" ");
  Serial.print(imu_gz[select], 2);
  Serial.print(" ");
//  Serial.print(" deg/s ");


#elif defined PRINT_RAW
  Serial.print(imu[select].gx);
  Serial.print(", ");
  Serial.print(imup[select].gy);
  Serial.print(", ");
  Serial.print(imu[select].gz);
#endif
}

void printAccel1(int select)
{

#ifdef PRINT_CALCULATED
  Serial.print(imu[select].calcAccel(imu[select].ax), 2);
  Serial.print(" ");
  Serial.print(imu[select].calcAccel(imu[select].ay), 2);
  Serial.print(" ");
  Serial.print(imu[select].calcAccel(imu[select].az), 2);
  Serial.print(" ");
  //Serial.print(" g ");


#elif defined PRINT_RAW
  Serial.print(imu[select].ax);
  Serial.print(", ");
  Serial.print(imu[select].ay);
  Serial.print(", ");
  Serial.print(imu[select].az);
#endif
}

void printMag1(int select)
{
#ifdef PRINT_CALCULATED

  Serial.print(imu[select].calcMag(imu[select].mx), 2);
  Serial.print(" ");
  Serial.print(imu[select].calcMag(imu[select].my), 2);
  Serial.print(" ");
  Serial.print(imu[select].calcMag(imu[select].mz), 2);
  Serial.print(" ");
  //Serial.print(" gauss ");


#elif defined PRINT_RAW
  Serial.print(imu[select].mx);
  Serial.print(", ");
  Serial.print(imu[select].my);
  Serial.print(", ");
  Serial.print(imu[select].mz);
#endif
}

void printAttitude(float ax, float ay, float az, float mx, float my, float mz)
{
  float roll = atan2(ay, az);
  float pitch = atan2(-ax, sqrt(ay * ay + az * az));
  float heading;
  if (my == 0)
    heading = (mx < 0) ? PI : 0;
  else
    heading = atan2(mx, my);

  heading -= DECLINATION * PI / 180;

  if (heading > PI) heading -= (2 * PI);
  else if (heading < -PI) heading += (2 * PI);

  // Convert everything from radians to degrees:
  heading *= 180.0 / PI;
  pitch *= 180.0 / PI;
  roll  *= 180.0 / PI;

//  Serial.print("Pitch, Roll: ");
  Serial.print(pitch, 2);
  Serial.print(" ");
  Serial.print(roll, 2);
//  Serial.print("Heading: ");
  Serial.print(" ");
  Serial.print(heading, 2);
  Serial.print(" ");
}
