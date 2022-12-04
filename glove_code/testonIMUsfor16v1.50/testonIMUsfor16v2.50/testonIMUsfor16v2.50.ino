#include <LSM9DS1_Registers.h>
#include <LSM9DS1_Types.h>
#include <SparkFunLSM9DS1.h>
#include <Wire.h>
#include <SPI.h>
#include <stdio.h> 
#include <SparkFunLSM9DS1.h>
LSM9DS1 imu1;
LSM9DS1 imu2;

#define TCAADDR  0x70 
#define TCAADDR2 0x71   //enable the A0 Pin on the second TCA
//TwoWire i2caddress=0x70;
#define num 8     //the number of the IMUs
int t;
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
#define PRINT_CALCULATED
#define PRINT_SPEED 6 // 100 ms between prints
static unsigned long lastPrint = 0; // Keep track of print time
#define DECLINATION -8.58 // Declination (degrees) in Boulder, CO.
void printGyro1();
void printAccel1();
void printMag1();
void printGyro2();
void printAccel2();
void printMag2();
void printAttitude(float ax, float ay, float az, float mx, float my, float mz);


void setup() {

  Serial.begin(115200);
  Wire.begin();

 //*************INITIALIZING FIRST IMU******************    
  tcaselect(1);
  //need add some initialization
 // imu[n].begin(TCAADDR);
 // SDO_XM and SDO_G are both pulled high, so our addresses are:
#define LSM9DS1_M   0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG  0x6B // Would be 0x6A if SDO_AG is LOW
imu1.settings.device.commInterface = IMU_MODE_I2C; // Set mode to I2C
imu1.settings.device.mAddress = LSM9DS1_M; // Set mag address to 0x1E
imu1.settings.device.agAddress = LSM9DS1_AG; // Set ag address to 0x6B

  if (imu1.begin() == false) // with no arguments, this uses default addresses (AG:0x6B, M:0x1E) and i2c port (Wire).
  {
    Serial.println("Failed to communicate with the LSM9DS1.1");
    Serial.println("Double-check wiring.");
    Serial.println("Default settings in this sketch will " \
                   "work for an out of the box LSM9DS1 " \
                   "Breakout, but may need to be modified " \
                   "if the board jumpers are.");
    while (1);
  }
 //**********************************************************************
 //*************INITIALIZING SECOND IMU******************    
  tcaselect(2);
  //need add some initialization
 // imu[n].begin(TCAADDR);
 // SDO_XM and SDO_G are both pulled high, so our addresses are:
#define LSM9DS1_M   0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG  0x6B // Would be 0x6A if SDO_AG is LOW
imu2.settings.device.commInterface = IMU_MODE_I2C; // Set mode to I2C
imu2.settings.device.mAddress = LSM9DS1_M; // Set mag address to 0x1E
imu2.settings.device.agAddress = LSM9DS1_AG; // Set ag address to 0x6B

  if (imu2.begin() == false) // with no arguments, this uses default addresses (AG:0x6B, M:0x1E) and i2c port (Wire).
  {
    Serial.println("Failed to communicate with the LSM9DS1.2");
    Serial.println("Double-check wiring.");
    Serial.println("Default settings in this sketch will " \
                   "work for an out of the box LSM9DS1 " \
                   "Breakout, but may need to be modified " \
                   "if the board jumpers are.");
    while (1);
  }
 //**********************************************************************


}

void loop() {

  tcaselect(1);
if ( imu1.gyroAvailable() )
    imu1.readGyro();
  if ( imu1.accelAvailable() )
    imu1.readAccel();
  if ( imu1.magAvailable() )
    imu1.readMag();
  if ((lastPrint + PRINT_SPEED) < millis())
  {
    printGyro1();  // Print "G: gx, gy, gz"
    printAccel1(); // Print "A: ax, ay, az"
    printMag1();   // Print "M: mx, my, mz"
    printAttitude(imu1.ax, imu1.ay, imu1.az,
                  -imu1.my, -imu1.mx, imu1.mz);
    Serial.println();
    lastPrint = millis(); // Update lastPrint time
  }
delay(3);
 
 tcaselect(2);
if ( imu2.gyroAvailable() )
    imu2.readGyro();
  if ( imu2.accelAvailable() )
    imu2.readAccel();
  if ( imu2.magAvailable() )
    imu2.readMag();
  if ((lastPrint + PRINT_SPEED) < millis())
  {
    printGyro2();  // Print "G: gx, gy, gz"
    printAccel2(); // Print "A: ax, ay, az"
    printMag2();   // Print "M: mx, my, mz"
    printAttitude(imu2.ax, imu2.ay, imu2.az,
                  -imu2.my, -imu2.mx, imu2.mz);
    Serial.println();
    lastPrint = millis(); // Update lastPrint time
  }
  
delay(3);
}

void printGyro1()
{
#ifdef PRINT_CALCULATED
  Serial.print(imu1.calcGyro(imu1.gx), 2);
  Serial.print(", ");
  Serial.print(imu1.calcGyro(imu1.gy), 2);
  Serial.print(", ");
  Serial.print(imu1.calcGyro(imu1.gz), 2);
  Serial.print(" deg/s ");
#elif defined PRINT_RAW
  Serial.print(imu1.gx);
  Serial.print(", ");
  Serial.print(imu1.gy);
  Serial.print(", ");
  Serial.print(imu1.gz);
#endif
}

void printAccel1()
{
#ifdef PRINT_CALCULATED
  Serial.print(imu1.calcAccel(imu1.ax), 2);
  Serial.print(", ");
  Serial.print(imu1.calcAccel(imu1.ay), 2);
  Serial.print(", ");
  Serial.print(imu1.calcAccel(imu1.az), 2);
  Serial.print(" g ");
#elif defined PRINT_RAW
  Serial.print(imu1.ax);
  Serial.print(", ");
  Serial.print(imu1.ay);
  Serial.print(", ");
  Serial.print(imu1.az);
#endif
}

void printMag1()
{
#ifdef PRINT_CALCULATED
  Serial.print(imu1.calcMag(imu1.mx), 2);
  Serial.print(", ");
  Serial.print(imu1.calcMag(imu1.my), 2);
  Serial.print(", ");
  Serial.print(imu1.calcMag(imu1.mz), 2);
  Serial.print(" gauss ");
#elif defined PRINT_RAW
  Serial.print(imu1.mx);
  Serial.print(", ");
  Serial.print(imu1.my);
  Serial.print(", ");
  Serial.print(imu1.mz);
#endif
}

//////////////////////////////2///////////////////////////////////////

void printGyro2()
{
#ifdef PRINT_CALCULATED
  Serial.print(imu2.calcGyro(imu2.gx), 2);
  Serial.print(", ");
  Serial.print(imu2.calcGyro(imu2.gy), 2);
  Serial.print(", ");
  Serial.print(imu2.calcGyro(imu2.gz), 2);
  Serial.print(" deg/s ");
#elif defined PRINT_RAW
  Serial.print(imu2.gx);
  Serial.print(", ");
  Serial.print(imu2.gy);
  Serial.print(", ");
  Serial.print(imu2.gz);
#endif
}

void printAccel2()
{
#ifdef PRINT_CALCULATED
  Serial.print(imu2.calcAccel(imu2.ax), 2);
  Serial.print(", ");
  Serial.print(imu2.calcAccel(imu2.ay), 2);
  Serial.print(", ");
  Serial.print(imu2.calcAccel(imu2.az), 2);
  Serial.print(" g ");
#elif defined PRINT_RAW
  Serial.print(imu2.ax);
  Serial.print(", ");
  Serial.print(imu2.ay);
  Serial.print(", ");
  Serial.print(imu2.az);
#endif
}

void printMag2()
{
#ifdef PRINT_CALCULATED
  Serial.print(imu2.calcMag(imu2.mx), 2);
  Serial.print(", ");
  Serial.print(imu2.calcMag(imu2.my), 2);
  Serial.print(", ");
  Serial.print(imu2.calcMag(imu2.mz), 2);
  Serial.print(" gauss ");
#elif defined PRINT_RAW
  Serial.print(imu2.mx);
  Serial.print(", ");
  Serial.print(imu2.my);
  Serial.print(", ");
  Serial.print(imu2.mz);
#endif
}
/////////////////////////for universal using ////////////////////// 
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

  Serial.print(pitch, 2);
  Serial.print(", ");
  Serial.print(roll, 2);
  Serial.println(heading, 2);
}
