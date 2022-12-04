#include <LSM9DS1_Registers.h>
#include <LSM9DS1_Types.h>
#include <SparkFunLSM9DS1.h>
#include <Wire.h>
#include <SPI.h>
#include <stdio.h> 
#include <SparkFunLSM9DS1.h>
LSM9DS1 imu[16];

#define TCAADDR  0x70 
#define TCAADDR2 0x71   //enable the A0 Pin on the second TCA
#define num 16     //the number of the IMUs
//int t;
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
#define PRINT_SPEED 5 // 100 ms between prints
static unsigned long lastPrint = 0; // Keep track of print time
#define DECLINATION -8.58 // Declination (degrees) in Boulder, CO.
void printGyro1();
void printAccel1();
void printMag1();
void printAttitude(float ax, float ay, float az, float mx, float my, float mz);


void setup() {

  Serial.begin(115200);
  Wire.begin();
for(int n=0;n<num;n++){
  
  tcaselect(n);
  //may need add some initialization
 // imu[n].begin(TCAADDR); 
 // SDO_XM and SDO_G are both pulled high, so our addresses are:
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
 }

void loop() {
int t=0;
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
    Serial.println();
    lastPrint = millis(); // Update lastPrint time
  }
 
 delay(5); 
}
}


void printGyro1(int select)
{
//Serial.print("G: ");
#ifdef PRINT_CALCULATED
  Serial.print(imu[select].calcGyro(imu[select].gx), 2);
  Serial.print(", ");
  Serial.print(imu[select].calcGyro(imu[select].gy), 2);
  Serial.print(", ");
  Serial.print(imu[select].calcGyro(imu[select].gz), 2);
  Serial.print(" deg/s ");
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
  // Now we can use the ax, ay, and az variables as we please.
  // Either print them as raw ADC values, or calculated in g's.
//Serial.print("1A: ");
#ifdef PRINT_CALCULATED
  Serial.print(imu[select].calcAccel(imu[select].ax), 2);
  Serial.print(", ");
  Serial.print(imu[select].calcAccel(imu[select].ay), 2);
  Serial.print(", ");
  Serial.print(imu[select].calcAccel(imu[select].az), 2);
  Serial.print(" g ");
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
  // Now we can use the mx, my, and mz variables as we please.
  // Either print them as raw ADC values, or calculated in Gauss.
 // Serial.print("1M: ");
#ifdef PRINT_CALCULATED
  Serial.print(imu[select].calcMag(imu[select].mx), 2);
  Serial.print(", ");
  Serial.print(imu[select].calcMag(imu[select].my), 2);
  Serial.print(", ");
  Serial.print(imu[select].calcMag(imu[select].mz), 2);
  Serial.print(" gauss ");
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
  Serial.print(", ");
  Serial.print(roll, 2);
//  Serial.print("Heading: ");
  Serial.println(heading, 2);
}
