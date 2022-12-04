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
#define num 16          //the number of the IMUs
void tcaselect (uint8_t i) { 

  if (i > num-1 ) return ;
  if (i<8){
  Wire.beginTransmission(TCAADDR); 
  Wire.write(1 << i); 
  Wire.endTransmission(); 
  Serial.println(i);
  }
  
  if (7<i<16){
  Wire.beginTransmission(TCAADDR2); 
  Wire.write(1 << i); 
  Wire.endTransmission(); 
  Serial.println(i);
  }
}


////////////////////////////
// Sketch Output Settings //
////////////////////////////
#define PRINT_CALCULATED
#define PRINT_SPEED 250 // 250 ms between prints
static unsigned long lastPrint = 0; // Keep track of print time
#define DECLINATION -8.58 // Declination (degrees) in Boulder, CO.
void printGyro1();
void printAccel1();
void printMag1();
void printAttitude(float ax, float ay, float az, float mx, float my, float mz);

int t=0;

void setup() {
  
  Serial.begin(115200);
  Wire.begin();
for(int n=0;n<num;n++){

  tcaselect(n);
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
for(;t<num;t++)
{
  tcaselect(t);
if ( imu[t].gyroAvailable() )
  {
    // To read from the gyroscope,  first call the
    // readGyro() function. When it exits, it'll update the
    // gx, gy, and gz variables with the most current data.
    imu[t].readGyro();
  }
  if ( imu[t].accelAvailable() )
  {
    // To read from the accelerometer, first call the
    // readAccel() function. When it exits, it'll update the
    // ax, ay, and az variables with the most current data.
    imu[t].readAccel();
  }
  if ( imu[t].magAvailable() )
  {
    // To read from the magnetometer, first call the
    // readMag() function. When it exits, it'll update the
    // mx, my, and mz variables with the most current data.
    imu[t].readMag();
  }

  if ((lastPrint + PRINT_SPEED) < millis())
  {
    printGyro1();  // Print "G: gx, gy, gz"
    printAccel1(); // Print "A: ax, ay, az"
    printMag1();   // Print "M: mx, my, mz"
    // Print the heading and orientation for fun!
    // Call print attitude. The LSM9DS1's mag x and y
    // axes are opposite to the accelerometer, so my, mx are
    // substituted for each other.
    printAttitude(imu[t].ax, imu[t].ay, imu[t].az,
                  -imu[t].my, -imu[t].mx, imu[t].mz);
    Serial.println();

    lastPrint = millis(); // Update lastPrint time
  }
 
delay(400); 
}

}

void printGyro1()
{
  // Now we can use the gx, gy, and gz variables as we please.
  // Either print them as raw ADC values, or calculated in DPS.
  Serial.print("G: ");
#ifdef PRINT_CALCULATED
  // If you want to print calculated values, you can use the
  // calcGyro helper function to convert a raw ADC value to
  // DPS. Give the function the value that you want to convert.
  Serial.print(imu[t].calcGyro(imu[t].gx), 2);
  Serial.print(", ");
  Serial.print(imu[t].calcGyro(imu[t].gy), 2);
  Serial.print(", ");
  Serial.print(imu[t].calcGyro(imu[t].gz), 2);
  Serial.println(" deg/s");
#elif defined PRINT_RAW
  Serial.print(imu[t].gx);
  Serial.print(", ");
  Serial.print(imup[t].gy);
  Serial.print(", ");
  Serial.println(imu[t].gz);
#endif
}

void printAccel1()
{
  // Now we can use the ax, ay, and az variables as we please.
  // Either print them as raw ADC values, or calculated in g's.
  Serial.print("1A: ");
#ifdef PRINT_CALCULATED
  // If you want to print calculated values, you can use the
  // calcAccel helper function to convert a raw ADC value to
  // g's. Give the function the value that you want to convert.
  Serial.print(imu[t].calcAccel(imu[t].ax), 2);
  Serial.print(", ");
  Serial.print(imu[t].calcAccel(imu[t].ay), 2);
  Serial.print(", ");
  Serial.print(imu[t].calcAccel(imu[t].az), 2);
  Serial.println(" g");
#elif defined PRINT_RAW
  Serial.print(imu[t].ax);
  Serial.print(", ");
  Serial.print(imu[t].ay);
  Serial.print(", ");
  Serial.println(imu[t].az);
#endif

}

void printMag1()
{
  // Now we can use the mx, my, and mz variables as we please.
  // Either print them as raw ADC values, or calculated in Gauss.
  Serial.print("1M: ");
#ifdef PRINT_CALCULATED
  // If you want to print calculated values, you can use the
  // calcMag helper function to convert a raw ADC value to
  // Gauss. Give the function the value that you want to convert.
  Serial.print(imu[t].calcMag(imu[t].mx), 2);
  Serial.print(", ");
  Serial.print(imu[t].calcMag(imu[t].my), 2);
  Serial.print(", ");
  Serial.print(imu[t].calcMag(imu[t].mz), 2);
  Serial.println(" gauss");
#elif defined PRINT_RAW
  Serial.print(imu[t].mx);
  Serial.print(", ");
  Serial.print(imu[t].my);
  Serial.print(", ");
  Serial.println(imu[t].mz);
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

  Serial.print("Pitch, Roll: ");
  Serial.print(pitch, 2);
  Serial.print(", ");
  Serial.println(roll, 2);
  Serial.print("Heading: "); Serial.println(heading, 2);
}
