
/**************************************************************************************************************
  Example for connecting four BME280 sensors to an I2C multiplexer (TCA9548A)
  written by Thiago Barros for BlueDot UG (haftungsbeschränkt)
  BSD License

  This sketch was written for the Bosch Sensor BME280 and the Chip TCA9548A from Texas Instrument.
  The BME280 is a MEMS device for measuring temperature, humidity and atmospheric pressure.
  The TCA9548A is an 8-Channel I2C Switch with reset function.
  For more technical information on the BME280 and TCA9548A, please go to ------> http://www.bluedot.space

 **************************************************************************************************************/


#include <Wire.h>
//#include <avr/wdt.h>
#include "BlueDot_BME280.h"
BlueDot_BME280 bme280_0;
BlueDot_BME280 bme280_1;
BlueDot_BME280 bme280_2;
BlueDot_BME280 bme280_3;


#define TCAADDR 0x70
void tcaselect(uint8_t i) {
  if (i > 7) return;
 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}

void setup() {
  Serial.begin(9600);
  Wire.begin();

  Serial.println(F("##############################"));            
  Serial.println(F("Starting Initialization"));
  Serial.println(F("##############################"));       
 

  //*************INITIALIZING FIRST SENSOR*******************************   
  tcaselect(0);
  bme280_0.parameter.communication = 0;
  bme280_0.parameter.I2CAddress = 0x77;                    //Choose I2C Address
  bme280_0.parameter.sensorMode = 0b11;                    //Choose sensor mode
  bme280_0.parameter.IIRfilter = 0b100;                    //Setup for IIR Filter  
  bme280_0.parameter.humidOversampling = 0b101;            //Setup Humidity Oversampling
  bme280_0.parameter.tempOversampling = 0b101;             //Setup Temperature Ovesampling  
  bme280_0.parameter.pressOversampling = 0b101;            //Setup Pressure Oversampling  
  if (bme280_0.init() != 0x60)
  { Serial.print(F("BME280 Nr.1 detected?\t")); Serial.println(F("No"));}
  else
  { Serial.print(F("BME280 Nr.1 detected?\t")); Serial.println(F("Yes"));}
  //**********************************************************************

  //*************INITIALIZING SECOND SENSOR*******************************  
  tcaselect(1);  
  bme280_1.parameter.communication = 0;
  bme280_1.parameter.I2CAddress = 0x77;                    //Choose I2C Address
  bme280_1.parameter.sensorMode = 0b11;                    //Choose sensor mode
  bme280_1.parameter.IIRfilter = 0b100;                    //Setup for IIR Filter  
  bme280_1.parameter.humidOversampling = 0b101;            //Setup Humidity Oversampling
  bme280_1.parameter.tempOversampling = 0b101;             //Setup Temperature Ovesampling  
  bme280_1.parameter.pressOversampling = 0b101;            //Setup Pressure Oversampling   
  if (bme280_1.init() != 0x60)
  { Serial.print(F("BME280 Nr.2 detected?\t")); Serial.println(F("No"));}
  else
  { Serial.print(F("BME280 Nr.2 detected?\t")); Serial.println(F("Yes"));}
  //*********************************************************************      

 //*************INITIALIZING THIRD SENSOR********************************    
  tcaselect(2);  
  bme280_2.parameter.communication = 0;
  bme280_2.parameter.I2CAddress = 0x77;                    //Choose I2C Address
  bme280_2.parameter.sensorMode = 0b11;                    //Choose sensor mode
  bme280_2.parameter.IIRfilter = 0b100;                    //Setup for IIR Filter  
  bme280_2.parameter.humidOversampling = 0b101;            //Setup Humidity Oversampling
  bme280_2.parameter.tempOversampling = 0b101;             //Setup Temperature Ovesampling  
  bme280_2.parameter.pressOversampling = 0b101;            //Setup Pressure Oversampling 
  
  if (bme280_2.init() != 0x60)
  { Serial.print(F("BME280 Nr.3 detected?\t")); Serial.println(F("No"));}
  else
  { Serial.print(F("BME280 Nr.3 detected?\t")); Serial.println(F("Yes"));}
  //**********************************************************************
  
  //*************INITIALIZING FOURTH SENSOR********************************    
  tcaselect(3);
  bme280_3.parameter.communication = 0;
  bme280_3.parameter.I2CAddress = 0x77;                    //Choose I2C Address
  bme280_3.parameter.sensorMode = 0b11;                    //Choose sensor mode
  bme280_3.parameter.IIRfilter = 0b100;                    //Setup for IIR Filter  
  bme280_3.parameter.humidOversampling = 0b101;            //Setup Humidity Oversampling
  bme280_3.parameter.tempOversampling = 0b101;             //Setup Temperature Ovesampling  
  bme280_3.parameter.pressOversampling = 0b101;            //Setup Pressure Oversampling
  if (bme280_3.init() != 0x60)
  { Serial.print(F("BME280 Nr.4 detected?\t")); Serial.println(F("No"));}
  else
  { Serial.print(F("BME280 Nr.4 detected?\t")); Serial.println(F("Yes"));}
  //**********************************************************************


  //**********************************************************************  
  Serial.println();
  Serial.println(F("##############################"));            //30-times #
  Serial.println(F("Initialization Finished"));
  Serial.println(F("##############################"));            //30-times # 
  Serial.println();
  Serial.println();
}
  //*********************************************************************
  //*************NOW LET'S START MEASURING*******************************
void loop() 
{ 
   tcaselect(0);
   Serial.print(F("Duration1[s]=\t"));
   Serial.print(float(millis())/1000);
   Serial.print("\t");
   Serial.print(F("Temperature1[C]=\t"));   
   Serial.print(bme280_0.readTempC());
   Serial.print("\t");
   Serial.print(F("Humidity1[%]=\t"));  
   Serial.print(bme280_0.readHumidity());
   Serial.print("\t");
   Serial.print(F("Pressure1[Pa]=\t")); 
   Serial.print(bme280_0.readPressure());
   Serial.print("\t");
   Serial.println();   
 
   tcaselect(1);
   Serial.print(F("Duration2[s]=\t"));
   Serial.print(float(millis())/1000);
   Serial.print("\t");
   Serial.print(F("Temperature2[C]=\t"));
   Serial.print(bme280_1.readTempC());
   Serial.print("\t");
   Serial.print(F("Humidity2[%]=\t")); 
   Serial.print(bme280_1.readHumidity());
   Serial.print("\t");
   Serial.print(F("Pressure2[Pa]=\t")); 
   Serial.print(bme280_1.readPressure());
   Serial.print("\t");
   Serial.println();

   tcaselect(2);
   Serial.print(F("Duration3[s]=\t"));
   Serial.print(float(millis())/1000);
   Serial.print("\t");
   Serial.print(F("Temperature3[C]=\t"));
   Serial.print(bme280_2.readTempC());
   Serial.print("\t");
   Serial.print(F("Humidity3[%]=\t")); 
   Serial.print(bme280_2.readHumidity());
   Serial.print("\t");
   Serial.print(F("Pressure3[Pa]=\t")); 
   Serial.print(bme280_2.readPressure());
   Serial.print("\t");
   Serial.println();
   
   tcaselect(3);
   Serial.print(F("Duration4[s]=\t"));
   Serial.print(float(millis())/1000);
   Serial.print("\t");
   Serial.print(F("Temperature4[C]=\t"));
   Serial.print(bme280_3.readTempC());
   Serial.print("\t");
   Serial.print(F("Humidity4[%]=\t")); 
   Serial.print(bme280_3.readHumidity());
   Serial.print("\t");
   Serial.print(F("Pressure4[Pa]=\t")); 
   Serial.print(bme280_3.readPressure());
   Serial.print("\t");
   Serial.println();

   
   Serial.println();
   Serial.println();

   delay(1000);   
 
}
