#include <EnvironmentCalculations.h>

#include "/home/luca/Developer/SailMAVx/Arduino/left_hull/src/MAVlink/PixhawkArduinoMAVLink.h"

#include <Wire.h>
#include <Scheduler.h>
#include <HardwareSerial.h>
#include "/home/luca/Developer/SailMAVx/Arduino/left_hull/src/Helper/Helper.h"

#include "/home/luca/Developer/SailMAVx/Arduino/left_hull/src/TSYS01/Tsys01.h"
#include <BME280I2C.h>

#define powerPin A0
#define TEMPERATURE_MIN 1.0
#define TEMPERATURE_MAX 30.0
//#define DEBUG

BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
BME280::PresUnit presUnit(BME280::PresUnit_Pa);

BME280I2C bme;
//Adafruit_BME280 bme; // I2C

HardwareSerial &hs = Serial1;
PixhawkArduinoMAVLink mav(hs);

Tsys01 sensor;
float temperature_a = 0.0;
float temperature_w = 1.11;
float time_a = 0.0;
float time_w = 0.0;

float hum(NAN), pres(NAN);

void setup() {

  #ifdef DEBUG
    SerialUSB.begin(9600);
    while (!SerialUSB) {;}
    SerialUSB.println("Starting in 1s");
  #endif
  delay(2000);
  SerialUSB.println("Started");

//////////////////   Sensor setup /////////////////

  uint8_t pass = HIGH;
  sensor = Tsys01(TSYS01_I2C, powerPin);
do {
    //sensor.powerOn();
    SerialUSB.println("Testing I2C...");
    //Pass if test is successful
    pass = HIGH;
    //Re-init sensor.
    
    readTemp();

    if(temperature_w > TEMPERATURE_MIN && temperature_w < TEMPERATURE_MAX){
      SerialUSB.println("I2C Communication OK, temperature is in valid range");
    }else{
      SerialUSB.println("I2C Communication OK, Temperature is not in valid range");
      pass = LOW;
    }
    //sensor.powerOff();
  //LED stays low if I2C does not pass
  }while(pass==LOW);
  //}while(pass==HIGH);


////////// Start Atm sensor ///////////

  Wire.begin();

  while(!bme.begin())
  {
    SerialUSB.println("Could not find BME280 sensor!");
    delay(1000);
  }
  switch(bme.chipModel())
  {
     case BME280::ChipModel_BME280:
       SerialUSB.println("Found BME280 sensor! Success.");
       break;
     case BME280::ChipModel_BMP280:
       SerialUSB.println("Found BMP280 sensor! No Humidity available.");
       break;
     default:
       SerialUSB.println("Found UNKNOWN sensor! Error!");
  } 
  //Wire.end();

////////// Start MAVlink ///////////

  while(!mav.begin()){
    SerialUSB.println("Not Connected!");
    delay(1000);
  }
  mav.Stream();
  delay(1000);
  Scheduler.startLoop(mavlink_listen_loop);
}

//**************************************************************************
// main loops, reading sensors and running control
void loop() {

  readTemp();
    
   Wire.begin();
   delay(100);
   bme.read(pres, temperature_a, hum, tempUnit, presUnit);
   Wire.end();
   delay(100);
   
 #ifdef DEBUG
   SerialUSB.println("pres");
   SerialUSB.println(pres);
   //SerialUSB.println(bme.readPressure());
   SerialUSB.println("temp");
   SerialUSB.println(temperature_a);
   //SerialUSB.println(bme.readTemperature());
   SerialUSB.println("hum");
   SerialUSB.println(hum);
   //SerialUSB.println(bme.readHumidity());
 #endif
 
   delay(100);

   time_a = (float)millis(); 
  #ifdef DEBUG
    SerialUSB.print("Temperature is ");
    SerialUSB.print(temperature_w);
    SerialUSB.print(",");
    SerialUSB.println(temperature_a);
  #endif

  mav.set_param("SMV_H2O_T",temperature_w, 9);
  mav.send_debug("SMV_H2O_T",time_w,temperature_w,mav.longitude,mav.latitude);
   
  mav.set_param("SMV_AIR_T",temperature_a, 9);
  mav.send_debug("SMV_AIR_T",time_a,temperature_a,pres,hum);

  SerialUSB.println("messages sent");
  delay(100);
}

//**************************************************************************
// secondary loops, listening to mavlink

void mavlink_listen_loop() {
  mav.Readdata();
  //yield called in the function
}

void readTemp()
{
    sensor.startAdc();
    delay(10);
    temperature_w = sensor.readTemperature();
    #ifdef DEBUG
      SerialUSB.print("Temperature is ");
      SerialUSB.println(temperature_w);
    #endif
}
