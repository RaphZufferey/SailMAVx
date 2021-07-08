#include "src/Mavlink/PixhawkArduinoMAVLink.h"

#include <Wire.h>
#include <Scheduler.h>
#include <HardwareSerial.h>

#include "src/Helper/Helper.h"

// #include "src/Tsys01/Tsys01.h"
#include <BME280I2C.h>

//#define powerPin A0
//#define TEMPERATURE_MIN 1.0
//#define TEMPERATURE_MAX 30.0
//#define DEBUG

BME280I2C bme;

HardwareSerial &hs = Serial1;
PixhawkArduinoMAVLink mav(hs);

//Tsys01 sensor;
float temperature = 0.0;

void setup() {

  #ifdef DEBUG
    SerialUSB.begin(57600);
    while (!SerialUSB) {;}
    SerialUSB.println("Starting in 1s");
  #endif
  delay(2000);
  SerialUSB.println("Started");

//////////////////   Sensor setup /////////////////
// uint8_t pass = HIGH;
//
//do {
//
//    SerialUSB.println("Testing I2C...");
//    //Pass if test is successful
//    pass = HIGH;
//
//    //Re-init sensor.
//    sensor = Tsys01(TSYS01_I2C, powerPin);
//
//    sensor.startAdc();
//
//    //ADC needs 10 ms
//    delay(10);
//    temperature = sensor.readTemperature();
//
//    SerialUSB.print("Temperature is ");
//    SerialUSB.println(temperature);
//
//    if(temperature > TEMPERATURE_MIN && temperature < TEMPERATURE_MAX)
//    {
//      SerialUSB.println("I2C Communication OK, temperature is in valid range");
//    }
//    else
//    {
//      SerialUSB.println("I2C Communication OK, Temperature is not in valid range");
//      pass = LOW;
//    }
//
//  //LED stays low if I2C does not pass
//
//  }while(pass==LOW);

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

  //  sensor.startAdc();
  //  delay(10);
  //  temperature = sensor.readTemperature();


  float hum(NAN), pres(NAN);

   BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
   BME280::PresUnit presUnit(BME280::PresUnit_Pa);

   bme.read(pres, temperature, hum, tempUnit, presUnit);

  mav.set_param("SMV_H20_T",temperature, 9);
  mav.send_debug("SMV_H20_T",millis(),temperature,mav.longitude,mav.latitude);
  SerialUSB.println("messages sent");
  delay(200);
}

//**************************************************************************
// secondary loops, listening to mavlink
void mavlink_listen_loop() {
  mav.Readdata();
  //yield called in the function
}
