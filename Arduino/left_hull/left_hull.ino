#include "src/Mavlink/PixhawkArduinoMAVLink.h"

#include <Wire.h>
#include <Scheduler.h>
#include <HardwareSerial.h>

#include "src/Helper/Helper.h"

#include "src/Tsys01/Tsys01.h"

#define powerPin A0
#define TEMPERATURE_MIN 1.0
#define TEMPERATURE_MAX 30.0

HardwareSerial &hs = Serial1;
PixhawkArduinoMAVLink mav(hs);

Tsys01 sensor;
float temperature = 0.0;

void setup() {

  #ifdef DEBUG
    Serial.begin(57600);
    while (!Serial) {;}
    Serial.println("Starting in 1s");
  #endif
  delay(2000);

//////////////////   Sensor setup /////////////////
uint8_t pass = HIGH;

do {

    Serial.println("Testing I2C...");
    //Pass if test is successful
    pass = HIGH;

    //Re-init sensor.
    sensor = Tsys01(TSYS01_I2C, powerPin);

    sensor.startAdc();

    //ADC needs 10 ms
    delay(10);
    temperature = sensor.readTemperature();

    Serial.print("Temperature is ");
    Serial.println(temperature);

    if(temperature > TEMPERATURE_MIN && temperature < TEMPERATURE_MAX)
    {
      Serial.println("I2C Communication OK, temperature is in valid range");
    }
    else
    {
      Serial.println("I2C Communication OK, Temperature is not in valid range");
      pass = LOW;
    }

  //LED stays low if I2C does not pass

  }while(pass==LOW);


  while(!mav.begin()){
    Serial.println("Not Connected!");
    delay(1000);
  }
  mav.Stream();
  delay(1000);

  Scheduler.startLoop(mavlink_listen_loop);


}

//**************************************************************************
// main loops, reading sensors and running control
void loop() {

    sensor.startAdc();
    delay(10);
    temperature = sensor.readTemperature();

  mav.set_param("SMV_H20_T",temperature, 9);
  mav.send_debug("SMV_H20_T",millis(),temperature,mav.longitude,mav.latitude);
  delay(200);
}

//**************************************************************************
// secondary loops, listening to mavlink
void mavlink_listen_loop() {
  mav.Readdata();
  //yield called in the function
}
