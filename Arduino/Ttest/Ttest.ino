/*
   Test software for ELL-i's TSYS01 board.
   This test is used to verify the functionality of your Tsys01 board.
   TSYS01 is 3.3V board. While it *might* survive a short connection to
   5V logic levels because of protection circuit onboard, this is not guaranteed.

   Test is success if the board returns valid temperature on both SPI and I2C communication.
   Powering off is not tested, as the communication lines can push enough power to
   sensor through protection diodes. Always pull communication lines down when entering
   power off mode.

   Test process:
   1) Connect wires as described below.
   2) Disconnect SPI Clock wire.
   3) LED BLINK -> Test 1 success.
   4) Connect SPI Clock wire.
   5) LED Stays on -> Test 2 success
   QC OK! Test Pass.

   If LED 2 does not stay on, test is failed.

   Test results:
   LED OFF    -> I2C fail
   LED Blinks -> I2C OK, SPI Fail
   LED ON     -> TEST PASS

   Board connection:
   Arduino       <-> TSYS01
   ------------------------
   5V OR 3V3     <-> Vin
   GND           <-> GND
   SCL AND SCLK  <-> SCLK/SCL
   SDA AND MOSI  <-> MOSI/SDA
   MISO          <->  MISO
   8             <-> MODE
   10            <-> CS/ADDR
   A0            <-> Shutdown
*/

#include "src/Tsys01.h"

/*
   Sanity check for returned values.
   If returned value is below or above levels below,
   value is considered faulty. Adjust these if you live in
   cold or hot region where room temperatures are different
*/

#define TEMPERATURE_MIN 2.0
#define TEMPERATURE_MAX 30.0

//debug led, becomes high after test has passed.
#define LED 13
#define slaveSelectPin 10
#define protocolPin 8
#define powerPin A0
#define SPI_TEST_BLINK_RATE 100
#define DEBUG

Tsys01 sensor;

void setup(void)
{

  float temperature = 0.0;

  uint8_t pass = HIGH;
  #ifdef DEBUG
    SerialUSB.begin(57600);
    while (!SerialUSB) {;}
    SerialUSB.println("Starting in 1s");
  #endif
  delay(2000);
  SerialUSB.println("Started");

  //Re-init sensor.
  sensor = Tsys01(TSYS01_I2C, powerPin);

  //I2C test
  do {
    SerialUSB.println("Testing I2C...");
    //Pass if test is successful
    pass = HIGH;
    
    sensor.startAdc();

    //ADC needs 10 ms
    delay(10);
    temperature = sensor.readTemperature();

    SerialUSB.print("Temperature is ");
    SerialUSB.println(temperature);

    if(temperature > TEMPERATURE_MIN && temperature < TEMPERATURE_MAX)
    {
      SerialUSB.println("I2C Communication OK, temperature is in valid range");
    }
    else
    {
      SerialUSB.println("I2C Communication OK, Temperature is not in valid range");
      pass = LOW;
    }

  //LED stays low if I2C does not pass
  digitalWrite(LED, pass);
  }while(pass==LOW);
}

void loop(void)
{
  sensor.startAdc();
  delay(10);
  float temperature = sensor.readTemperature();
  SerialUSB.print("Temperature is ");
  SerialUSB.println(temperature);
  delay(2000);
}
