#include "src/Mavlink/PixhawkArduinoMAVLink.h"

#include "src/ELVH/ELVH.h"
#include "src/SSC/SSC.h"

#include "src/Sampling/Sampling.h"
#include "src/Sampling/Sample.h"
#include "src/Jets/Jets.h"
#include "src/Attitude/Attitude.h"

#include <Wire.h>
#include <Scheduler.h>
#include <HardwareSerial.h>

#include "src/Helper/Helper.h"


HardwareSerial &hs = Serial1;
PixhawkArduinoMAVLink mav(hs);
ELVH dp_sensor;
SSC p_sensor;
Sampling sampling;
Jets jets;
Attitude attitude;

void setup() {

  #ifdef DEBUG
    Serial.begin(57600);
    while (!Serial) {;}
    Serial.println("Starting in 1s");
  #endif
  delay(2000);

  Wire.begin();
  dp_sensor.init();
  p_sensor.init();
  sampling.init();
  jets.init();
  attitude.init();

  while(!mav.begin()){
    Serial.println("Not Connected!");
    delay(1000);
  }
  mav.Stream();
  delay(1000);

  Scheduler.startLoop(mavlink_listen_loop);
  Scheduler.startLoop(external_input);
}

//**************************************************************************
// main loops, reading sensors and running control
void loop() {
  // put your main code here, to run repeatedly:
  // Update pressure and temperature readings
  dp_sensor.read();
  //dp_sensor.print();
  //dp_sensor.test_calculate();

  p_sensor.read();
  //p_sensor.print();
  //p_sensor.test_calculate();

  sampling.loop();

  if (sampling.get_status() == 4){
    mav.send_debug(MD_STATUS, millis(), 1, sampling.get_current_sample_nb(), sampling.get_status());
    mav.send_debug(MD_STREAM, millis(), p_sensor.get_pressure(), dp_sensor.get_pressure(), sampling.get_current_sample_volume());
  }



  jets.loop();
  //jets.print();

  attitude.set_current_depth(p_sensor.get_pressure()/100);
  attitude.loop();
  
  delay(50);
}

//**************************************************************************
// secondary loops, listening to mavlink
void mavlink_listen_loop() {
  mav.Readdata();
  //yield called in the function
  if(mav.cha8 > 0.6){
    sampling.start_sampling();
  } else if (mav.cha8 < 0.4) {
    sampling.stop_sampling(Sampling::stop_reason_t::SR_SERIAL_KILL);
  }
}

//**************************************************************************
// external input loops = user inputs
void external_input(){

  if(Serial.available()>0){
    byte data = Serial.read();
    Serial.print("received ");
    Serial.println(data);
    if('0' <= data && data <='9'){
      switch(data){
        case '0':
        {
          sampling.start_sampling();
        } break;

        case '1':
        {
          sampling.stop_sampling(Sampling::stop_reason_t::SR_SERIAL_KILL);
        } break;

        case '2':
        {
          sampling.start_purging();
        } break;

        case '3':
        {
          sampling.stop_purging(Sampling::stop_reason_t::SR_SERIAL_KILL);
        } break;

        case '4':
        {
          Serial.print(sampling.get_status_str());
          Sample sample0 = sampling.get_sample(0);
          sample0.print();
          Sample sample1 = sampling.get_sample(1);
          sample1.print();
        } break;

        case '5':
        {
          Serial.print("starting attitude control");
          attitude.start_stabilize();
        } break;

        case '6':
        {
          Serial.print("stopping attitude control");
          attitude.stop_stabilize();
        } break;
      }
    }
  }
}
