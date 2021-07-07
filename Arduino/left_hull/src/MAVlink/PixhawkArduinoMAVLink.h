/*
  PixhawkArduinoMAVLink.h - Library for using Arduino to recieve Pixhawk sensors data.
  Created by Shashi Kant, June 23, 2018.
  Using MAVLink C headers files generated from the ardupilotmega.xml with the help of mavgenerator.
*/

#ifndef PixhawkArduinoMAVLink_h
#define PixhawkArduinoMAVLink_h

#include "ardupilotmega/mavlink.h"
#include "checksum.h"
#include "mavlink_types.h"
#include "protocol.h"
#include <Arduino.h>
#include <HardwareSerial.h>
#include <Scheduler.h>

#include "medusa_mavlink_debug.h"

#include "../Helper/Helper.h"

class PixhawkArduinoMAVLink
{
  public:
  float cha1, cha2, cha3, cha4, cha5, cha6, cha7, cha8;
  float param_v, d_v1, d_v2, d_v3, d_0, latitude, longitude;

    PixhawkArduinoMAVLink(HardwareSerial &hs);
    bool begin();
    void Stream();
    void Readdata();
    void ReadRC(int16_t *ch1, int16_t *ch2, int16_t *ch3, int16_t *ch4, int16_t *ch5, int16_t *ch6, int16_t *ch7, int16_t *ch8);
    void req_param(const char *param_id);
    void set_param(const char *param_id, float param_value, uint8_t param_type);
    void send_debug(const char *name, float time, float x, float y, float z);
    HardwareSerial* _MAVSerial;
    double MILLIG_TO_MS2;
    uint8_t system_id;
    uint8_t component_id;
    uint8_t type;
    uint8_t autopilot;
    uint8_t received_sysid; // Pixhawk sysid
    uint8_t received_compid; // Pixhawk compid
    unsigned long time_now = 0;
    float scaledRC(uint16_t raw_value);
};

#endif
