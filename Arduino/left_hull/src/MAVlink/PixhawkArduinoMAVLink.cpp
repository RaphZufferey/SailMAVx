/*
  PixhawkArduinoMAVLink.cpp - Library for using Arduino to recieve Pixhawk's sensor data as well as some other usefull data which you might need.
  Created by Shashi Kant, June 23, 2018.
  Using MAVLink C headers files generated from the ardupilotmega.xml with the help of mavgenerator.
*/

#include "PixhawkArduinoMAVLink.h"
#include <Scheduler.h>

#include "../Helper/Helper.h"

PixhawkArduinoMAVLink::PixhawkArduinoMAVLink(HardwareSerial &hs){
  _MAVSerial = &hs;
  MILLIG_TO_MS2 = 9.80665 / 1000.0;
  system_id = 1; // Your i.e. Arduino sysid
  component_id = 158; // Your i.e. Arduino compid
  type = MAV_TYPE_QUADROTOR;
  autopilot =  MAV_AUTOPILOT_INVALID;
}

bool PixhawkArduinoMAVLink::begin(){
  _MAVSerial->begin(9600);
  if(_MAVSerial->available()<=0){
    return 0;
  }else{
    return 1;
  }
}

// At first we will send some HeartBeats to Pixhawk to check whether it's available or not??
// After that we will check for whether we are recieving HeartBeats back from Pixhawk if Yes,
// We will note down its sysid and compid to send it a req to Stream Data.
void PixhawkArduinoMAVLink::Stream(){
  delay(2000);
  int flag=1;
  SerialUSB.println("Sending Heartbeats...");
  mavlink_message_t msghb;
  mavlink_heartbeat_t heartbeat;
  uint8_t bufhb[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_heartbeat_pack(system_id, component_id, &msghb, type, autopilot, MAV_MODE_PREFLIGHT, 0, MAV_STATE_STANDBY);
  uint16_t lenhb = mavlink_msg_to_send_buffer(bufhb, &msghb);
  delay(1000);
  _MAVSerial->write(bufhb,lenhb);
  SerialUSB.println("Heartbeats sent! Now will check for recieved heartbeats to record sysid and compid...");

  // Looping untill we get the required data.
  while(flag==1){
    delay(1);
    while(_MAVSerial->available()>0){
      mavlink_message_t msgpx;
      mavlink_status_t statuspx;
      uint8_t ch = _MAVSerial->read();
      if(mavlink_parse_char(MAVLINK_COMM_0, ch, &msgpx, &statuspx)){
        //SerialUSB.println("Message Parsing Done!");
        switch(msgpx.msgid){
          case MAVLINK_MSG_ID_HEARTBEAT:
          {
            mavlink_heartbeat_t packet;
            mavlink_msg_heartbeat_decode(&msgpx, &packet);
            received_sysid = msgpx.sysid; // Pixhawk sysid
            received_compid = msgpx.compid; // Pixhawk compid
            //SerialUSB.println("sysid and compid successfully recorded");
            flag = 0;
            break;
          }
        }
      }
    }
  }

  // Sending request for data stream...
  //SerialUSB.println("Now sending request for data stream...");
  delay(2000);
  mavlink_message_t msgds;
  uint8_t bufds[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_request_data_stream_pack(system_id, component_id, &msgds, received_sysid, received_compid, MAV_DATA_STREAM_CUSTOM , 0x05, 1);
  uint16_t lends = mavlink_msg_to_send_buffer(bufds, &msgds);
  delay(1000);
  _MAVSerial->write(bufds,lends);
  //SerialUSB.println("Request sent! Now you are ready to recieve datas...");

}

void PixhawkArduinoMAVLink::Readdata(){
  while(_MAVSerial->available() > 0){
    mavlink_message_t msg;
    mavlink_status_t status1;
    uint8_t ch = _MAVSerial->read();
    // SerialUSB.println(ch);
    if(mavlink_parse_char(MAVLINK_COMM_0, ch, &msg, &status1)){
      //SerialUSB.println("Message Parsing Done!");
      SerialUSB.println();
      SerialUSB.print("message ID: ");
      SerialUSB.print(msg.msgid);
      //time_now = millis();
      //SerialUSB.print(" Time: ");
      //SerialUSB.print(time_now);

      switch(msg.msgid){
        case MAVLINK_MSG_ID_PARAM_VALUE:{
          SerialUSB.println();
          SerialUSB.print("Reading parameter");
          mavlink_param_value_t data;
          mavlink_msg_param_value_decode(&msg, &data);
          char* param_id = (data.param_id);
          param_v = (data.param_value);
          delay(10);
          break;
        }
        case MAVLINK_MSG_ID_DEBUG_VECT:{
          SerialUSB.println();
          SerialUSB.print("Reading debug_vect");
          mavlink_debug_vect_t data;
          mavlink_msg_debug_vect_decode(&msg, &data);
          d_v1 = (data.x);
          d_v2 = (data.y);
          d_v3 = (data.z);
          delay(10);
          break;
        }
        case MAVLINK_MSG_ID_DEBUG:{
          SerialUSB.println();
          SerialUSB.print("Reading debug");
          mavlink_debug_t data;
          mavlink_msg_debug_decode(&msg, &data);
          d_0 = (data.value);
          delay(10);
          break;
        }
        case MAVLINK_MSG_ID_RC_CHANNELS:{
          //SerialUSB.println("Reading RC");
          mavlink_rc_channels_t data;
          mavlink_msg_rc_channels_decode(&msg, &data);

          cha1 = scaledRC(data.chan1_raw);
          cha2 = scaledRC(data.chan2_raw);
          cha3 = scaledRC(data.chan3_raw);
          cha4 = scaledRC(data.chan4_raw);
          cha5 = scaledRC(data.chan5_raw);
          cha6 = scaledRC(data.chan6_raw);
          cha7 = scaledRC(data.chan7_raw);
          cha8 = scaledRC(data.chan8_raw);
          delay(10);
          break;
        }
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:{
          SerialUSB.println();
          SerialUSB.print("Reading GPS");
          mavlink_global_position_int_t data;
          mavlink_msg_global_position_int_decode(&msg, &data);
          latitude = (data.lat);
          longitude = (data.lon);
          delay(10);
          break;
        }
      }
    }
    yield();
  }
return;
}


float PixhawkArduinoMAVLink::scaledRC(uint16_t raw_value){
  float scaled_value;
  if (raw_value < 1050){
    scaled_value = 0.0;
  } else if (raw_value > 1950){
    scaled_value = 1.0;
  } else {
    scaled_value = (float) raw_value / 900.0 - 7.0/6.0;
  }
  return scaled_value;
}



void PixhawkArduinoMAVLink::req_param(const char *param_id){
  // Sending request for parameter...
  //SerialUSB.println();
  //SerialUSB.print("Sending parameter read request...");
  delay(20);
  mavlink_message_t msg_s;
  uint8_t bufds[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_param_request_read_pack(system_id, component_id, &msg_s, received_sysid, received_compid, param_id , -1);
  uint16_t lends = mavlink_msg_to_send_buffer(bufds, &msg_s);
  delay(100);
  _MAVSerial->write(bufds,lends);
  //SerialUSB.println();
  //SerialUSB.print("Parameter request sent!");
  return;
}

void PixhawkArduinoMAVLink::set_param(const char *param_id, float param_value, uint8_t param_type){
  // Sending request for parameter...
  SerialUSB.println();
  SerialUSB.print("Setting param value...");
  delay(20);
  mavlink_message_t msg_s;
  uint8_t bufds[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_param_set_pack(system_id, component_id, &msg_s, received_sysid, received_compid, param_id, param_value, param_type);
  uint16_t lends = mavlink_msg_to_send_buffer(bufds, &msg_s);
  delay(100);
  _MAVSerial->write(bufds,lends);
  SerialUSB.println();
  SerialUSB.print("Param value set!");
  return;
  //1	MAV_PARAM_TYPE_UINT8
  //2	MAV_PARAM_TYPE_INT8
  //3	MAV_PARAM_TYPE_UINT16
  //4	MAV_PARAM_TYPE_INT16
  //5	MAV_PARAM_TYPE_UINT32
  //6	MAV_PARAM_TYPE_INT32
  //7	MAV_PARAM_TYPE_UINT64
  //8	MAV_PARAM_TYPE_INT64
  //9	MAV_PARAM_TYPE_REAL32
  //10	MAV_PARAM_TYPE_REAL64
}

void PixhawkArduinoMAVLink::send_debug(const char *name, float time, float x, float y, float z){
  // Sending request for parameter...
  //SerialUSB.println();
  //SerialUSB.print("Sending parameter read request...");
  delay(20);
  mavlink_message_t msg;
  uint8_t bufds[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_debug_vect_pack(system_id, component_id, &msg, name, time, x, y, z);

  uint16_t lends = mavlink_msg_to_send_buffer(bufds, &msg);
  delay(10);
  _MAVSerial->write(bufds,lends);

  return;
}
