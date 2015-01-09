#include "mavlink.h"
#include "MavlinkComm.h"
#include <stdio.h>
mavlink_raw_imu_t mlRawIMU;
uint8_t UartOutBuff[MAVLINK_MAX_PACKET_LEN];


uint16_t PackHeartBeat(void){
  mavlink_system_t mavlink_system;

  mavlink_system.sysid = 20;                   ///< ID 20 for this airplane
  mavlink_system.compid = MAV_COMP_ID_IMU;     ///< The component sending the message is the IMU, it could be also a Linux process
  mavlink_system.type = MAV_TYPE_FIXED_WING;   ///< This system is an airplane / fixed wing

  // Define the system type, in this case an airplane
  uint8_t system_type = MAV_TYPE_FIXED_WING;
  uint8_t autopilot_type = MAV_AUTOPILOT_GENERIC;

  uint8_t system_mode = MAV_MODE_PREFLIGHT; ///< Booting up
  uint32_t custom_mode = 0;                 ///< Custom mode, can be defined by user/adopter
  uint8_t system_state = MAV_STATE_STANDBY; ///< System ready for flight
  mavlink_message_t msg;
  

// Pack the message
  mavlink_msg_heartbeat_pack(mavlink_system.sysid, mavlink_system.compid, &msg, system_type, autopilot_type, system_mode, custom_mode, system_state);

// Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(UartOutBuff, &msg);
  // Send the message with the standard UART send function
// uart0_send might be named differently depending on
// the individual microcontroller / library in use.
//uart0_send(buf, len);
  return(len);
}


//#include "AUAV_V3_TestMavLink.h"
//extern MCHP_UART1_TxStr MCHP_UART1_Tx;

uint16_t PackRawIMU(uint8_t system_id, uint8_t component_id, mavlink_raw_imu_t mlRawIMUData ,uint32_t time_usec){
  mavlink_system_t mavlink_system;

  mavlink_system.sysid = system_id;                   ///< ID 20 for this airplane
  mavlink_system.compid = component_id;//MAV_COMP_ID_IMU;     ///< The component sending the message is the IMU, it could be also a Linux process
  mavlink_system.type = MAV_TYPE_FIXED_WING;   ///< This system is an airplane / fixed wing

  // Define the system type, in this case an airplane
  uint8_t system_type = MAV_TYPE_FIXED_WING;
  uint8_t autopilot_type = MAV_AUTOPILOT_GENERIC;

  uint8_t system_mode = MAV_MODE_PREFLIGHT; ///< Booting up
  uint32_t custom_mode = 0;                 ///< Custom mode, can be defined by user/adopter
  uint8_t system_state = MAV_STATE_STANDBY; ///< System ready for flight

  //////////////////////////////////////////////////////////////////////////
  mavlink_message_t msg;
    uint16_t bytes2Send = 0;
    static uint8_t SendThis = 0;
    static uint8_t SendScdCounter = 0;
    char vr_message[50];
    if (SendScdCounter == 0)
      SendThis = 0;
    else
      SendThis = 1;
    switch(SendThis){
      case 0:
        // clear the msg to pack a new variable
        memset(&msg, 0, sizeof (mavlink_message_t));
        // Pack the Heartbeat message
        mavlink_msg_heartbeat_pack(mavlink_system.sysid, mavlink_system.compid, &msg, system_type, autopilot_type, system_mode, custom_mode, system_state);
        /*
        mavlink_msg_heartbeat_pack(system_id,
                        component_id,
                        &msg,
                        MAV_TYPE_FIXED_WING,//MAV_TYPE_GENERIC,
                        MAV_AUTOPILOT_SLUGS,//MAV_AUTOPILOT_GENERIC,
                        MAV_MODE_PREFLIGHT,//mlHeartbeatLocal.base_mode,
                        MAV_MODE_PREFLIGHT,//mlHeartbeatLocal.custom_mode,
                        MAV_STATE_UNINIT//mlHeartbeatLocal.system_status
                        );
*/
        // Copy the message to the send buffer
        bytes2Send += mavlink_msg_to_send_buffer(UartOutBuff, &msg);
        SendThis = 1;
        break;
      case 1:
        memset(&msg, 0, sizeof (mavlink_message_t));
        mavlink_msg_raw_imu_pack(mavlink_system.sysid, mavlink_system.compid, &msg , time_usec , mlRawIMUData.xacc , mlRawIMUData.yacc , mlRawIMUData.zacc , mlRawIMUData.xgyro , mlRawIMUData.ygyro , mlRawIMUData.zgyro , mlRawIMUData.xmag , mlRawIMUData.ymag , mlRawIMUData.zmag );
        bytes2Send += mavlink_msg_to_send_buffer(UartOutBuff, &msg);
        SendThis = 2;
        break;
      case 2:
        memset(vr_message, 0, sizeof (vr_message));
        sprintf(vr_message, "Hello World");
        mavlink_msg_statustext_pack(mavlink_system.sysid,
        mavlink_system.compid,
        &msg,
        0,
        vr_message);
        bytes2Send += mavlink_msg_to_send_buffer(UartOutBuff, &msg);
        SendThis = 0;
        break;

    }
    SendScdCounter++;
    SendScdCounter %= 50;
    return(bytes2Send);

}

char sendQGCDebugMessage(const char * dbgMessage, char severity, unsigned char* bytesToAdd, char positionStart) {
    mavlink_message_t msg;
    unsigned char bytes2Send = 0; // size in bytes of the mavlink packed message (return value)

    mavlink_msg_statustext_pack(101,
        1,
        &msg,
        severity,
        dbgMessage);

    bytes2Send = mavlink_msg_to_send_buffer(UartOutBuff, &msg);

    return bytes2Send;
}
  

uint8_t GetCharAtBuffIdx(int32_t idx){
    return(UartOutBuff[idx]);
}
