#include "mavlink.h"
#include "MavlinkComm.h"
#include "circBuffer.h"
#include "inttypes.h"
#include "gpsPort.h"
#include <stdio.h>
#include "AUAV_V3_TestMavlink.h"
uint8_t UartOutBuff[MAVLINK_MAX_PACKET_LEN];



uint16_t PackHeartBeat(uint8_t system_id, uint8_t component_id){
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
  mavlink_message_t msg;
  uint16_t bytes2Send = 0;
  //////////////////////////////////////////////////////////////////////////
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
  return(bytes2Send);
}

uint16_t PackTextMsg(uint8_t system_id, uint8_t component_id){
  mavlink_message_t msg;
  mavlink_system_t mavlink_system;
  mavlink_system.sysid = system_id;                   ///< ID 20 for this airplane
  mavlink_system.compid = component_id;//MAV_COMP_ID_IMU;     ///< The component sending the message is the IMU, it could be also a Linux process
  char vr_message[50];
  memset(vr_message, 0, sizeof (vr_message));
  sprintf(vr_message, "Hello World");
  mavlink_msg_statustext_pack(mavlink_system.sysid,
  mavlink_system.compid,
  &msg,
  0,
  vr_message);
  return(mavlink_msg_to_send_buffer(UartOutBuff, &msg));
}


uint16_t PackRawIMU(uint8_t system_id, uint8_t component_id, mavlink_raw_imu_t mlRawIMUData ,uint32_t time_usec){
  mavlink_system_t mavlink_system;

  mavlink_system.sysid = system_id;                   ///< ID 20 for this airplane
  mavlink_system.compid = component_id;//MAV_COMP_ID_IMU;     ///< The component sending the message is the IMU, it could be also a Linux process
  //////////////////////////////////////////////////////////////////////////
  mavlink_message_t msg;
  memset(&msg, 0, sizeof (mavlink_message_t));
  mavlink_msg_raw_imu_pack(mavlink_system.sysid, mavlink_system.compid, &msg , time_usec , mlRawIMUData.xacc , mlRawIMUData.yacc , mlRawIMUData.zacc , mlRawIMUData.xgyro , mlRawIMUData.ygyro , mlRawIMUData.zgyro , mlRawIMUData.xmag , mlRawIMUData.ymag , mlRawIMUData.zmag );
  return( mavlink_msg_to_send_buffer(UartOutBuff, &msg));
}

uint16_t PackGpsRawInt(uint8_t system_id, uint8_t component_id, mavlink_gps_raw_int_t mlRawGpsDataInt ,uint32_t time_usec){
  mavlink_system_t mavlink_system;

  mavlink_system.sysid = system_id;                   ///< ID 20 for this airplane
  mavlink_system.compid = component_id;//MAV_COMP_ID_IMU;     ///< The component sending the message is the IMU, it could be also a Linux process
  //////////////////////////////////////////////////////////////////////////
  mavlink_message_t msg;
  memset(&msg, 0, sizeof (mavlink_message_t));
  mavlink_msg_gps_raw_int_pack(mavlink_system.sysid, mavlink_system.compid, &msg , time_usec ,mlRawGpsDataInt.fix_type, mlRawGpsDataInt.lat,
          mlRawGpsDataInt.lon, mlRawGpsDataInt.alt, mlRawGpsDataInt.eph, mlRawGpsDataInt.epv, mlRawGpsDataInt.vel,
          mlRawGpsDataInt.cog, mlRawGpsDataInt.satellites_visible);
  return( mavlink_msg_to_send_buffer(UartOutBuff, &msg));
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
/* Declare UART1 Tx Circular Buffer Structure */
extern MCHP_UART1_TxStr MCHP_UART1_Tx;

void TxN_Data_OverU1(uint16_t N){
  uint16_T i;
  for (i = 0U; i < N; i++) {
    uint16_T Tmp;
    Tmp = ~(MCHP_UART1_Tx.tail - MCHP_UART1_Tx.head);
    Tmp = Tmp & (Tx_BUFF_SIZE_Uart1 - 1);/* Modulo Buffer Size */
    if (Tmp != 0) {
      MCHP_UART1_Tx.buffer[MCHP_UART1_Tx.tail] = UartOutBuff[i];
      MCHP_UART1_Tx.tail = (MCHP_UART1_Tx.tail + 1) & (Tx_BUFF_SIZE_Uart1 - 1);
      Tmp--;
    }
  }
  _U1TXIF = U1STAbits.TRMT;
}