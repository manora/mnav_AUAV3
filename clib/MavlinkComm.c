#include "mavlink.h"
#include "MavlinkComm.h"
#include "circBuffer.h"
#include "inttypes.h"
#include "gpsPort.h"
#include <stdio.h>
#include "AUAV_V3_TestMavlink.h"
uint8_t UartOutBuff[MAVLINK_MAX_PACKET_LEN];
struct CircBuffer comMavlinkBuffer;
CBRef uartMavlinkInBuffer;
uint8_T DatafromGSmavlink[MAXINLEN+2];
mavlink_pending_requests_t mlPending;
mavlink_heartbeat_t mlHeartbeat;

struct pi_struct mlParamInterface;

void uartMavlinkBufferInit (void){
  _U1RXIP = 1;                         /* Rx Interrupt priority set to 1 */
  _U1RXIF = 0;
  _U1RXIE = 1;                         /* Enable Interrupt */
  /* Configure Remappables Pins */
   RPINR18 = 0x62;

  uartMavlinkInBuffer = (struct CircBuffer*) &comMavlinkBuffer;
  newCircBuffer(uartMavlinkInBuffer);
}

void __attribute__((__interrupt__,__auto_psv__)) _U1RXInterrupt(void)
{
  //
        _U1RXIF = 0;
    // Read the buffer while it has data
    // and add it to the circular buffer
    while (U1STAbits.URXDA == 1) {
        writeBack(uartMavlinkInBuffer, (unsigned char) U1RXREG);
    }

    // If there was an overun error clear it and continue
    if (U1STAbits.OERR == 1) {
        _U1RXIF = 0;
        U1STAbits.OERR = 0;
    }

    // clear the interrupt
    IFS0bits.U1RXIF = 0;

}

uint8_t isFinite(float s) {
  // By IEEE 754 rule, 2*Inf equals Inf
  return ((s == s) && ((s == 0) || (s != 2*s)));
}

void InitParameterInterface(void)
{
    strcpy(mlParamInterface.param_name[PAR_PID_AIRSPEED_P], "PID_AIRSPD_P");
    strcpy(mlParamInterface.param_name[PAR_PID_AIRSPEED_I], "PID_AIRSPD_I");
    strcpy(mlParamInterface.param_name[PAR_PID_AIRSPEED_D], "PID_AIRSPD_D");

    strcpy(mlParamInterface.param_name[PAR_PID_PITCH_FO_P], "PID_PIT_FO_P");
    strcpy(mlParamInterface.param_name[PAR_PID_PITCH_FO_I], "PID_PIT_FO_I");
    strcpy(mlParamInterface.param_name[PAR_PID_PITCH_FO_D], "PID_PIT_FO_D");

    strcpy(mlParamInterface.param_name[PAR_PID_ROLL_CON_P], "PID_ROLL_CO_P");
    strcpy(mlParamInterface.param_name[PAR_PID_ROLL_CON_I], "PID_ROLL_CO_I");
    strcpy(mlParamInterface.param_name[PAR_PID_ROLL_CON_D], "PID_ROLL_CO_D");

    strcpy(mlParamInterface.param_name[PAR_PID_HE_TO_PI_P], "PID_HE2PITC_P");
    strcpy(mlParamInterface.param_name[PAR_PID_HE_TO_PI_I], "PID_HE2PITC_I");

    strcpy(mlParamInterface.param_name[PAR_PID_HEI_ERR_FF], "PID_HERR_FF");

    strcpy(mlParamInterface.param_name[PAR_PID_YAW_DAMP_P], "PID_YAW_DA_P");
    strcpy(mlParamInterface.param_name[PAR_PID_YAW_DAMP_I], "PID_YAW_DA_I");
    strcpy(mlParamInterface.param_name[PAR_PID_YAW_DAMP_D], "PID_YAW_DA_D");

}

void protDecodeMavlink(void) {

    uint8_t indx, writeSuccess, commChannel = 1;
    mavlink_param_set_t set;
    mavlink_message_t msg;
    mavlink_status_t status;
    uint8_t* dataIn;
    // fix the data length so if the interrupt adds data
    // during execution of this block, it will be read
    // until the next gsRead
    unsigned int tmpLen = getLength(uartMavlinkInBuffer), i = 0;

    // if the buffer has more data than the max size, set it to max,
    // otherwise set it to the length
    //DatafromGSmavlink[0] = (tmpLen > MAXINLEN) ? MAXINLEN : tmpLen;

    // read the data
    //for (i = 1; i <= DatafromGSmavlink[0]; i += 1) {
      //mavlink_parse_char(commChannel, readFront(uartBufferInMavlink), &msg, &status);
      //DatafromGSmavlink[i] = readFront(uartMavlinkInBuffer);
    //}
    //dataIn = DatafromGSmavlink;
    // increment the age of heartbeat
    mlPending.heartbeatAge++;

    for (i = 0; i <= tmpLen; i++) {
        // Try to get a new message
       if (mavlink_parse_char(commChannel, readFront(uartMavlinkInBuffer), &msg, &status)) {
                    // Handle message
            switch (msg.msgid) {
                case MAVLINK_MSG_ID_HEARTBEAT:
                    mavlink_msg_heartbeat_decode(&msg, &mlHeartbeat);
                    // Reset the heartbeat
                    mlPending.heartbeatAge = 0;
                    break;
                case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
                    mlPending.piTransaction = 1;
                    mlPending.piProtState = PI_SEND_ALL_PARAM;
                    mlPending.piCurrentParamInTransaction = 0;
                    break;

                case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
                    // If it was in the middle of a list transmission or there is already a param enqueued
                    mlPending.piTransaction = 1;
                    switch (mlPending.piProtState) {
                        case PI_IDLE:
                            mlPending.piBackToList = 0; // no need to go back
                            mlPending.piQIdx = -1; // no Index
                            mlPending.piCurrentParamInTransaction = mavlink_msg_param_request_read_get_param_index(&msg); // assign directly
                            mlPending.piProtState = PI_SEND_ONE_PARAM;
                            break;

                        case PI_SEND_ALL_PARAM:
                            mlPending.piBackToList = 1; // mark to go back
                            mlPending.piQIdx++; // done like this because when empty index = -1
                            mlPending.piQueue[mlPending.piQIdx] = mavlink_msg_param_request_read_get_param_index(&msg); // put in in queue
                            mlPending.piProtState = PI_SEND_ONE_PARAM;
                            break;

                        case PI_SEND_ONE_PARAM:
                            if (mlPending.piBackToList) {
                                mlPending.piQIdx++; // done like this because when empty index = -1
                                mlPending.piQueue[mlPending.piQIdx] = mavlink_msg_param_request_read_get_param_index(&msg); // put in in queue
                            }
                            mlPending.piProtState = PI_SEND_ONE_PARAM;
                            break;
                    }
                    break;

                case MAVLINK_MSG_ID_PARAM_SET:
                    mavlink_msg_param_set_decode(&msg, &set);

                    if ((uint8_t) set.target_system == (uint8_t) SYSTEMID &&
                        (uint8_t) set.target_component == (uint8_t) COMPID) {


                        char* key = (char*) set.param_id;
                        uint8_t i, j;
                        uint8_t match;
                        for (i = 0; i < PAR_PARAM_COUNT; i++) {
                            match = 1;
                            for (j = 0; j < PARAM_NAME_LENGTH; j++) {
                                // Compare
                                if (((char) (mlParamInterface.param_name[i][j]))
                                    != (char) (key[j])) {
                                    match = 0;
                                } // if

                                // End matching if null termination is reached
                                if (((char) mlParamInterface.param_name[i][j]) == '\0') {
                                    break;
                                } // if
                            }// for j

                            // Check if matched
                            if (match) {
                                //sw_debug = 1;
                                // Only write and emit changes if there is actually a difference
                                // AND only write if new value is NOT "not-a-number"
                                // AND is NOT infinity

                                if (isFinite(set.param_value)) {

                                    mlParamInterface.param[i] = set.param_value;

                                    // Report back new value
                                    mlPending.piBackToList = 0; // no need to go back
                                    mlPending.piQIdx = -1; // no Index
                                    mlPending.piCurrentParamInTransaction = i; // assign directly
                                    mlPending.piProtState = PI_SEND_ONE_PARAM;
                                    mlPending.piTransaction = 1;

                                } // if different and not nan and not inf
                            } // if match
                        }// for i
                    } // if addressed to this
                    break;
              default:
                break;
            }
       }
    }
}

uint16_t ParameterInterfaceResponse(uint8_t system_id, uint8_t component_id){
  mavlink_message_t msg;
  uint16_t bytes2Send = 0;
  if (mlPending.piTransaction){

    switch (mlPending.piProtState) {
      case PI_SEND_ALL_PARAM:
          if (mlPending.piCurrentParamInTransaction < PAR_PARAM_COUNT) {

              mavlink_msg_param_value_pack(system_id,
                  component_id,
                  &msg,
                  mlParamInterface.param_name[mlPending.piCurrentParamInTransaction],
                  mlParamInterface.param[mlPending.piCurrentParamInTransaction],
                  MAV_PARAM_TYPE_REAL32, // TODO: make sure this is correct later on
                  PAR_PARAM_COUNT,
                  mlPending.piCurrentParamInTransaction);

              mlPending.piCurrentParamInTransaction++;

              // Copy the message to the send buffer
              bytes2Send += mavlink_msg_to_send_buffer(UartOutBuff, &msg);

          } else {
              mlPending.piProtState = PI_IDLE;
              mlPending.piCurrentParamInTransaction = PAR_PARAM_COUNT;
              mlPending.piTransaction = 0;
              mlPending.piBackToList = 0;
              mlPending.piQIdx = -1;
          }
          break;

      case PI_SEND_ONE_PARAM:


          if (!mlPending.piBackToList) { // if this is just a single pm read, i.e. not a retransmission
              mavlink_msg_param_value_pack(system_id,
                  component_id,
                  &msg,
                  mlParamInterface.param_name[mlPending.piCurrentParamInTransaction],
                  mlParamInterface.param[mlPending.piCurrentParamInTransaction],
                  MAV_PARAM_TYPE_REAL32,
                  PAR_PARAM_COUNT,
                  mlPending.piCurrentParamInTransaction);

              // Copy the message to the send buffer
              bytes2Send += mavlink_msg_to_send_buffer(UartOutBuff, &msg);


              mlPending.piCurrentParamInTransaction = PAR_PARAM_COUNT;
              mlPending.piProtState = PI_IDLE;
              mlPending.piTransaction = 0;
          } else { // you will need to go back to send all

              if (mlPending.piQIdx < 0) { // if you've sent all the requests, then go back to list
                  mlPending.piProtState = PI_SEND_ALL_PARAM;
                  mlPending.piBackToList = 0;

              } else { // send the requests
                  mavlink_msg_param_value_pack(system_id,
                      component_id,
                      &msg,
                      mlParamInterface.param_name[mlPending.piQueue[mlPending.piQIdx]],
                      mlParamInterface.param[mlPending.piQueue[mlPending.piQIdx]],
                      MAV_PARAM_TYPE_REAL32,
                      PAR_PARAM_COUNT,
                      mlPending.piQueue[mlPending.piQIdx]);

                  // Copy the message to the send buffer
                  bytes2Send += mavlink_msg_to_send_buffer(UartOutBuff, &msg);

                  // decrement the queue index
                  mlPending.piQIdx--;
              } // QIdx < 0
          }// !backToList
    }//switch
  }//if (mlPending.piTransaction)
  return(bytes2Send);
}
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


uint16_t PackScaledPressure(uint8_t system_id, uint8_t component_id, mavlink_scaled_pressure_t mlAirData ,uint32_t time_usec){
  mavlink_system_t mavlink_system;

  mavlink_system.sysid = system_id;                   ///< ID 20 for this airplane
  mavlink_system.compid = component_id;//MAV_COMP_ID_IMU;     ///< The component sending the message is the IMU, it could be also a Linux process
  //////////////////////////////////////////////////////////////////////////
  mavlink_message_t msg;
  memset(&msg, 0, sizeof (mavlink_message_t));
  mavlink_msg_scaled_pressure_pack(system_id, component_id, &msg,
						       time_usec/1000, mlAirData.press_abs, mlAirData.press_diff, mlAirData.temperature);
  return( mavlink_msg_to_send_buffer(UartOutBuff, &msg));
}

uint16_t PackSysStatus(uint8_t system_id, uint8_t component_id, mavlink_sys_status_t mlSysStatus){
  mavlink_system_t mavlink_system;

  mavlink_system.sysid = system_id;                   ///< ID 20 for this airplane
  mavlink_system.compid = component_id;//MAV_COMP_ID_IMU;     ///< The component sending the message is the IMU, it could be also a Linux process
  //////////////////////////////////////////////////////////////////////////
  mavlink_message_t msg;
  memset(&msg, 0, sizeof (mavlink_message_t));
  mavlink_msg_sys_status_pack(system_id, component_id, &msg,
						       mlSysStatus.onboard_control_sensors_present,mlSysStatus.onboard_control_sensors_enabled, mlSysStatus.onboard_control_sensors_health,
                   mlSysStatus.load, mlSysStatus.voltage_battery, mlSysStatus.current_battery,
                   mlSysStatus.battery_remaining, mlSysStatus.drop_rate_comm, mlSysStatus.errors_comm,
                   mlSysStatus.errors_count1, mlSysStatus.errors_count2, mlSysStatus.errors_count3, mlSysStatus.errors_count4);
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
  

//uint8_t GetCharAtBuffIdx(int32_t idx){
//    return(UartOutBuff[idx]);
//}
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