#ifndef  _MAVLINK_COMM_H_
#define  _MAVLINK_COMM_H_
uint16_t PackRawIMU(uint8_t system_id, uint8_t component_id, mavlink_raw_imu_t mlRawIMUData ,uint32_t time_usec);
char sendQGCDebugMessage(const char * dbgMessage, char severity, unsigned char* bytesToAdd, char positionStart) ;
//uint8_t GetCharAtIdx(int2_t idx);
#endif