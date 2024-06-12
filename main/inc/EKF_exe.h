#include "GPS_Measurement.h"
#include "Velocity_Measurement.h"
#include "Heading_Measurement.h"
void EKF_Exe();
void EKF_ExeInit();
void float2Bytes( uint8_t bytes_temp[4],float float_variable);
float simulateYawSignal();