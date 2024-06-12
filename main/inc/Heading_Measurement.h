/*
 * Heading_Measurement.h
 *
 *  Created on: Nov 29, 2023
 *      Author: win 10
 */

#ifndef HEADING_MEASUREMENT_H_
#define HEADING_MEASUREMENT_H_
#include "EKF.h"
typedef struct{
	float Yaw;
	float Covariane;
}Heading;
void Heading_Init(Heading*Heading);
// void Heading_HandleMeasurement(EKF *EKF, Input *Input, Heading *Heading);
void Heading_MeasurementModel(EKF *EKF, Heading *Heading, Input *Input);
float bytes2Float(uint8_t bytes[4]);
float decompress_uint8_to_float(uint8_t value, float minRange, float maxRange);
void float2Bytes( uint8_t bytes_temp[4],float float_variable);
#endif /* HEADING_MEASUREMENT_H_ */
