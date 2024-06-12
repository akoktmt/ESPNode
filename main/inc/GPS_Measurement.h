/*
 * GPS_Measurement.h
 *
 *  Created on: Nov 29, 2023
 *      Author: win 10
 */

#ifndef GPS_MEASUREMENT_H_
#define GPS_MEASUREMENT_H_
#include "EKF.h"
typedef struct{
	float GPSGetPosition[2];
	float GPSCovariance[2][2];
	float GPS_Model[2][NUMBEROFMODLE];
}GPS;

void GPS_Init(GPS *GPS);
void EFK_GPSHandleMeasurement(GPS *GPS, EKF *EKF );


#endif /* GPS_MEASUREMENT_H_ */
