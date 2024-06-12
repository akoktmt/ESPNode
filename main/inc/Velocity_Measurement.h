/*
 * Velocity_Measurement.h
 *
 *  Created on: Nov 29, 2023
 *      Author: win 10
 */

#ifndef VELOCITY_MEASUREMENT_H_
#define VELOCITY_MEASUREMENT_H_
#include "EKF.h"
typedef struct{
	float VelocityX;
	float CovarianeVx;
}Velocity;
void Velocity_Init(Velocity *Velocity);
void Velocity_MeasurementModel(EKF *EKF, Velocity *Velocity, Angle *Angle);
#endif /* VELOCITY_MEASUREMENT_H_ */
