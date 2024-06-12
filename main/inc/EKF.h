/*
 * EKF.h
 *
 *  Created on: Nov 11, 2023
 *      Author: win 10
 */

#ifndef EKF_H_
#define EKF_H_

#define LENGTH_REAR 0.24
#define LENGTH_CAR 0.48
#define SIZE 5
#define NUMBEROFMODLE 5
#define EARTH_RADIUS 6371000.0f // Earth's radius in meters
#define PI 3.14159265f
#define TIME_SCALE 10.0f
typedef struct{
	float FirPx;
	float FirPy;
	float FirVelx;
	float FirHea;
	float FirStee;

	float NexPx;
	float NexPy;
	float NexVelx;
	float NexHea;
	float NexStee;

	float CovPx;
	float CovPy;
	float CovVelx;
	float CovHea;
	float CovStee;

	float Prediction_CovarianceNex[NUMBEROFMODLE][NUMBEROFMODLE];
	float Prediction_CovarianceFir[NUMBEROFMODLE][NUMBEROFMODLE];
}EKF;

typedef struct {
	float Velx;
	float Stee;
	float Head;
	float Accx;
	float Accy;
	float Accz;
	float Time;
}Input;

typedef struct{
	float AngleBeta;
	float Roll;
	float Pitch;
	float Yaw;
}Angle;

void EKF_Init(EKF *EKF,Input *Input);
void EKF_PredictionStep(EKF *EKF, Angle *Angle, Input *Input);

#endif /* EKF_H_ */


