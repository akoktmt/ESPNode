#ifndef STEERING_MEASUREMENT_H_
#define STEERING_MEASUREMENT_H_
#include "EKF.h"
typedef struct{
	float Steering;
	float Covariane;
}Steering;
void Steering_Init(Steering*Steering);
void Steering_MeasurementModel( EKF *EKF, Steering *Steering);
#endif /* Steering_MEASUREMENT_H_ */