#include "EKFmath.h"
#include <math.h>
#include "Steering_MeasurementModel.h"
#include <stdint.h>
#include <string.h>
void Steering_Init(Steering*Steering){
    Steering->Covariane=0;
    Steering->Steering=0;
}
void Steering_MeasurementModel( EKF *EKF, Steering *Steering){

    float Inovation;
    float InovationCovariance;
    float KalmanGain;

    Inovation = Steering->Steering - EKF->NexStee;

    InovationCovariance = EKF->Prediction_CovarianceNex[5][5] +  Steering->Covariane ;

    KalmanGain = EKF->Prediction_CovarianceNex[5][5] * InovationCovariance;

    EKF->FirStee = EKF->NexStee + KalmanGain * Inovation;
    
    EKF->Prediction_CovarianceFir[5][5]= (1 - KalmanGain) * EKF->Prediction_CovarianceNex[5][5];
    
}