/*
 * Velocity_Measurement.c
 *
 *  Created on: Nov 29, 2023
 *      Author: win 10
 */
#include "EKFmath.h"
#include <math.h>
#include "Velocity_Measurement.h"
#include <stdint.h>
#include <string.h>
#include "driver/twai.h"
#include "app_internal.h"
#include "can.h"

void Velocity_Init(Velocity *Velocity)
{
    Velocity->VelocityX = 0;
    Velocity->CovarianeVx = 0;
}

void Velocity_MeasurementModel(EKF *EKF, Velocity *Velocity, Angle *Angle)
{
    float InovationVelx = 0;
    float I_Matrix[5][5];
    float JacobianVelx[5];
    float Error[5];
    //----------------------------------------
    // float JacobianRel[4][5];
    // float JacobianRelTrans[5][4];
    // float JacobianVelspeed[4][4];
    // float NoiseMatrix[5][5];
    float JacobianVelxRel[5];
    float InnovationCov=0;
    float Inovation_=0;
    float Kalman[5];
    float KalmanGian[5];
    float CovarianX[5][5];
    float Covarian_matrixX[5][5];
     //----------------------------------------memset
    // memset(JacobianVelx,0,sizeof(JacobianVelx));
    memset(JacobianVelxRel,0,sizeof(JacobianVelxRel));
    memset(Kalman,0,sizeof(Kalman));
    memset(KalmanGian,0,sizeof(KalmanGian));
    memset(CovarianX,0,sizeof(CovarianX));
    memset(Covarian_matrixX,0,sizeof(Covarian_matrixX));
    memset(JacobianVelx,0,sizeof(JacobianVelx));
    memset(I_Matrix,0,sizeof(I_Matrix));
    memset(Error,0,sizeof(Error));
    //----------------------------------------inovation
    InovationVelx = Velocity->VelocityX - EKF->NexVelx;
    Error[0]=InovationVelx;
    //-----------------------------------------------------------imatrix
    I_Matrix[0][0] = 1;
    I_Matrix[1][1] = 1;
    I_Matrix[2][2] = 1;
    I_Matrix[3][3] = 1;
    I_Matrix[4][4] = 1;
    //-----------------------------------------------------------
    JacobianVelx[0]=1;
    //-----------------------------------------------------------
    multiplyVectorByMatrix(JacobianVelx,EKF->Prediction_CovarianceNex,JacobianVelxRel);
	InnovationCov = dot_product(JacobianVelxRel,JacobianVelx) + Velocity->CovarianeVx; 
    //-----------------------------------------------------------
    Inovation_= 1 / InnovationCov;
	multiplyMatrixByVector(EKF->Prediction_CovarianceNex,JacobianVelx,Kalman);
	multiplyVectorByScalar(Kalman,Inovation_,KalmanGian);
    //------------------------------------------------------------
    EKF->FirVelx = EKF->NexVelx + dot_product(KalmanGian,Error);
   // printf(" IN %f\r\n",EKF->FirPx);
    //------------------------------------------------------------
    multiply_transpose_matrix_with_matrix(KalmanGian, JacobianVelx, CovarianX);
    subtractMatrices(I_Matrix, CovarianX, Covarian_matrixX, 5, 5);
    multiplyMatrices(Covarian_matrixX, EKF->Prediction_CovarianceNex, EKF->Prediction_CovarianceFir);
}