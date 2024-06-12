/*
 * GPS_Measurement.c
 *
 *  Created on: Nov 29, 2023
 *      Author: win 10
 */
#include "GPS_Measurement.h"
#include <string.h>
#include <stdio.h>
#include "EKFmath.h"
extern uint8_t GPSrec;
void GPS_Init(GPS *GPS){

	memset(GPS->GPSCovariance,0,sizeof(GPS->GPSCovariance));
	memset(GPS->GPSGetPosition,0,sizeof(GPS->GPSGetPosition));
	memset(GPS->GPS_Model,0,sizeof(GPS->GPS_Model));
}

void EFK_GPSHandleMeasurement(GPS *GPS, EKF *EKF ){
	float Error[2];
	float HmatrixPx[5];
	float HmatrixPy[5];
	float HmatrixRelPx[5];
	float HmatrixRelPy[5];
	float InvovationCovPx=0;
	float InvovationCovPy=0;
	float InvovationCovPx_=0;
	float InvovationCovPy_=0;
	float KalmanPx[5];
	float KalmanPy[5];
	float KalmanPxCpy[5];
	float KalmanPyCpy[5];
	float Updatexrel[5];
	float Updateyrel[5];
	float CovarianX[5][5];
	float CovarianY[5][5];
	float I_Matrix[5][5];
	float Covarian_matrixX[5][5];
	float Covarian_matrixY[5][5];
	float KalmanGainX[5];
	float KalmanGainY[5];
	float ErrorX[5];
	float ErrorY[5];

	//-------------------------------------------------
	//-------------------------------------------------
	memset(I_Matrix,0,sizeof(I_Matrix));
	memset(KalmanPxCpy,0,sizeof(KalmanPxCpy));
	memset(HmatrixPx,0,sizeof(HmatrixPx));
	memset(HmatrixPy,0,sizeof(HmatrixPy));
	memset(HmatrixRelPx,0,sizeof(HmatrixRelPx));
	memset(KalmanPx,0,sizeof(KalmanPx));
	memset(Updatexrel,0,sizeof(Updatexrel));
	memset(Updateyrel,0,sizeof(Updateyrel));
	memset(KalmanPyCpy,0,sizeof(KalmanPyCpy));
	memset(HmatrixRelPy,0,sizeof(HmatrixRelPy));
	memset(KalmanPy,0,sizeof(KalmanPy));
	memset(Covarian_matrixX,0,sizeof(Covarian_matrixX));
	memset(Covarian_matrixY,0,sizeof(Covarian_matrixY));
	memset(KalmanGainX,0,sizeof(KalmanGainX));
	memset(KalmanGainY,0,sizeof(KalmanGainY));
	memset(ErrorX,0,sizeof(ErrorX));
	memset(ErrorY,0,sizeof(ErrorY));
	//-------------------------------------------------
	HmatrixPx[2]=1;
	HmatrixPy[3]=1;
	//------------------------------------------------
	I_Matrix[0][0] = 1;
    I_Matrix[1][1] = 1;
    I_Matrix[2][2] = 1;
    I_Matrix[3][3] = 1;
    I_Matrix[4][4] = 1;
	//------------------------------------------------
	//Error
	Error[0]= GPS->GPSGetPosition[0] - EKF->NexPx;
	Error[1]= GPS->GPSGetPosition[1] - EKF->NexPy;
	//------------------------------------------------
	ErrorX[2]=Error[0];
	ErrorY[3]=Error[1];
	// printf("Px %f || Py %f\r\n",GPS->GPSGetPosition[0],GPS->GPSGetPosition[1]);
	// printf("%f || %f\r\n",EKF->NexPx,EKF->NexPy);
	// printf("%f || %f\r\n",Error[0],Error[1]);
	//InovationCovarian
	multiplyVectorByMatrix(HmatrixPx,EKF->Prediction_CovarianceNex,HmatrixRelPx);
	InvovationCovPx = dot_product(HmatrixRelPx,HmatrixPx) + GPS->GPSCovariance[0][0]; 
	//---------------------------------------------------------------------
	multiplyVectorByMatrix(HmatrixPy,EKF->Prediction_CovarianceNex,HmatrixRelPy);
	InvovationCovPy = dot_product(HmatrixRelPy,HmatrixPy) + GPS->GPSCovariance[1][1]; 
	//Kalman Gain
	InvovationCovPx_=1 / InvovationCovPx;
	multiplyMatrixByVector(EKF->Prediction_CovarianceNex,HmatrixPx,KalmanPx);
	multiplyVectorByScalar(KalmanPx,InvovationCovPx_,KalmanGainX);
	//---------------------------------------------------------------------
	InvovationCovPy_=1 / InvovationCovPy;
	multiplyMatrixByVector(EKF->Prediction_CovarianceNex,HmatrixPy,KalmanPy);
	multiplyVectorByScalar(KalmanPy,InvovationCovPy_,KalmanGainY);
	//Update
	EKF->FirPx = EKF->NexPx + dot_product(KalmanGainX,ErrorX);
	//printf("%f || %f || %f || %f || %f\r\n",Updatexrel[0],Updatexrel[1],Updatexrel[2],Updatexrel[3],Updatexrel[4]);
	//-------------------------------------------------------------------------
	EKF->FirPy = EKF->NexPy + dot_product(KalmanGainY,ErrorY);
	//Covarian
 	multiply_transpose_matrix_with_matrix(KalmanGainX, HmatrixPx, CovarianX);
    subtractMatrices(I_Matrix, CovarianX, Covarian_matrixX, 5, 5);
    multiplyMatrices(Covarian_matrixX, EKF->Prediction_CovarianceNex, EKF->Prediction_CovarianceFir);
	//-------------------------------------------------------------------------------------------------
	multiply_transpose_matrix_with_matrix(KalmanGainY, HmatrixPy, CovarianY);
    subtractMatrices(I_Matrix, CovarianY, Covarian_matrixY, 5, 5);
    multiplyMatrices(Covarian_matrixY, EKF->Prediction_CovarianceNex, EKF->Prediction_CovarianceFir);
}
