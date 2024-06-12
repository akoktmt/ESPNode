/*
 * EKF.c
 *
 *  Created on: Nov 11, 2023
 *      Author: win 10
 */
#include "EKF.h"
#include "EKFmath.h"
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "app_internal.h"
#include <stdio.h>
// Freertos
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
// ESP LOG
#include "esp_log.h"
#include "esp_err.h"
#include "sdkconfig.h"

void EKF_Init(EKF *EKF,Input *Input)
{
//First Step
	EKF->FirPx= 0;  //input
	EKF->FirPy=	0; //input
	EKF->FirVelx=0; //input
	EKF->FirHea=0; //input
	EKF->FirStee=0; //input
//Next Step
	EKF->NexPx=0;
	EKF->NexPy=0;
	EKF->NexVelx=0;
	EKF->NexHea=0;
	EKF->NexStee=0;
//Covariance
	// EKF->CovPx=0; //input
	// EKF->CovPy=0; //input
	// EKF->CovVelx=0;//input
	// EKF->CovVely=0;//input
	// EKF->CovHea=0; //input
	// EKF->CovStee=0; //input
//FirstIput
	Input->Accx=0;
	Input->Accy=0;
	Input->Accz=0;
	Input->Time=0; //input
//Covariance
	memset(EKF->Prediction_CovarianceNex,0,sizeof(EKF->Prediction_CovarianceNex));
	memset(EKF->Prediction_CovarianceFir,0,sizeof(EKF->Prediction_CovarianceFir));
	EKF->Prediction_CovarianceFir[0][0]= EKF->CovVelx;
	EKF->Prediction_CovarianceFir[1][1]= EKF->CovStee;
	EKF->Prediction_CovarianceFir[2][2]= EKF->CovPx;
	EKF->Prediction_CovarianceFir[3][3]= EKF->CovPy;
	// EKF->Prediction_CovarianceFir[2][2]= EKF->CovVelx;
	EKF->Prediction_CovarianceFir[4][4]= EKF->CovHea;
	// EKF->Prediction_CovarianceFir[4][4]= EKF->CovStee;
}
// Hàm chuyển đổi vận tốc theo vĩ độ
float convert_velocity_latitude(float velocity_mps) {
    return velocity_mps / (EARTH_RADIUS * PI / 180);
}

// Hàm chuyển đổi vận tốc theo kinh độ
float convert_velocity_longitude(float velocity_mps, float latitude_degrees) {
    return velocity_mps / (EARTH_RADIUS * cos(latitude_degrees * PI / 180) * PI / 180);
}
void EKF_PredictionStep(EKF *EKF, Angle *Angle, Input *Input){
// Prediction State
	// float a_hx;
	// float a_hy;

	// a_hx = Input->Accx * cos(Angle->Pitch) + Input->Accy * sin(Angle->Roll) * sin(Angle->Pitch) + Input->Accz * cos(Angle->Roll) * sin(Angle->Pitch);
	// a_hy = Input->Accy * cos(Angle->Roll) - Input->Accz * sin(Angle->Roll);
 	// float metersPerDegree = (PI / 180.0f) * EARTH_RADIUS;
    // float cosLatitude = (PI*cosf(EKF->FirPx * PI / 180.0f)*EARTH_RADIUS)/180;
	// float metersPerDegreeLongitude = EKF->FirVelx/cosLatitude;
	float velocityDegreesPerSecondLatitude =  convert_velocity_latitude (EKF->FirVelx);
	float velocityDegreesPerSecondLongitude = convert_velocity_longitude(EKF->FirVelx,EKF->FirPx);

	// printf("velocityDegreesPerSecondLongitude %f\r\n",velocityDegreesPerSecondLongitude);
	// printf("velocityDegreesPerSecondLatitude %f\r\n",velocityDegreesPerSecondLatitude);
	//printf("Acc %f\r\n",Input->Accx);
	 Angle->AngleBeta= (atan(LENGTH_REAR*tan(EKF->FirStee*PI/180))/LENGTH_CAR);

	 EKF->NexVelx= EKF->FirVelx +  Input->Accx *Input->Time;
	// printf("Acc %f\r\n",EKF->NexVelx);
	 EKF->NexStee= EKF->FirStee + Input->Stee*Input->Time;

	 EKF->NexPx= EKF->FirPx + (velocityDegreesPerSecondLatitude * cos(Angle->AngleBeta*PI/180+EKF->FirHea*PI/180))*Input->Time;

	 EKF->NexPy= EKF->FirPy + (velocityDegreesPerSecondLongitude * sin(Angle->AngleBeta*PI/180 +EKF->FirHea*PI/180))*Input->Time;
	//printf("%f || %f\r\n",EKF->NexPx,EKF->NexPy);
	 EKF->NexHea= EKF->FirHea + ((EKF->FirVelx*tan(EKF->FirStee*PI/180)*cos(Angle->AngleBeta*PI/180))/LENGTH_CAR)*Input->Time;

	//   printf("EKF->NexHea %f\r\n", EKF->NexHea);
	//   printf("EKF->FirVelx %f\r\n", EKF->FirVelx);
	//   printf("EKF->FirStee %f\r\n", EKF->FirStee);
	//   printf("Angle->AngleBeta %f\r\n", Angle->AngleBeta);
	//    printf("Input->Stee %f\r\n", Input->Stee);
	//  printf("ax %f || ay%f\r\n",a_hx,a_hy);
// Prediction Covariance 
	 float Mul_Result[5][5];
	 float Trans_Result[5][5];
	 float Jacobian[5][5];
	 float Qmatrix[5][5];
	 float Jacobianw[5][5];
	 float Jacobianwresult[5][5];
	 float Qresult[5][5];
	 float Fresult[5][5];

	 memset(Trans_Result,0,sizeof(Trans_Result));
	 memset(Jacobian,0,sizeof(Jacobian));
	 memset(Mul_Result,0,sizeof(Mul_Result));
	 memset(Qmatrix,0,sizeof(Qmatrix));
	 memset(Jacobianw,0,sizeof(Jacobianw));
	 memset(Fresult,0,sizeof(Fresult));
	 memset(Qresult,0,sizeof(Qresult));
	
	 Jacobian[0][0]=1;
	 Jacobian[0][1]=0;
	 Jacobian[0][2]=0;
	 Jacobian[0][3]=0;
	 Jacobian[0][4]=0;

	 Jacobian[1][0]=0;
	 Jacobian[1][1]=1;
	 Jacobian[1][2]=0;
	 Jacobian[1][3]=0;
	 Jacobian[1][4]=0;

	 Jacobian[2][0]=(180*Input->Time*cos(EKF->FirHea*PI/180+atan((LENGTH_REAR*tan(EKF->FirStee*PI/180))/LENGTH_CAR)))/(EARTH_RADIUS*PI);
	 Jacobian[2][1]=-(180*EKF->FirVelx *LENGTH_CAR * Input->Time*LENGTH_REAR*sin(EKF->FirHea*PI/180 + atan(LENGTH_REAR*tan(EKF->FirStee*PI/180)/LENGTH_CAR))*(1/cos(EKF->FirStee*PI/180)*cos(EKF->FirStee*PI/180)))/EARTH_RADIUS*PI*(LENGTH_CAR*LENGTH_CAR+LENGTH_REAR*LENGTH_REAR*tan(EKF->FirStee*PI/180)*tan(EKF->FirStee*PI/180));
	 Jacobian[2][2]=1;
	 Jacobian[2][3]=0;
	 Jacobian[2][4]=-180*(EKF->FirVelx*Input->Time*sin(EKF->FirHea*PI/180+atan(LENGTH_REAR*tan(EKF->FirStee*PI/180)/LENGTH_CAR)))/(EARTH_RADIUS*PI);

 	 Jacobian[3][0]=180*Input->Time*sin(EKF->FirHea*PI/180+atan((LENGTH_REAR*tan(EKF->FirStee*PI/180))/LENGTH_CAR))/(EARTH_RADIUS*PI)*cosf(EKF->FirPx * PI / 180.0f);
	 Jacobian[3][1]=(EKF->FirVelx *180*LENGTH_CAR * Input->Time*LENGTH_REAR*cos(EKF->FirHea*PI/180+atan(LENGTH_REAR*tan(EKF->FirStee*PI/180)/LENGTH_CAR))*(1/cos(EKF->FirStee*PI/180)*cos(EKF->FirStee*PI/180)))/(EARTH_RADIUS*PI)*(LENGTH_CAR*LENGTH_CAR+LENGTH_REAR*LENGTH_REAR*tan(EKF->FirStee*PI/180)*tan(EKF->FirStee*PI/180))*cosf(EKF->FirPx * PI / 180.0f);
	 Jacobian[3][2]=EKF->FirVelx *Input->Time* sin(EKF->FirPx * PI/180) * sin(EKF->FirHea*PI/180+atan((LENGTH_REAR*tan(EKF->FirStee*PI/180))/LENGTH_CAR))/EARTH_RADIUS *cosf(EKF->FirPx * PI / 180.0f)*cosf(EKF->FirPx * PI / 180.0f) ;
	 Jacobian[3][3]=1;
	 Jacobian[3][4]=EKF->FirVelx*180*Input->Time*cos(EKF->FirHea*PI/180+atan(LENGTH_REAR*tan(EKF->FirStee*PI/180)/LENGTH_CAR))/(EARTH_RADIUS*PI)*cosf(EKF->FirPx * PI / 180.0f);

	 Jacobian[4][0]=(Input->Time*tan(EKF->FirStee*PI/180)*fabs(LENGTH_CAR))/(LENGTH_CAR*sqrt(LENGTH_CAR*LENGTH_CAR+LENGTH_REAR*LENGTH_REAR*tan(EKF->FirStee*PI/180)*tan(EKF->FirStee*PI/180)));
	 Jacobian[4][1]=(EKF->FirVelx*LENGTH_CAR*LENGTH_CAR*LENGTH_CAR*Input->Time*(1/cos(EKF->FirStee*PI/180)*cos(EKF->FirStee*PI/180)))/(pow((LENGTH_CAR*LENGTH_CAR+LENGTH_REAR*LENGTH_REAR*tan(EKF->FirStee*PI/180)*tan(EKF->FirStee*PI/180)),1.5)*fabs(LENGTH_CAR));
	 Jacobian[4][2]=0;
	 Jacobian[4][3]=0;
	 Jacobian[4][4]=1;
	//Noise prediction
	Qmatrix[0][0]=0.52;
	Jacobianw[0][0]=Input->Time;
	// printf("J02 %f|| J04 %f|| J05 %f||J13 %f ||J14%f ||J15%f || J43 %f|| J45 %f\r\n",Jacobian[0][2],Jacobian[0][4],Jacobian[0][5],Jacobian[1][3],Jacobian[1][4],Jacobian[1][5],Jacobian[4][3],Jacobian[4][5]);
	 // printf("%f || %f  || %f||  %f||  %f || %f\r\n", EKF->Prediction_CovarianceFir[0][0], EKF->Prediction_CovarianceFir[1][1], EKF->Prediction_CovarianceFir[2][2], EKF->Prediction_CovarianceFir[3][3], EKF->Prediction_CovarianceFir[4][4], EKF->Prediction_CovarianceFir[5][5]);
	 multiplyMatrices(Jacobian,EKF->Prediction_CovarianceFir, Mul_Result);
	 //printf("mul 1 %f|| mul 2 %f|| mul 3%f|| mul 4%f|| mul 5 %f|| mul 6%f\r\n",EKF->Prediction_CovarianceFir[0][0],EKF->Prediction_CovarianceFir[1][1],EKF->Prediction_CovarianceFir[2][2],EKF->Prediction_CovarianceFir[3][3],EKF->Prediction_CovarianceFir[4][4],EKF->Prediction_CovarianceFir[5][5]);
	// printf("mul 1 %f|| mul 2 %f|| mul 3%f|| mul 4%f|| mul 5 %f|| mul 6%f\r\n",Mul_Result[0][0],Mul_Result[1][1],Mul_Result[2][2],Mul_Result[3][3],Mul_Result[4][4],Mul_Result[5][5]);
	 transposeSquareMatrix(Jacobian);
	 multiplyMatrices(Mul_Result,Jacobian,Fresult);
	 multiplyMatrices(Jacobianw,Qmatrix,Jacobianwresult);
	 transposeSquareMatrix(Jacobianw);
	 multiplyMatrices(Jacobianwresult,Jacobianw,Qresult);
	 addMatrices(Fresult,Qresult,EKF->Prediction_CovarianceNex);
//	 printf("mul 1 %f|| mul 2 %f|| mul 3%f|| mul 4%f|| mul 5 %f|| mul 6%f\r\n",EKF->Prediction_CovarianceNex[0][0],EKF->Prediction_CovarianceNex[1][1],EKF->Prediction_CovarianceNex[2][2],EKF->Prediction_CovarianceNex[3][3],EKF->Prediction_CovarianceNex[4][4],EKF->Prediction_CovarianceNex[5][5]);
}	
