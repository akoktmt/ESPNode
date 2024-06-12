// File Server
#include "app_internal.h"
// Freertos
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
// ESP LOG
#include "esp_log.h"
#include "esp_err.h"
#include "sdkconfig.h"
//
#include "EKF_exe.h"
#include "EKF.h"
#include "Heading_Measurement.h"
#include "can.h"
#include "driver/twai.h"
//
#include "Steering_MeasurementModel.h"
//
#include "Velocity_Measurement.h"
#include "EKFmath.h"
#include <math.h>
 EKF EKFexe;
 Input Inputexe;
 Angle Angleexe;
 Heading Headingexe;
 Steering Steeringexe;
 Velocity Velocityexe;
 GPS GPSexe;
 extern twai_message_t rx_msg;
 extern volatile uint8_t CAN_flag;
float ax,ay;
float accx,accy;
extern uint8_t AccX[4];
extern uint8_t AccY[4];
extern uint8_t BNOmessage;
float EncoderVel;
float EncoderVelX, EncoderVelY;
bool Headingtrue = false;
extern bool Kalman;
uint8_t GPSrec=0;
uint8_t Velrec=0;
uint8_t Hearec =0;
float vel;
float HeadingOut;
uint8_t HeadingHandle[4];
float EKFcheckPx=0,EKFcheckPy=0;
void GPS_Exe()
{
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    for(;;)
    {
        TickType_t xCurrentTime = xTaskGetTickCount();
        Inputexe.Time = (double)(xCurrentTime - xLastWakeTime) / configTICK_RATE_HZ;
        xLastWakeTime = xCurrentTime;
       // generateRandomAcceleration(&ax,&ay);
        // Angleexe. Yaw   =  1;   //rx_msg.data[2];
        // Inputexe .Accx = ax;  //rx_msg .data[3];
        // Inputexe .Accy = ay;  //rx_msg .data[4];
        // printf("Ax %f|| Ay %f\r\n", Inputexe .Accx ,  Inputexe .Accy);
        // Inputexe .Stee = 0.5;
       // printf("Heading yaw %f\r\n",Headingexe.Yaw);
      //  if(rx_msg.identifier==0x302){
      //   Inputexe.Stee =bytes2Float(rx_msg.data);
      //   accx=rx_msg.data[4];
      //   accy=rx_msg.data[5];
      //  }
      //  if(rx_msg.identifier==0x002){
      //   EncoderVel=rx_msg.data[0];
      //  }
       // printf("Ax %f ||Ay %f\r\n",ax,ay);
      //  if(BNOmessage==1){
      //    accx=bytes2Float(AccX);
      //    accy=bytes2Float(AccY);
      //   // printf("Ax %+2.2f ||Ay %+2.2f\r\n",accx,accy);
      //  }
       //AccelerationData
        Inputexe .Accx = decompress_uint8_to_float(accx,-20.0f,20.0f);  //rx_msg .data[3]; 
      //  printf("Accx: %f\r\n",Inputexe .Accx);
        //Velocity
        Velocityexe.VelocityX=EncoderVel;
        //Yaw
        Headingexe.Yaw=Inputexe.Stee;       
        // Headingexe.Yaw=simulateYawSignal();
       // printf("YAw: %f\r\n",Headingexe.Yaw);
       //  printf("EncoderVel %f\r\n",EncoderVel);
        //GPS
        // GPSexe.GPSGetPosition[0]=10.876650;
        // GPSexe.GPSGetPosition[1]=106.803365;
        //printf("Yaw: %f\r\n", Headingexe.Yaw);
       // printf("Heading %f\r\n", Headingexe.Yaw);
       // printf("PxMea %f || PyMea %f\r\n",GPSexe.GPSGetPosition[0],GPSexe.GPSGetPosition[1]);
       // printf("Px %f || Py %f\r\n",GPSexe.GPSGetPosition[0],GPSexe.GPSGetPosition[1]);
        EKF_PredictionStep(&EKFexe,&Angleexe,&Inputexe);
      //  printf("Px_ %f Py_ %f\r\n",GPSexe.GPSGetPosition[0],GPSexe.GPSGetPosition[1]);
       // printf("PxNe %f PyNe %f, %f\r\n", EKFexe.NexPx , EKFexe.NexPy, Inputexe.Time);
      //  EKFexe.FirVelx=  EKFexe.NexVelx ;
      //   printf("Vx: %f\r\n",EKFexe.NexVelx);
        // printf("hea: %f\r\n",EKFexe.NexHea);
       // printf("%f || %f  || %f||  %f||  %f || %f\r\n", EKFexe.Prediction_CovarianceFir[0][0], EKFexe.Prediction_CovarianceFir[1][1], EKFexe.Prediction_CovarianceFir[2][2], EKFexe.Prediction_CovarianceFir[3][3], EKFexe.Prediction_CovarianceFir[4][4], EKFexe.Prediction_CovarianceFir[5][5]);
      //  printf("%f %f %f %f %f %f\r\n", EKFexe.Prediction_CovarianceNex[0][0], EKFexe.Prediction_CovarianceNex[1][1], EKFexe.Prediction_CovarianceNex[2][2], EKFexe.Prediction_CovarianceNex[3][3], EKFexe.Prediction_CovarianceNex[4][4], EKFexe.Prediction_CovarianceNex[5][5]);
       if(GPSrec==1 && EKFcheckPx!= GPSexe.GPSGetPosition[0] && EKFcheckPy!= GPSexe.GPSGetPosition[1]){
        EKFcheckPx=GPSexe.GPSGetPosition[0];
        EKFcheckPy=GPSexe.GPSGetPosition[1];
       // EFK_GPSHandleMeasurement(&GPSexe,&EKFexe);
        Kalman=true;
         printf("Px %f Py %f\r\n", EKFexe.FirPx , EKFexe.FirPy);
         printf("Px_ %f Py_ %f\r\n",GPSexe.GPSGetPosition[0],GPSexe.GPSGetPosition[1]);
       }
       else {
        EKFexe.FirPx=EKFexe.NexPx;
        EKFexe.FirPy=EKFexe.NexPy;
        printf("Px %f Py %f\r\n", EKFexe.NexPx , EKFexe.NexPy);
        printf("Px_ %f Py_ %f\r\n", GPSexe.GPSGetPosition[0] , GPSexe.GPSGetPosition[1]);
        Kalman=false; 
       }
      //   if(Velrec==1){
      //   Velocity_MeasurementModel(&EKFexe,&Velocityexe,&Angleexe);
      //   // vel=EKFexe.FirVelx;
      //   // vel = abs(vel);
      //  //  printf("Vx %f\r\n",EKFexe.FirVelx);
      //   //  printf("Px: %f || Py %f\r\n", EKFexe.FirPx , EKFexe.FirPy);
      //   }
      //   if(Hearec==1){
      //     Headingtrue = true;
      //     Heading_MeasurementModel(&EKFexe,&Headingexe,&Inputexe);
      //     EKFexe.FirStee=EKFexe.NexStee;
      //    // printf(" Heading %f\r\n",EKFexe.FirHea);
      //   //  HeadingOut=EKFexe.FirHea;
      //   //  float2Bytes(HeadingHandle,HeadingOut);
      //    // printf("%f\r\n",HeadingOut);
      //   }
      //   else
      //   {
      //      Headingtrue = false;
      //   }
       // printf("GPS %u || Vel %u || Hea %u\r\n",GPSrec,Velrec,Hearec);
         //printf("Px: %f || Py %f\r\n", EKFexe.NexPx , EKFexe.NexPy);
       // Heading_MeasurementModel(&EKFexe,&Headingexe,&Inputexe);
      //  Velocity_MeasurementModel(&EKFexe,&Velocityexe,&Angleexe);
      //  printf("H %f\r\n",EKFexe.FirHea);
        // printf("Vx %f\r\n",EKFexe.FirVelx);
        //Velocity_MeasurementModel(&EKFexe,&Velocityexe,&Angleexe);
       // printf("%f %f %f %f %f %f\r\n", EKFexe.Prediction_CovarianceNex[0][0], EKFexe.Prediction_CovarianceNex[1][1], EKFexe.Prediction_CovarianceNex[2][2], EKFexe.Prediction_CovarianceNex[3][3], EKFexe.Prediction_CovarianceNex[4][4], EKFexe.Prediction_CovarianceNex[5][5]);
      //  Heading_MeasurementModel(&EKFexe,&Headingexe,&Inputexe);
     //   printf("%f || %f  || %f||  %f||  %f || %f\r\n", EKFexe.Prediction_CovarianceFir[0][0], EKFexe.Prediction_CovarianceFir[1][1], EKFexe.Prediction_CovarianceFir[2][2], EKFexe.Prediction_CovarianceFir[3][3], EKFexe.Prediction_CovarianceFir[4][4], EKFexe.Prediction_CovarianceFir[5][5]);
     //   printf("Vx %f || Vy %f\r\n || Px: %f || Py %f\r\n",EKFexe.FirVelx, EKFexe.FirVely, EKFexe.FirPx , EKFexe.FirPy);
    // if( EKFexe.FirStee!=0){
    //     Heading_MeasurementModel(&EKFexe,&Headingexe,&Inputexe);
    //     }
     //   printf("Px %f || Py %f || Vx %f || Vy %f || H %f || S %f || Pxcov %f|| Pycov %f|| Vxcov %f|| Vycov %f|| Hcov %f|| Scov %f\r\n",EKFexe.NexPx,EKFexe.NexPy,EKFexe.NexVelx,EKFexe.NexVely,EKFexe.NexHea,EKFexe.NexStee,EKFexe.Prediction_CovarianceNex[0][0],EKFexe.Prediction_CovarianceNex[1][1],EKFexe.Prediction_CovarianceNex[2][2],EKFexe.Prediction_CovarianceNex[3][3],EKFexe.Prediction_CovarianceNex[4][4],EKFexe.Prediction_CovarianceNex[5][5]);
       // printf("Heading %f\r\n",EKFexe.FirHea);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
void Heading_Vel_Exe(){
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  for(;;){
   Inputexe .Accx = decompress_uint8_to_float(accx,-20.0f,20.0f);  //rx_msg .data[3]; 
      //  printf("Accx: %f\r\n",Inputexe .Accx);
        //Velocity
   Velocityexe.VelocityX=EncoderVel;
        //Yaw
   Headingexe.Yaw=Inputexe.Stee;  
   TickType_t xCurrentTime = xTaskGetTickCount();
   Inputexe.Time = (double)(xCurrentTime - xLastWakeTime) / configTICK_RATE_HZ;
   xLastWakeTime = xCurrentTime;
   EKF_PredictionStep(&EKFexe,&Angleexe,&Inputexe);
   if(Velrec==1){
        Velocity_MeasurementModel(&EKFexe,&Velocityexe,&Angleexe);
        // vel=EKFexe.FirVelx;
        // vel = abs(vel);
       //  printf("Vx %f\r\n",EKFexe.FirVelx);
        //  printf("Px: %f || Py %f\r\n", EKFexe.FirPx , EKFexe.FirPy);
        }
        if(Hearec==1){
          Headingtrue = true;
          Heading_MeasurementModel(&EKFexe,&Headingexe,&Inputexe);
          EKFexe.FirStee=EKFexe.NexStee;
         // printf(" Heading %f\r\n",EKFexe.FirHea);
        //  HeadingOut=EKFexe.FirHea;
        //  float2Bytes(HeadingHandle,HeadingOut);
         // printf("%f\r\n",HeadingOut);
        }
        else
        {
           Headingtrue = false;
        }
         vTaskDelay(pdMS_TO_TICKS(100)); 
}
}
void EKF_ExeInit()
{
  //Predicton covariane
  EKFexe.CovPx=0.000002; 
	EKFexe.CovPy=0.000002; 
	EKFexe.CovVelx=0.1;
	EKFexe.CovHea=0.2; 
	EKFexe.CovStee=0.1; 
  EKF_Init(&EKFexe,&Inputexe);
   // printf("%f %f %f %f %f %f\r\n", EKFexe.Prediction_CovarianceNex[0][0], EKFexe.Prediction_CovarianceNex[1][1], EKFexe.Prediction_CovarianceNex[2][2], EKFexe.Prediction_CovarianceNex[3][3], EKFexe.Prediction_CovarianceNex[4][4], EKFexe.Prediction_CovarianceNex[5][5]);
  // printf("%f || %f  || %f||  %f||  %f || %f\r\n", EKFexe.Prediction_CovarianceFir[0][0], EKFexe.Prediction_CovarianceFir[1][1], EKFexe.Prediction_CovarianceFir[2][2], EKFexe.Prediction_CovarianceFir[3][3], EKFexe.Prediction_CovarianceFir[4][4], EKFexe.Prediction_CovarianceFir[5][5]);
    EKFexe.FirVelx=0; 
    EKFexe.FirStee=0;
    //10.870400, 106.802198//10.869763, 106.802328
    EKFexe.FirPx=10.869327;
    EKFexe.FirPy= 106.802869; 

	  EKFexe.FirHea=0; 
    //
    Inputexe.Time=1;
    //
    Heading_Init(&Headingexe);
    Headingexe.Covariane=0.025;
    //
    Steering_Init(&Steeringexe);
    Steeringexe.Covariane=0.025;
    //
    Velocity_Init(&Velocityexe);
    Velocityexe.CovarianeVx=0.02;
    GPS_Init(&GPSexe);
    GPSexe.GPSCovariance[0][0]=0.000005;
    GPSexe.GPSCovariance[1][1]=0.000005;
   // printf("%f || %f  || %f||  %f||  %f || %f\r\n", EKFexe.Prediction_CovarianceFir[0][0], EKFexe.Prediction_CovarianceFir[1][1], EKFexe.Prediction_CovarianceFir[2][2], EKFexe.Prediction_CovarianceFir[3][3], EKFexe.Prediction_CovarianceFir[4][4], EKFexe.Prediction_CovarianceFir[5][5]);
    xTaskCreate(GPS_Exe, "GPS_Exe", 1048 * 5, NULL, 10, NULL);
    xTaskCreate(Heading_Vel_Exe, "Heading_Exe", 1048 * 5, NULL, 11, NULL);
}
