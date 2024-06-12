#include "EKFmath.h"
#include <math.h>
#include "Heading_Measurement.h"
#include <stdint.h>
#include <string.h>
#include "driver/twai.h"
#include "app_internal.h"
#include "can.h"
extern twai_message_t rx_msg;
extern volatile uint8_t CAN_flag;

float decompress_uint8_to_float(uint8_t value, float minRange, float maxRange) {
    float normalizedValue = (float)value / 255.0f;
    return normalizedValue * (maxRange - minRange) + minRange;
}
float bytes2Float(uint8_t bytes[4])
{
  union
  {
    uint8_t bytes[4];
    float a;
  } thing;
  memcpy(thing.bytes, bytes, 4);
  return thing.a;
}
void float2Bytes( uint8_t bytes_temp[4],float float_variable){
  union {
    float a;
    unsigned char bytes[4];
  } thing;
  thing.a = float_variable;
  memcpy(bytes_temp, thing.bytes, 4);
}
void Heading_Init(Heading *Heading)
{
  Heading->Covariane = 0;
  Heading->Yaw = 0;
}

void Heading_MeasurementModel(EKF *EKF, Heading *Heading, Input *Input)
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
    InovationVelx = Heading->Yaw - EKF->NexHea;
    // printf(" IN %f\r\n",InovationVelx);
    Error[4]=InovationVelx;
    //-----------------------------------------------------------imatrix
    I_Matrix[0][0] = 1;
    I_Matrix[1][1] = 1;
    I_Matrix[2][2] = 1;
    I_Matrix[3][3] = 1;
    I_Matrix[4][4] = 1;
    //-----------------------------------------------------------
    JacobianVelx[4]=1;
    //-----------------------------------------------------------
    multiplyVectorByMatrix(JacobianVelx,EKF->Prediction_CovarianceNex,JacobianVelxRel);
	  InnovationCov = dot_product(JacobianVelxRel,JacobianVelx) + Heading->Covariane;
    // printf(" IN %f\r\n",InnovationCov); 
    //-----------------------------------------------------------
    Inovation_= 1 / InnovationCov;
  //  printf(" IN___ %f\r\n",Inovation_); 
	  multiplyMatrixByVector(EKF->Prediction_CovarianceNex,JacobianVelx,Kalman);
	  multiplyVectorByScalar(Kalman,Inovation_,KalmanGian);
    // printf(" %f|| %f|| %f|| %f|| %f\r\n",KalmanGian[0],KalmanGian[1],KalmanGian[2],KalmanGian[3],KalmanGian[4]);
    //  printf(" %f|| %f|| %f|| %f|| %f\r\n",Kalman[0],Kalman[1],Kalman[2],Kalman[3],Kalman[4]);
    //   printf(" %f|| %f|| %f|| %f|| %f\r\n",JacobianVelx[0],JacobianVelx[1],JacobianVelx[2],JacobianVelx[3],JacobianVelx[4]);
    //   printf(" %f|| %f|| %f|| %f|| %f\r\n",EKF->Prediction_CovarianceNex[0][0],EKF->Prediction_CovarianceNex[1][1],EKF->Prediction_CovarianceNex[2][2],EKF->Prediction_CovarianceNex[3][3],EKF->Prediction_CovarianceNex[4][4]);
    //------------------------------------------------------------
    EKF->FirHea = EKF->NexHea + dot_product(KalmanGian,Error);
  //   printf(" EKF->FirHea %f\r\n",EKF->FirHea);
    //------------------------------------------------------------
    multiply_transpose_matrix_with_matrix(KalmanGian, JacobianVelx, CovarianX);
    subtractMatrices(I_Matrix, CovarianX, Covarian_matrixX, 5, 5);
    multiplyMatrices(Covarian_matrixX, EKF->Prediction_CovarianceNex, EKF->Prediction_CovarianceFir);
  // float Inovation=0;
  // float InovationCovariance=0;
  // float KalmanGain[5]={0};
  // float Jacobian[5]={0};
  // float I_result[5]={0};
  // float S_result[5]={0};
  // float C_KalmanGain[5]={0};
  // float State[5]={0};
  // float P_result[5][5]={0};
  // float P_result_1[5][5]={0};
  // float I_Matrix[5][5]={0};

  // I_Matrix[0][0] = 1;
  // I_Matrix[1][1] = 1;
  // I_Matrix[2][2] = 1;
  // I_Matrix[3][3] = 1;
  // I_Matrix[4][4] = 1;

  // State[0] = EKF->NexVelx;
  // State[1] = EKF->NexStee;
  // State[2] = EKF->NexPx;
  // State[3] = EKF->NexPy;
  // State[4] = EKF->NexHea;

  // Inovation = Heading->Yaw - EKF->NexHea;

  // Jacobian[0] = 0;
  // Jacobian[1] = 0;
  // Jacobian[2] = 0;
  // Jacobian[3] = 0;
  // Jacobian[4] = 1;

  // A_matrix_vector_multiply(Jacobian, EKF->Prediction_CovarianceNex, I_result);

  // InovationCovariance = dot_product(I_result, Jacobian) + Heading->Covariane;
  // // printf("Inovation %f\r\n",InovationCovariance);

  // matrix_vector_multiply_A(EKF->Prediction_CovarianceNex, Jacobian, KalmanGain);

  // scalar_multiply_vector(KalmanGain, (1 / InovationCovariance));
  // //printf("K %f\r\n",KalmanGain[0]);
  // C_KalmanGain[0] = KalmanGain[0];
  // C_KalmanGain[1] = KalmanGain[1];
  // C_KalmanGain[2] = KalmanGain[2];
  // C_KalmanGain[3] = KalmanGain[3];
  // C_KalmanGain[4] = KalmanGain[4];
  // C_KalmanGain[5] = KalmanGain[5];

  // scalar_multiply_vector(KalmanGain, Inovation);
  // //printf("K+ %f\r\n",KalmanGain[0]);
  // add_vectors(State, KalmanGain, S_result);
  // //convert to radian
  // // float latconvert = S_result[0] * PI /180;
  // // float longconvert= S_result[1] * PI /180;
  // EKF->FirVelx = S_result[0];
  // EKF->FirStee = S_result[1];
  // EKF->FirPx  =   S_result[2];
  // EKF->FirPy =   S_result[3];
  // EKF->FirHea =  S_result[4];

  // multiply_transpose_matrix_with_matrix(C_KalmanGain, Jacobian, P_result);

  // subtractMatrices(I_Matrix,P_result,P_result_1,5,5);

  // multiplyMatrices(P_result_1,EKF->Prediction_CovarianceNex,EKF->Prediction_CovarianceFir);

}