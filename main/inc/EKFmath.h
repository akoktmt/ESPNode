/*
 * math.h
 *
 *  Created on: Nov 22, 2023
 *      Author: win 10
 */

#ifndef EKFMATH_H_
#define EKFMATH_H_
#include <EKF.h>
#define N 5 
#define ROWS1 4
#define COLS1 5
#define ROWS2 5
#define COLS2 5
#define ROWS 4
#define COLS 5
void transposeSquareMatrix(float matrix[N][N]);
void assignMatrix(float sourceMatrix[SIZE][SIZE], float destinationMatrix[SIZE][SIZE]);
void subtractMatrices(float firstMatrix[SIZE][SIZE], float secondMatrix[SIZE][SIZE], float result[SIZE][SIZE], int row, int column);
void multiplyMatrices(float A[N][N], float B[N][N], float C[N][N]);
void A_matrix_vector_multiply(float a[SIZE], float b[SIZE][SIZE], float c[SIZE]) ;
float dot_product(float vec1[SIZE], float vec2[SIZE]);
void matrix_vector_multiply_A(float C[SIZE][SIZE], float A[SIZE], float D[SIZE]);
void multiplyVectorWithScalar(float AT[N], float scalar, float result[N]);
void add_vectors(float vector1[SIZE], float vector2[SIZE], float result[SIZE]);
void multiply_transpose_matrix_with_matrix(float C[SIZE], float A[SIZE], float result[SIZE][SIZE]);
void scaleMatrix(float matrix[N][N], float scale);
float findMaxValue(float matrix[N][N]);
void generateRandomAcceleration(float *ax, float *ay);
void addMatrices(float A[N][N], float B[N][N], float C[N][N]);
void scalar_multiply_vector(float vector[SIZE], float scalar);
float invertMatrix(float matrix[2][2], float inverseMatrix[2][2]);
void multiplyMatrixByVector(float matrix[5][5], float vector[5], float resultVector[5]);
void multiplyVectorByMatrix(float vector[5], float matrix[5][5], float resultVector[5]);
void multiplyVectorByScalar(float vector[5], float scalar, float resultVector[5]);
void multiplyMatrices_vel(float firstMatrix[ROWS1][COLS1], float secondMatrix[ROWS2][COLS2], float resultMatrix[ROWS1][COLS2]);
void transposeMatrix_vel(float original[ROWS][COLS], float transposed[COLS][ROWS]);
void multiplyMatrices_velspeed(float firstMatrix[4][5], float secondMatrix[5][4], float resultMatrix[4][4]);
#endif /* EKFMATH_H_ */
