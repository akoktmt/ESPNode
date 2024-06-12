/*
 * math.c
 *
 *  Created on: Nov 22, 2023
 *      Author: win 10
 */
#include <EKF.h>
#include "EKFmath.h"
#include <float.h>
#include <stdlib.h>
#include <time.h>

void assignMatrix(float sourceMatrix[SIZE][SIZE], float destinationMatrix[SIZE][SIZE]) {
    int i, j;
    for (i = 0; i < SIZE; ++i) {
        for (j = 0; j < SIZE; ++j) {
            destinationMatrix[i][j] = sourceMatrix[i][j];
        }
    }
}

void subtractMatrices(float firstMatrix[SIZE][SIZE], float secondMatrix[SIZE][SIZE], float result[SIZE][SIZE], int row, int column) {
    for (int i = 0; i < row; ++i) {
        for (int j = 0; j < column; ++j) {
            result[i][j] = firstMatrix[i][j] - secondMatrix[i][j];
        }
    }
}

void multiplyMatrices(float A[N][N], float B[N][N], float C[N][N]) {
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            C[i][j] = 0;
            for (int k = 0; k < N; k++) {
                C[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}

void multiplyMatrices_vel(float firstMatrix[ROWS1][COLS1], float secondMatrix[ROWS2][COLS2], float resultMatrix[ROWS1][COLS2]) {
    for (int i = 0; i < ROWS1; i++) {
        for (int j = 0; j < COLS2; j++) {
            resultMatrix[i][j] = 0;  // Initialize the result matrix with 0
            for (int k = 0; k < COLS1; k++) {
                resultMatrix[i][j] += firstMatrix[i][k] * secondMatrix[k][j];
            }
        }
    }
}

void transposeSquareMatrix(float matrix[N][N]) {
    int temp;
    for (int i = 0; i < N; i++) {
        for (int j = i + 1; j < N; j++) {
            temp = matrix[i][j];
            matrix[i][j] = matrix[j][i];
            matrix[j][i] = temp;
        }
    }
}

void A_matrix_vector_multiply(float a[SIZE], float b[SIZE][SIZE], float c[SIZE]) {
    // Khởi tạo vector kết quả c
    int i, j;
    // Khởi tạo mảng kết quả với giá trị 0
    for (i = 0; i < SIZE; i++) {
        c[i] = 0;
    }

    // Nhân ma trận
    for (i = 0; i < SIZE; i++) {
        for (j = 0; j < SIZE; j++) {
            c[i] += a[j] * b[j][i];
        }
    }
}
float dot_product(float vec1[SIZE], float vec2[SIZE]) {
    float result = 0;
    for (int i = 0; i < SIZE; i++) {
        result += vec1[i] * vec2[i];
    }
    return result;
}

void matrix_vector_multiply_A(float C[SIZE][SIZE], float A[SIZE], float D[SIZE]) {
    for (int i = 0; i < SIZE; i++) {
        D[i] = 0; // Khởi tạo giá trị ban đầu cho mỗi phần tử của vector kết quả
        for (int j = 0; j < SIZE; j++) {
            D[i] += C[i][j] * A[j];
        }
    }
}

void scalar_multiply_vector(float vector[SIZE], float scalar) {
    for (int i = 0; i < SIZE; i++) {
        vector[i] *= scalar;
    }
}

void add_vectors(float vector1[SIZE], float vector2[SIZE], float result[SIZE]) {
    for (int i = 0; i < SIZE; i++) {
        result[i] = vector1[i] + vector2[i];
    }
}

void multiply_transpose_matrix_with_matrix(float C[SIZE], float A[SIZE], float result[SIZE][SIZE]) {
    for (int i = 0; i < SIZE; i++) {
        for (int j = 0; j < SIZE; j++) {
            result[i][j] = C[i] * A[j];
        }
    }
}
float findMaxValue(float matrix[N][N]) {
    float maxVal = -DBL_MAX;
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            if (matrix[i][j] > maxVal) {
                maxVal = matrix[i][j];
            }
        }
    }
    return maxVal;
}
void scaleMatrix(float matrix[N][N], float scale) {
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            matrix[i][j] *= scale;
        }
    }
}
void addMatrices(float A[N][N], float B[N][N], float C[N][N]) {
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            C[i][j] = A[i][j] + B[i][j];
        }
    }
}

void multiplyVectorWithScalar(float AT[N], float scalar, float result[N]) {
    for (int i = 0; i < N; ++i) {
        result[i] = AT[i] * scalar;
    }
}

float invertMatrix(float matrix[2][2], float inverseMatrix[2][2]) {
    float a = matrix[0][0], b = matrix[0][1];
    float c = matrix[1][0], d = matrix[1][1];
    
    // Tính định thức của ma trận
    float det = a * d - b * c;
    
    // Kiểm tra định thức có bằng 0 (không có ma trận nghịch đảo)
    if (det == 0) {
        return -1; // Trả về -1 nếu ma trận không khả nghịch
    }
    
    // Tính nghịch đảo của ma trận
    inverseMatrix[0][0] = d / (float)det;
    inverseMatrix[0][1] = -b / (float)det;
    inverseMatrix[1][0] = -c / (float)det;
    inverseMatrix[1][1] = a / (float)det;
    return 0; // Trả về 0 nếu thành công
}
void multiplyVectorByMatrix(float vector[5], float matrix[5][5], float resultVector[5]) {
    // Initializing resultVector with zeros
    for (int j = 0; j < 5; j++) {
        resultVector[j] = 0;
        // Calculating the dot product of the vector with the j-th column of matrix
        for (int i = 0; i < 5; i++) {
            resultVector[j] += vector[i] * matrix[i][j];
        }
    }
}
void multiplyMatrixByVector(float matrix[5][5], float vector[5], float resultVector[5]) {
    // Initializing resultVector with zeros
    for (int i = 0; i < 5; i++) {
        resultVector[i] = 0;
        // Calculating the dot product of the i-th row of matrix with the vector
        for (int j = 0; j < 5; j++) {
            resultVector[i] += matrix[i][j] * vector[j];
        }
    }
}

void multiplyVectorByScalar(float vector[5], float scalar, float resultVector[5]) {
    for (int i = 0; i < 5; i++) {
        resultVector[i] = vector[i] * scalar;
    }
}
void transposeMatrix_vel(float original[ROWS][COLS], float transposed[COLS][ROWS]) {
    for (int i = 0; i < ROWS; i++) {
        for (int j = 0; j < COLS; j++) {
            transposed[j][i] = original[i][j];
        }
    }
}
void multiplyMatrices_velspeed(float firstMatrix[4][5], float secondMatrix[5][4], float resultMatrix[4][4]) {
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            resultMatrix[i][j] = 0;  // Khởi tạo giá trị bằng 0
            for (int k = 0; k < 5; k++) {
                resultMatrix[i][j] += firstMatrix[i][k] * secondMatrix[k][j];
            }
        }
    }
}