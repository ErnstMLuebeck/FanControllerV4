#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include <Arduino.h>
#include <math.h>

#define NX 2
#define NY 1
#define NU 1
#define NZ 1

class KalmanFilter
{

public:
    KalmanFilter();
    void calculate(float* x_kalm, float* x_sens, float* u_kn1);
    
private:
    void MatrixMultiply(float* A, float* B, int m, int p, int n, float* C);
    void MatrixPrint(float* A, int m, int n);
    void MatrixSubtract(float* A, float* B, int m, int n, float* C);
    void MatrixAdd(float* A, float* B, int m, int n, float* C);
    void MatrixScale(float* A, int m, int n, float k, float* C);
    void MatrixCopy(float* A, int n, int m, float* B);
    void MatrixTranspose(float* A, int m, int n, float* C);
    int MatrixInvert(float* A, int n);
    
    void calcF(float* A, float* C, int nx, int ny, int np, float* F);
    void calcPhi(float* A, float* B, float* C, int nx, int ny, int nu, int np, float* Phi);
    
    float A[NX][NX] ={
        {0.990703820085258, 0.0929617991474188},
        {-0.185923598294838, 0.859235982948375}};
    
    float AT[NX][NX];
    
    float B[NX][NU] = {
        {-0.0995351910042629},
        {0.00929617991474188}};
    
    float C[NY][NX] = {
        {-0.0929617991474188, 0.929617991474187}};

    float Q[NX][NX] = {{0}};
    float R[NX][NX] = {{0}};
    float H[NX][NX] = {{0}};
    float P[NX][NX] = {{0}};
    float P_kn1[NX][NX] = {{1, 0},{0, 1}};
    
    float x_kn1[NX][1] = {{1},{1}};
     
};


#endif
















