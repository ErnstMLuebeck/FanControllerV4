#ifndef STATESPACEMODEL_H
#define STATESPACEMODEL_H

#include <Arduino.h>
#include <math.h>

#define NX 2
#define NY 1
#define NU 1
#define NZ 1

class StateSpaceModel
{

public:
    StateSpaceModel();
    void calculate(float* u_k);
    void getStates(float* _x_k);
    void getOutputs(float* _y_k);
    
private:
    void MatrixMultiply(float* A, float* B, int m, int p, int n, float* C);
    void MatrixPrint(float* A, int m, int n);
    void MatrixSubtract(float* A, float* B, int m, int n, float* C);
    void MatrixAdd(float* A, float* B, int m, int n, float* C);
    void MatrixScale(float* A, int m, int n, float k, float* C);
    void MatrixCopy(float* A, int n, int m, float* B);
    void MatrixTranspose(float* A, int m, int n, float* C);
    int MatrixInvert(float* A, int n);
    
    float A[NX][NX] = {
        {0.990703820085258, 0.0929617991474188},
        {-0.185923598294838, 0.859235982948375}};
    
    float B[NX][NU] = {
        {-0.0995351910042629},
        {0.00929617991474188}};
    
    float C[NY][NX] = {
        {-0.0929617991474188, 0.929617991474187}};

    float x_kn1[NX][1] = {{0}};  
    float y_k[NY][1] = {{0}}; 
};


#endif
















