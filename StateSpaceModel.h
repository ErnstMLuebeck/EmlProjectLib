#ifndef STATESPACEMODEL_H
#define STATESPACEMODEL_H

/* State Space Model (MIMO compatible) with observer */

#include <Arduino.h>
#include <math.h>

#define NX 3
#define NY 1
#define NU 2
#define NZ 1

class StateSpaceModel
{

public:
    StateSpaceModel();
    void calculate(float* u_k);
    void calculateObserver(float* u_k, float* y_sens);

    void setStates(float* _x0);
    void getStates(float* _x_k);
    void setObserverStates(float* _x0_hat);
    void getObserverStates(float* _x_hat_k);

    void getOutputs(float* _y_k);
    void getOutputsKn1(float* _y_kn1);
    
private:
    void MatrixMultiply(float* A, float* B, int m, int p, int n, float* C);
    void MatrixPrint(float* A, int m, int n);
    void MatrixSubtract(float* A, float* B, int m, int n, float* C);
    void MatrixAdd(float* A, float* B, int m, int n, float* C);
    void MatrixScale(float* A, int m, int n, float k, float* C);
    void MatrixCopy(float* A, int n, int m, float* B);
    void MatrixTranspose(float* A, int m, int n, float* C);
    int MatrixInvert(float* A, int n);

    /* Plant Model Parameters */
    float Ts = {0.00100};
    int Nx = {3};
    int Nu = {2};
    int Ny = {1};

    float A[3][3] = {{0.70970, -0.03520, 0.00000},
    {0.05609, 0.99858, 0.00000},
    {0.00003, 0.00100, 1.00000}};
    float B[3][2] = {{0.10057, 0.00220},
    {0.00330, -0.12491},
    {0.00000, -0.00006}};

    float C[1][3] = {{0.00001, 0.00050, 1.00000}};
    float D[1][2] = {{0.00000, -0.00003}};

    /* Observer Parameters */
    float p[1][3] = {{0.80000, 0.81000, 0.82000}};
    float L[3][1] = {{-22.65057},
    {25.86176},
    {0.26568}};

    /* Pre-calculated matrices */
    float L_C[NX][NX] = {{0}}; /* L*C */
    float A_L_C[NX][NX]; /* A-L*C */

    float x_kn1[NX][1] = {{0}};  
    float y_k[NY][1] = {{0}}; 
    float y_kn1[NY][1] = {{0}};

    float x_hat_kn1[NX][1] = {{10}, {20},{33}};
    
};


#endif
















