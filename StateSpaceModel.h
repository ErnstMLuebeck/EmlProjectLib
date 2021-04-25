#ifndef STATESPACEMODEL_H
#define STATESPACEMODEL_H

#include <Arduino.h>
#include "ProjectLib.h"

/**
 * State Space Model (MIMO compatible) with state observer:
 * 
 * \f$ x[k] = A \cdot x[k-1] + B \cdot u[k] \f$
 * 
 * \f$ y[k] = C \cdot x[k-1] \f$
 * 
 * State observer:
 * 
 * \f$ \hat{x}[k] = (A-L\cdot C) \cdot \hat{x}[k-1] + B \cdot u[k] + L \cdot y_{sens}\f$
 * 
 * @author E. M. Luebeck
 * @date 2021-04-23
 */
class StateSpaceModel
{

public:
    StateSpaceModel();
    void initStateSpaceModel(float* Aa, float* Bb, float* Cc, int _Nx, int _Nu, int _Ny, float _Ts);
    void initStateObserver(float* Ll, float* pp);

    void calculate(float* u_k);
    void calculateObserver(float* u_k, float* y_sens);

    void setStates(float* _x0);
    void getStates(float* _x_k);
    void setObserverStates(float* _x0_hat);
    void getObserverStates(float* _x_hat_k);

    void getOutputs(float* _y_k);
    void getOutputsKn1(float* _y_kn1);
    
private:
    /* Plant Model Parameters */
    float Ts = {0.00100};
    int Nx = 1;
    int Nu = 1;
    int Ny = 1;

    /* Memory is allocated in the init functions */
    float* A;
    float* B;
    float* C;
    float* L;
    float* p;

    float* x_kn1;  
    float* y_k; 
    float* y_kn1;

    float* x_hat_kn1;
    float* L_C; /* L*C */
    float* A_L_C; /* A-L*C */

    // float A[3][3] = {{0.70970, -0.03520, 0.00000},
    // {0.05609, 0.99858, 0.00000},
    // {0.00003, 0.00100, 1.00000}};
    // float B[3][2] = {{0.10057, 0.00220},
    // {0.00330, -0.12491},
    // {0.00000, -0.00006}};

    // float C[1][3] = {{0.00001, 0.00050, 1.00000}};
    // float D[1][2] = {{0.00000, -0.00003}};

    /* Observer Parameters */
    // float p[1][3] = {{0.80000, 0.81000, 0.82000}};
    // float L[3][1] = {{-22.65057}, {25.86176}, {0.26568}};

    /* Pre-calculated matrices */
    // float L_C[NX][NX] = {{0}}; /* L*C */
    // float A_L_C[NX][NX] = {{0}};; /* A-L*C */

    // float x_kn1[NX][1] = {{0}};  
    // float y_k[NY][1] = {{0}}; 
    // float y_kn1[NY][1] = {{0}};

    // float x_hat_kn1[NX][1] = {{10}, {20}, {33}};
    
};


#endif /* STATESPACEMODEL_H */
















