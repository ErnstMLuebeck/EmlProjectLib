#include "StateSpaceModel.h"

/**
 * Nothing is done here.
 */
StateSpaceModel::StateSpaceModel()
{   

}

/**
 * Initialize the SSM with all matrices and dimensions. Be careful what to pass this method, 
 * since it can easily mess with undesired memory sections!
 * 
 * @param Aa pointer to source matrix
 * @param Bb pointer to source matrix
 * @param Cc pointer to source matrix
 * @param Nx number of states
 * @param Nu number of inputs (actuators)
 * @param Ny outputs 
 */ 
void StateSpaceModel::initStateSpaceModel(float* Aa, float* Bb, float* Cc, int _Nx, int _Nu, int _Ny, float _Ts)
{
    Nx = _Nx;
    Nu = _Nu;
    Ny = _Ny;
    Ts = _Ts;

    A = (float*)calloc(sizeof(float), Nx * Nx);
    B = (float*)calloc(sizeof(float), Nx * Nu);
    C = (float*)calloc(sizeof(float), Ny * Nx);

    x_kn1 = (float*)calloc(sizeof(float), Nx * 1);
    y_k = (float*)calloc(sizeof(float), Ny * 1);
    y_kn1 = (float*)calloc(sizeof(float), Ny * 1);

    ProjectLib::MatrixCopy((float*)Aa, Nx, Nx, A);
    // MatrixPrint(A, Nx, Nx);

    ProjectLib::MatrixCopy((float*)Bb, Nx, Nu, B);
    // MatrixPrint(B, Nx, Nu);

    ProjectLib::MatrixCopy((float*)Cc, Ny, Nx, C);
    // MatrixPrint(C, Ny, Nx);

}

/**
 * Initialize the state observer with all matrices and dimensions. Be careful what to pass this method, 
 * since it can easily mess with undesired memory sections!
 * 
 * @param Ll pointer to source matrix
 * @param pp pointer to pole matrix
 */ 
void StateSpaceModel::initStateObserver(float* Ll, float* pp)
{
    L = (float*)calloc(sizeof(float), Nx * 1);
    p = (float*)calloc(sizeof(float), 1 * Nx);

    L_C = (float*)calloc(sizeof(float), Nx * Nx); /* L*C */
    A_L_C = (float*)calloc(sizeof(float), Nx * Nx); /* A-L*C */

    x_hat_kn1 = (float*)calloc(sizeof(float), Nx * 1);

    ProjectLib::MatrixCopy((float*)Ll, Nx, 1, L);
    // MatrixPrint(L, Nx, 1);

    ProjectLib::MatrixCopy((float*)pp, 1, Nx, p);
    // MatrixPrint(p, 1, Nx);

    /* Pre-calculate arrays */
    int row1 = Nx;
    int col1 = 1;
    int row2 = 1;
    int col2 = Nx;
    ProjectLib::MatrixMultiply((float*)L, (float*)C, row1, col1, col2, (float*)L_C); 
    // MatrixPrint((float*)L_C, row1, col2);

    row1 = Nx;
    col1 = Nx;
    row2 = Nx;
    col2 = Nx;
    ProjectLib::MatrixSubtract((float*)A, (float*)L_C, row1, col1, (float*)A_L_C);
    // MatrixPrint((float*) A_L_C, row1, col1);
}

/**
 * Calculate the new system outputs based on the new inputs. The outputs and states can then be 
 * obtained by the respective get* functions. Standard form of a state-space model:
 * 
 * \f$ x[k] = A \cdot x[k-1] + B \cdot u[k] \f$
 * 
 * \f$ y[k] = C \cdot x[k-1] \f$
 *  
 * @param u_k [-], system input vector (can also be a vector of size [1x1])
 */
void StateSpaceModel::calculate(float* u_k)
{  
    /* x_k = A*x_kn1 + B*u_k; */

    int row1 = Nx;
    int col1 = Nx;
    int row2 = Nx;
    int col2 = 1;
    float A_x_kn1[row1][col2];
    ProjectLib::MatrixMultiply((float*)A, (float*)x_kn1, row1, col1, col2, (float*)A_x_kn1); 
    //MatrixPrint((float*)A_x_kn1, row1, col2);

    row1 = Nx;
    col1 = Nu;
    row2 = Nu;
    col2 = 1;
    float B_u_k[row1][col2];
    ProjectLib::MatrixMultiply((float*)B, (float*)u_k, row1, col1, col2, (float*)B_u_k); 
    //MatrixPrint((float*)B_u_k, row1, col2);

    /* y_k = C*x_kn1; */

    ProjectLib::MatrixCopy((float*)y_k, Ny, 1, (float*)y_kn1);

    row1 = Ny;
    col1 = Nx;
    row2 = Nx;
    col2 = 1;
    ProjectLib::MatrixMultiply((float*)C, (float*)x_kn1, row1, col1, col2, (float*)y_k); 
    //MatrixPrint((float*)y_k, row1, col2);

    row1 = Nx;
    col1 = 1;
    row2 = Nx;
    col2 = 1;
    ProjectLib::MatrixAdd((float*)A_x_kn1, (float*)B_u_k, row1, col1, (float*)x_kn1);
    //MatrixPrint((float*) x_kn1, row1, col1);

    // Serial.println("x_plant = ");
    // MatrixPrint((float*) x_kn1, Nx, 1);

}

/**
 * Calculate the new observer based on the new inputs and measured plant output. 
 * The observer outputs and states can then be obtained by the respective get* functions. 
 * Standard matrix equations of a state observer:
 * 
 * \f$ \hat{x}[k] = (A-L\cdot C) \cdot \hat{x}[k-1] + B \cdot u[k] + L \cdot y_{sens}\f$
 *  
 * @param u_k [-], system input vector (can also be a vector of size [1x1])
 * @param y_sens [-], measured plant output
 */
void StateSpaceModel::calculateObserver(float* u_k, float* y_sens)
{  
    /* x_hat_k = (A-L*C) * x_hat_kn1 + B*u_k + L*y_sens; */

    /* (A-L*C) is pre-calcuated in the constructor */

    int row1 = Nx;
    int col1 = Nx;
    int row2 = Nx;
    int col2 = 1;
    float A_L_C_x_hat_kn1[row1][col2];
    ProjectLib::MatrixMultiply((float*)A_L_C, (float*)x_hat_kn1, row1, col1, col2, (float*)A_L_C_x_hat_kn1); 
    //MatrixPrint((float*)A_L_C_x_hat_kn1, row1, col2);

    row1 = Nx;
    col1 = 1;
    row2 = 1; /* only 1 sensor! */
    col2 = 1;
    float L_y_sens[row1][col2];
    ProjectLib::MatrixMultiply((float*)L, (float*)y_sens, row1, col1, col2, (float*)L_y_sens); 
    //MatrixPrint((float*)L_y_sens, row1, col2);

    row1 = Nx;
    col1 = Nu;
    row2 = Nu;
    col2 = 1;
    float B_u_k[row1][col2];
    ProjectLib::MatrixMultiply((float*)B, (float*)u_k, row1, col1, col2, (float*)B_u_k); 
    //MatrixPrint((float*)B_u_k, row1, col2);

    /* y_k = C*x_kn1; */
    // ProjectLib::MatrixCopy((float*)y_k, Ny, 1, (float*)y_kn1);
    // row1 = Ny;
    // col1 = Nx;
    // row2 = Nx;
    // col2 = 1;
    // ProjectLib::MatrixMultiply((float*)C, (float*)x_kn1, row1, col1, col2, (float*)y_k); 
    //MatrixPrint((float*)y_k, row1, col2);

    row1 = Nx;
    col1 = 1;
    row2 = Nx;
    col2 = 1;
    float temp[row1][col2]; /* (A-L*C)*x_hat_kn1 + L*y_sens */
    ProjectLib::MatrixAdd((float*)A_L_C_x_hat_kn1, (float*)L_y_sens, row1, col1, (float*)temp);
    //MatrixPrint((float*) temp, row1, col1);

    row1 = Nx;
    col1 = 1;
    row2 = Nx;
    col2 = 1;
    ProjectLib::MatrixAdd((float*)temp, (float*)B_u_k, row1, col1, (float*)x_hat_kn1);
    //MatrixPrint((float*) x_kn1, row1, col1);

}

/**
 * Get states of the state space system.
 * 
 * @return _x_k [-], last calculated system state vector
 */
void StateSpaceModel::getStates(float* _x_k)
{
    int row1 = Nx;
    int col1 = 1;
    ProjectLib::MatrixCopy((float*)x_kn1, row1, col1, (float*)_x_k);

}

/**
 * Set states of the state space system.
 * 
 * @return _x_kn1 [-], state vector, mainly used for initialization
 */
void StateSpaceModel::setStates(float* _x_kn1)
{
    int row1 = Nx;
    int col1 = 1;
    ProjectLib::MatrixCopy((float*)_x_kn1, row1, col1, (float*)x_kn1);

}

/**
 * Get states of the state observer.
 * 
 * @return _x_hat_k [-], last calculated observer state vector
 */
void StateSpaceModel::getObserverStates(float* _x_hat_k)
{
    int row1 = Nx;
    int col1 = 1;
    ProjectLib::MatrixCopy((float*)x_hat_kn1, row1, col1, (float*)_x_hat_k);

}

void StateSpaceModel::setObserverStates(float* _x_hat_kn1)
{
    int row1 = Nx;
    int col1 = 1;
    ProjectLib::MatrixCopy((float*)_x_hat_kn1, row1, col1, (float*)x_hat_kn1);

}

void StateSpaceModel::getOutputs(float* _y_k)
{
    int row1 = Ny;
    int col1 = 1;
    ProjectLib::MatrixCopy((float*)y_k, row1, col1, (float*)_y_k);

    //MatrixPrint((float*) y_k, row1, col1);
}

void StateSpaceModel::getOutputsKn1(float* _y_kn1)
{
    int row1 = Ny;
    int col1 = 1;
    ProjectLib::MatrixCopy((float*)y_kn1, row1, col1, (float*)_y_kn1);

}