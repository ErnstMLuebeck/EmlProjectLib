#include "SimpleMpc.h"


SimpleMpc::SimpleMpc()
{
    Serial.println("MPC Constructor:");
    
    calcF((float*)A, (float*)C, NX, NY, NP, (float*)F);
    //ProjectLib::MatrixPrint((float*)F, NP, NX);
    
    calcPhi((float*) A, (float*) B, (float*) C, NX, NY, NU, NP, (float*) Phi);
    //ProjectLib::MatrixPrint((float*)Phi, NP, NP);
    
    float PhiT[NP][NP];
    ProjectLib::MatrixTranspose((float*) Phi, NP, NP, (float*) PhiT);
    
    // H = PhiT*Phi + Ru;
    float Temp[NP][NP];
    ProjectLib::MatrixMultiply((float*)PhiT, (float*)Phi, NP, NP, NP, (float*)Temp);
    //ProjectLib::MatrixPrint((float*)Temp, NP, NP);
    
    float Ru[NP][NP] = {{0}};
    for(int i=0; i<NP; i++)
    {
        Ru[i][i] = ru;
    }
    
    float H[NP][NP];
    ProjectLib::MatrixAdd((float*)Temp, (float*)Ru, NP, NP, (float*)H);
    
    ProjectLib::MatrixInvert((float*) H, NP); // Hinv
    
    ProjectLib::MatrixMultiply((float*)H, (float*)PhiT, NP, NP, NP, (float*)Hinv_PhiT);
    
    //ProjectLib::MatrixPrint((float*)Hinv_PhiT, NP, NP);
    
}


void SimpleMpc::calculate(float _y_sensor)
{
    Serial.println("Calc MPC:");
    
    /* Read in sensor: y_sensor = */
    y_sensor = _y_sensor;
    
    /* Generate reference trajectory: Y_ref =
    ProjectLib::MatrixCopy((float*)Y_ref, NP, 1, (float*)Y_ref_kn1);
    float acc = Y_ref[0][1];
    for(int i=0; i<NP-1; i++)
    {   Y_ref[i][1] = Y_ref[i+1][1];
    }
    Y_ref[NP-1][1] = acc;
    
    ProjectLib::MatrixPrint((float*)Y_ref, NP, 1);
    */
     
     
    /* Calculate MPC equation: u_opt = */
    
    /* Update Observer: x_hat = */
    
    /* Apply control value: actuator = u_opt */
     
    int row1;
    int col1;
    int row2;
    int col2;
    
    /* MPC Equations:
     * deltaU_opt = Hinv * PhiT * (Y_ref - F * x_hat_kn1) */
       
    row1 = NP*NY;
    col1 = NX;
    row2 = NX;
    col2 = 1;
    float Temp1[row1][col2];    
    ProjectLib::MatrixMultiply((float*)F, (float*)x_hat_kn1, row1, col1, col2, (float*)Temp1); // F*x_hat_kn1 
    //ProjectLib::MatrixPrint((float*) Temp1, row1, col2);
       
    row1 = NP;
    col1 = 1;
    row2 = NP;
    col2 = 1;
    float Temp2[row1][col1]; 
    ProjectLib::MatrixSubtract((float*) Y_ref, (float*) Temp1, row1, col1, (float*) Temp2); // Y_ref - Temp1
    //ProjectLib::MatrixPrint((float*) Temp2, row1, col1);
    
    row1 = NP;
    col1 = NP;
    row2 = NP;
    col2 = 1;
    //float deltaU_opt[row1][col2];
    ProjectLib::MatrixCopy((float*)deltaU_opt, row1, col2, (float*)deltaU_opt_kn1); // deltaU_opt_kn1 = deltaU_opt;
    ProjectLib::MatrixMultiply((float*)Hinv_PhiT, (float*)Temp2, row1, col1, col2, (float*)deltaU_opt); // Hinv_PhiT * Temp2 
    //ProjectLib::MatrixPrint((float*)deltaU_opt, row1, col2);
    
    float u_opt = deltaU_opt[0];
    //printf("\n%f\n",u_opt);
    Serial.print("u_opt = ");
    Serial.println(u_opt,4);
    
    //Y_opt = F*x_hat_kn1 + Phi*deltaU_opt;
    row1 = NP;
    col1 = NP;
    row2 = NP;
    col2 = NU;
    float Temp9[row1][col2];
    ProjectLib::MatrixMultiply((float*)Phi, (float*)deltaU_opt, row1, col1, col2, (float*)Temp9); // Phi*deltaU_opt
    
    row1 = NP;
    col1 = NP;
    row2 = NP;
    col2 = NU;
    float Temp10[row1][col2];
    ProjectLib::MatrixMultiply((float*)F, (float*)x_hat_kn1, row1, col1, col2, (float*)Temp10); // F*x_hat_kn1
    
    row1 = NP;
    col1 = 1;
    row2 = NP;
    col2 = 1;
    ProjectLib::MatrixCopy((float*)Y_opt, row1, col1, (float*)Y_opt_kn1);
    ProjectLib::MatrixAdd((float*)Temp10, (float*)Temp9, row1, col1, (float*)Y_opt); // Temp10 + Temp9

    Serial.print("y_opt = ");
    Serial.println(Y_opt[0]);
    
  
    /* Observer update:
     * x_hat = (A-L*C)*x_hat_kn1 + L*y_sensor + B*u_opt; 
     * x_hat_kn1 = x_hat; */
    
    row1 = NX;
    col1 = NU;
    row2 = 1;
    col2 = 1;
    float Temp3[row1][col1];
    ProjectLib::MatrixScale((float*)B, row1, col1, u_opt, (float*)Temp3); // B*u_opt
    //ProjectLib::MatrixPrint((float*)Temp3, row1, col1);
    
    row1 = NX;
    col1 = NU;
    row2 = 1;
    col2 = 1;
    float Temp4[row1][col1];
    ProjectLib::MatrixScale((float*)L, row1, col1, y_sensor, (float*)Temp4); // L*y_sensor
    //ProjectLib::MatrixPrint((float*)Temp4, row1, col1);
    
    row1 = NX;
    col1 = 1;
    row2 = NY;
    col2 = NX;
    float Temp5[row1][col2];
    ProjectLib::MatrixMultiply((float*)L, (float*)C, row1, col1, col2, (float*)Temp5); // L*C
    //ProjectLib::MatrixPrint((float*)Temp5, row1, col2);
    
    row1 = NX;
    col1 = NX;
    row2 = NX;
    col2 = NX;
    float Temp6[row1][col1]; 
    ProjectLib::MatrixSubtract((float*) A, (float*) Temp5, row1, col1, (float*) Temp6); // A-Temp5
    //ProjectLib::MatrixPrint((float*) Temp6, row1, col1);
    
    row1 = NX;
    col1 = NX;
    row2 = NX;
    col2 = 1;
    float Temp7[row1][col2];
    ProjectLib::MatrixMultiply((float*)Temp6, (float*)x_hat_kn1, row1, col1, col2, (float*)Temp7); // Temp6*x_hat_kn1
    //ProjectLib::MatrixPrint((float*)Temp7, row1, col2);
    
    row1 = NX;
    col1 = 1;
    row2 = NX;
    col2 = 1;
    float Temp8[row1][col1]; 
    ProjectLib::MatrixAdd((float*)Temp7, (float*)Temp4, row1, col1, (float*)Temp8); // Temp7 + Temp4
    //ProjectLib::MatrixPrint((float*) Temp8, row1, col1);
    
    row1 = NX;
    col1 = 1;
    row2 = NX;
    col2 = 1;
    ProjectLib::MatrixAdd((float*)Temp8, (float*)Temp3, row1, col1, (float*)x_hat); // x_hat = Temp8 + Temp3

    Serial.println("x_hat = ");
    ProjectLib::MatrixPrint((float*) x_hat, row1, col1);
    
    ProjectLib::MatrixCopy((float*)x_hat, row1, col1, (float*)x_hat_kn1); // x_hat_kn1 = x_hat
    //ProjectLib::MatrixPrint((float*)x_hat_kn1, row1, col1);
    
}

void SimpleMpc::calcF(float* A, float* C, int nx, int ny, int np, float* F)
{
    float Temp[nx][ny];
    float Temp1[nx][ny];
    
    ProjectLib::MatrixCopy((float*)C, ny, nx, (float*)Temp);
    
    int NumElTemp = ny*nx;
    
    for(int i=0; i<np; i++)
    {   ProjectLib::MatrixMultiply((float*)Temp, (float*)A, ny, nx, nx, (float*)Temp1);
        ProjectLib::MatrixCopy((float*)Temp1, ny, nx, (float*)Temp);
        
        for(int j=0; j<NumElTemp; j++)
        {   /* Todo: fix linear indexing for MIMO MPC */
            F[i*NumElTemp+j] = Temp[0][j];
        }
    }
}

void SimpleMpc::calcPhi(float* A, float* B, float* C, int nx, int ny, int nu, int np, float* Phi)
{
    //delay(100);
    float firstColPhi[np];
    
    // firstColPhi(1:NY,:) = C*B;
    float Temp[1];
    ProjectLib::MatrixMultiply((float*)C, (float*)B, ny, nx, nu, (float*)Temp);
    
    firstColPhi[0] = Temp[0];
    
    float Temp1[ny][nx];
    float Temp2[ny][nx];
    
    // Temp = C; % [NY x NX]
    ProjectLib::MatrixCopy((float*)C, ny, nx, (float*)Temp1);
    
    float Temp3[1];
    ProjectLib::MatrixMultiply((float*)C, (float*)B, ny, nx, nu, (float*)Temp3);
    
    firstColPhi[0] = Temp3[0];

    for(int i=1; i<np; i++)
    {
        // Temp1 = Temp1 * A;
        ProjectLib::MatrixMultiply((float*)Temp1, (float*)A, ny, nx, nx, (float*)Temp2);
        ProjectLib::MatrixCopy((float*)Temp2, ny, nx, (float*)Temp1); // = F
        // ProjectLib::MatrixPrint((float*)Temp2, ny, nx); // =F
        
        // Temp2 = Temp1 * B;
        ProjectLib::MatrixMultiply((float*)Temp2, (float*)B, ny, nx, nu, (float*)Temp3);

        //Serial.println(Temp3[0],4);
        
        firstColPhi[i] = Temp3[0];
    }
    
    //ProjectLib::MatrixPrint((float*)firstColPhi, NP, 1);
    
    int offs = 0;
    for(int ii=0; ii<np*nu; ii++)
    {
        for(int k=0; k<np*ny; k++)
        {
            if(k>=offs)
            {
                Phi[k*NP+ii] = firstColPhi[k-offs];
            }
        }
        offs += 1;
    }
    
    //delay(100);
    //ProjectLib::MatrixPrint((float*)Phi, np, np);

}

void SimpleMpc::setYrefReceeding(float _y_ref)
{
    float acc = Y_ref[0];
    for(int i=0; i<NP-1; i++)
    {   Y_ref[i] = Y_ref[i+1];
    }
    Y_ref[NP-1] = _y_ref;
}

void SimpleMpc::setYref(float* _Y_ref)
{
    for(int i=0; i<NP; i++)
    {   
        Y_ref[i] = _Y_ref[i];
    }
}

void SimpleMpc::getYopt(float* _Y_opt)
{
    for (int ii = 0; ii < NP; ii++)
        _Y_opt[ii] = Y_opt[ii];
}

void SimpleMpc::getUopt(float* _U_opt)
{
    for (int ii = 0; ii < NP; ii++)
        _U_opt[ii] = deltaU_opt[ii];
}

void SimpleMpc::getYoptKn1(float* _Y_opt_kn1)
{
    for (int ii = 0; ii < NP; ii++)
        _Y_opt_kn1[ii] = Y_opt_kn1[ii];
}

void SimpleMpc::getUoptKn1(float* _U_opt_kn1)
{
    for (int ii = 0; ii < NP; ii++)
        _U_opt_kn1[ii] = deltaU_opt_kn1[ii];
}

void SimpleMpc::getXhat(float* _x_hat)
{   /*
    for (int ii = 0; ii < NX; ii++)
        _x_hat[ii] = x_hat[ii];
    */

    int row1 = NX;
    int col1 = 1;
    ProjectLib::MatrixCopy((float*)x_hat, row1, col1, (float*)_x_hat);

    ProjectLib::MatrixPrint((float*)_x_hat, row1, col1);
}

















