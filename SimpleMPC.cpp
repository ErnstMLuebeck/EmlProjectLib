#include "SimpleMpc.h"


SimpleMpc::SimpleMpc()
{
    Serial.println("MPC Constructor:");
    
    calcF((float*)A, (float*)C, NX, NY, NP, (float*)F);
    //MatrixPrint((float*)F, NP, NX);
    
    calcPhi((float*) A, (float*) B, (float*) C, NX, NY, NU, NP, (float*) Phi);
    //MatrixPrint((float*)Phi, NP, NP);
    
    float PhiT[NP][NP];
    MatrixTranspose((float*) Phi, NP, NP, (float*) PhiT);
    
    // H = PhiT*Phi + Ru;
    float Temp[NP][NP];
    MatrixMultiply((float*)PhiT, (float*)Phi, NP, NP, NP, (float*)Temp);
    //MatrixPrint((float*)Temp, NP, NP);
    
    float Ru[NP][NP] = {{0}};
    for(int i=0; i<NP; i++)
    {
        Ru[i][i] = ru;
    }
    
    float H[NP][NP];
    MatrixAdd((float*)Temp, (float*)Ru, NP, NP, (float*)H);
    
    MatrixInvert((float*) H, NP); // Hinv
    
    MatrixMultiply((float*)H, (float*)PhiT, NP, NP, NP, (float*)Hinv_PhiT);
    
    //MatrixPrint((float*)Hinv_PhiT, NP, NP);
    
}


void SimpleMpc::calculate(float _y_sensor)
{
    Serial.println("Calc MPC:");
    
    /* Read in sensor: y_sensor = */
    y_sensor = _y_sensor;
    
    /* Generate reference trajectory: Y_ref =
    MatrixCopy((float*)Y_ref, NP, 1, (float*)Y_ref_kn1);
    float acc = Y_ref[0][1];
    for(int i=0; i<NP-1; i++)
    {   Y_ref[i][1] = Y_ref[i+1][1];
    }
    Y_ref[NP-1][1] = acc;
    
    MatrixPrint((float*)Y_ref, NP, 1);
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
    MatrixMultiply((float*)F, (float*)x_hat_kn1, row1, col1, col2, (float*)Temp1); // F*x_hat_kn1 
    //MatrixPrint((float*) Temp1, row1, col2);
       
    row1 = NP;
    col1 = 1;
    row2 = NP;
    col2 = 1;
    float Temp2[row1][col1]; 
    MatrixSubtract((float*) Y_ref, (float*) Temp1, row1, col1, (float*) Temp2); // Y_ref - Temp1
    //MatrixPrint((float*) Temp2, row1, col1);
    
    row1 = NP;
    col1 = NP;
    row2 = NP;
    col2 = 1;
    //float deltaU_opt[row1][col2];
    MatrixCopy((float*)deltaU_opt, row1, col2, (float*)deltaU_opt_kn1); // deltaU_opt_kn1 = deltaU_opt;
    MatrixMultiply((float*)Hinv_PhiT, (float*)Temp2, row1, col1, col2, (float*)deltaU_opt); // Hinv_PhiT * Temp2 
    //MatrixPrint((float*)deltaU_opt, row1, col2);
    
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
    MatrixMultiply((float*)Phi, (float*)deltaU_opt, row1, col1, col2, (float*)Temp9); // Phi*deltaU_opt
    
    row1 = NP;
    col1 = NP;
    row2 = NP;
    col2 = NU;
    float Temp10[row1][col2];
    MatrixMultiply((float*)F, (float*)x_hat_kn1, row1, col1, col2, (float*)Temp10); // F*x_hat_kn1
    
    row1 = NP;
    col1 = 1;
    row2 = NP;
    col2 = 1;
    MatrixCopy((float*)Y_opt, row1, col1, (float*)Y_opt_kn1);
    MatrixAdd((float*)Temp10, (float*)Temp9, row1, col1, (float*)Y_opt); // Temp10 + Temp9

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
    MatrixScale((float*)B, row1, col1, u_opt, (float*)Temp3); // B*u_opt
    //MatrixPrint((float*)Temp3, row1, col1);
    
    row1 = NX;
    col1 = NU;
    row2 = 1;
    col2 = 1;
    float Temp4[row1][col1];
    MatrixScale((float*)L, row1, col1, y_sensor, (float*)Temp4); // L*y_sensor
    //MatrixPrint((float*)Temp4, row1, col1);
    
    row1 = NX;
    col1 = 1;
    row2 = NY;
    col2 = NX;
    float Temp5[row1][col2];
    MatrixMultiply((float*)L, (float*)C, row1, col1, col2, (float*)Temp5); // L*C
    //MatrixPrint((float*)Temp5, row1, col2);
    
    row1 = NX;
    col1 = NX;
    row2 = NX;
    col2 = NX;
    float Temp6[row1][col1]; 
    MatrixSubtract((float*) A, (float*) Temp5, row1, col1, (float*) Temp6); // A-Temp5
    //MatrixPrint((float*) Temp6, row1, col1);
    
    row1 = NX;
    col1 = NX;
    row2 = NX;
    col2 = 1;
    float Temp7[row1][col2];
    MatrixMultiply((float*)Temp6, (float*)x_hat_kn1, row1, col1, col2, (float*)Temp7); // Temp6*x_hat_kn1
    //MatrixPrint((float*)Temp7, row1, col2);
    
    row1 = NX;
    col1 = 1;
    row2 = NX;
    col2 = 1;
    float Temp8[row1][col1]; 
    MatrixAdd((float*)Temp7, (float*)Temp4, row1, col1, (float*)Temp8); // Temp7 + Temp4
    //MatrixPrint((float*) Temp8, row1, col1);
    
    row1 = NX;
    col1 = 1;
    row2 = NX;
    col2 = 1;
    MatrixAdd((float*)Temp8, (float*)Temp3, row1, col1, (float*)x_hat); // x_hat = Temp8 + Temp3

    Serial.println("x_hat = ");
    MatrixPrint((float*) x_hat, row1, col1);
    
    MatrixCopy((float*)x_hat, row1, col1, (float*)x_hat_kn1); // x_hat_kn1 = x_hat
    //MatrixPrint((float*)x_hat_kn1, row1, col1);
    
}

void SimpleMpc::MatrixMultiply(float* A, float* B, int m, int p, int n, float* C)
{   // MatrixMath.cpp Library for Matrix Math
	// A = input matrix (m x p)
	// B = input matrix (p x n)
	// m = number of rows in A
	// p = number of columns in A = number of rows in B
	// n = number of columns in B
	// C = output matrix = A*B (m x n)
	int i, j, k;
	for (i = 0; i < m; i++)
		for(j = 0; j < n; j++)
		{
			C[n * i + j] = 0;
			for (k = 0; k < p; k++)
				C[n * i + j] = C[n * i + j] + A[p * i + k] * B[n * k + j];
		}
}

void SimpleMpc::MatrixPrint(float* A, int m, int n)
{
	// A = input matrix (m x n)
	int i, j;
	
	for (i = 0; i < m; i++)
	{
		for (j = 0; j < n; j++)
		{
			//printf("%f ", A[n * i + j]);
            Serial.print(A[n * i + j],4);
            Serial.print(" ");
		}
		//printf("\n");
        Serial.println();
	}
    Serial.println();
}

void SimpleMpc::MatrixAdd(float* A, float* B, int m, int n, float* C)
{
	// A = input matrix (m x n)
	// B = input matrix (m x n)
	// m = number of rows in A = number of rows in B
	// n = number of columns in A = number of columns in B
	// C = output matrix = A+B (m x n)
	int i, j;
	for (i = 0; i < m; i++)
		for(j = 0; j < n; j++)
			C[n * i + j] = A[n * i + j] + B[n * i + j];
}


//Matrix Subtraction Routine
void SimpleMpc::MatrixSubtract(float* A, float* B, int m, int n, float* C)
{
	// A = input matrix (m x n)
	// B = input matrix (m x n)
	// m = number of rows in A = number of rows in B
	// n = number of columns in A = number of columns in B
	// C = output matrix = A-B (m x n)
	int i, j;
	for (i = 0; i < m; i++)
		for(j = 0; j < n; j++)
			C[n * i + j] = A[n * i + j] - B[n * i + j];
}

void SimpleMpc::MatrixScale(float* A, int m, int n, float k, float* C)
{
	for (int i = 0; i < m; i++)
		for (int j = 0; j < n; j++)
			C[n * i + j] = A[n * i + j] * k;
}

void SimpleMpc::MatrixCopy(float* A, int n, int m, float* B)
{
	int i, j;
	for (i = 0; i < m; i++)
		for(j = 0; j < n; j++)
		{
			B[n * i + j] = A[n * i + j];
		}
}

void SimpleMpc::MatrixTranspose(float* A, int m, int n, float* C)
{
    // A = input matrix (m x n)
    // m = number of rows in A
    // n = number of columns in A
    // C = output matrix = the transpose of A (n x m)
    int i, j;
    for (i = 0; i < m; i++)
        for(j = 0; j < n; j++)
            C[m * j + i] = A[n * i + j];
}

//Matrix Inversion Routine
// * This function inverts a matrix based on the Gauss Jordan method.
// * Specifically, it uses partial pivoting to improve numeric stability.
// * The algorithm is drawn from those presented in
//     NUMERICAL RECIPES: The Art of Scientific Computing.
// * The function returns 1 on success, 0 on failure.
// * NOTE: The argument is ALSO the result matrix, meaning the input matrix is REPLACED
int SimpleMpc::MatrixInvert(float* A, int n)
{
    // A = input matrix AND result matrix
    // n = number of rows = number of columns in A (n x n)
    int pivrow = 0;        // keeps track of current pivot row
    int k, i, j;        // k: overall index along diagonal; i: row index; j: col index
    int pivrows[n]; // keeps track of rows swaps to undo at end
    float tmp;        // used for finding max value and making column swaps
    
    for (k = 0; k < n; k++)
    {
        // find pivot row, the row with biggest entry in current column
        tmp = 0;
        for (i = k; i < n; i++)
        {
            if (abs(A[i * n + k]) >= tmp)    // 'Avoid using other functions inside abs()?'
            {
                tmp = abs(A[i * n + k]);
                pivrow = i;
            }
        }
        
        // check for singular matrix
        if (A[pivrow * n + k] == 0.0f)
        {
            Serial.println("Inversion failed due to singular matrix");
            return 0;
        }
        
        // Execute pivot (row swap) if needed
        if (pivrow != k)
        {
            // swap row k with pivrow
            for (j = 0; j < n; j++)
            {
                tmp = A[k * n + j];
                A[k * n + j] = A[pivrow * n + j];
                A[pivrow * n + j] = tmp;
            }
        }
        pivrows[k] = pivrow;    // record row swap (even if no swap happened)
        
        tmp = 1.0f / A[k * n + k];    // invert pivot element
        A[k * n + k] = 1.0f;        // This element of input matrix becomes result matrix
        
        // Perform row reduction (divide every element by pivot)
        for (j = 0; j < n; j++)
        {
            A[k * n + j] = A[k * n + j] * tmp;
        }
        
        // Now eliminate all other entries in this column
        for (i = 0; i < n; i++)
        {
            if (i != k)
            {
                tmp = A[i * n + k];
                A[i * n + k] = 0.0f; // The other place where in matrix becomes result mat
                for (j = 0; j < n; j++)
                {
                    A[i * n + j] = A[i * n + j] - A[k * n + j] * tmp;
                }
            }
        }
    }
    
    // Done, now need to undo pivot row swaps by doing column swaps in reverse order
    for (k = n - 1; k >= 0; k--)
    {
        if (pivrows[k] != k)
        {
            for (i = 0; i < n; i++)
            {
                tmp = A[i * n + k];
                A[i * n + k] = A[i * n + pivrows[k]];
                A[i * n + pivrows[k]] = tmp;
            }
        }
    }
    return 1;
}

void SimpleMpc::calcF(float* A, float* C, int nx, int ny, int np, float* F)
{
    float Temp[nx][ny];
    float Temp1[nx][ny];
    
    MatrixCopy((float*)C, ny, nx, (float*)Temp);
    
    int NumElTemp = ny*nx;
    
    for(int i=0; i<np; i++)
    {   MatrixMultiply((float*)Temp, (float*)A, ny, nx, nx, (float*)Temp1);
        MatrixCopy((float*)Temp1, ny, nx, (float*)Temp);
        
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
    MatrixMultiply((float*)C, (float*)B, ny, nx, nu, (float*)Temp);
    
    firstColPhi[0] = Temp[0];
    
    float Temp1[ny][nx];
    float Temp2[ny][nx];
    
    // Temp = C; % [NY x NX]
    MatrixCopy((float*)C, ny, nx, (float*)Temp1);
    
    float Temp3[1];
    MatrixMultiply((float*)C, (float*)B, ny, nx, nu, (float*)Temp3);
    
    firstColPhi[0] = Temp3[0];

    for(int i=1; i<np; i++)
    {
        // Temp1 = Temp1 * A;
        MatrixMultiply((float*)Temp1, (float*)A, ny, nx, nx, (float*)Temp2);
        MatrixCopy((float*)Temp2, ny, nx, (float*)Temp1); // = F
        // MatrixPrint((float*)Temp2, ny, nx); // =F
        
        // Temp2 = Temp1 * B;
        MatrixMultiply((float*)Temp2, (float*)B, ny, nx, nu, (float*)Temp3);

        //Serial.println(Temp3[0],4);
        
        firstColPhi[i] = Temp3[0];
    }
    
    //MatrixPrint((float*)firstColPhi, NP, 1);
    
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
    //MatrixPrint((float*)Phi, np, np);

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
    MatrixCopy((float*)x_hat, row1, col1, (float*)_x_hat);

    MatrixPrint((float*)_x_hat, row1, col1);
}

















