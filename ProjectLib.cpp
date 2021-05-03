#include "ProjectLib.h"

namespace ProjectLib {

/**
 * 2D look-up table (1D axis and 1D data) which uses linear interpolation between breakpoints 
 * and saturation in case the input is outside the axis range.
 * 
 * @param axis (float*) pointer to breakpoint vector
 * @param data (float*) pointer to curve data vector
 * @param size (uint8_t) length of axis and data vector
 * @param input (float) input to be mapped on data 
 * 
 */
float LookupTable(float axis[], float data[], uint8_t size, float input)
{   /* Axis must be strictly monotonic increasing */

    float output = 0;
    float factor = 0;

    /* saturate under-/overflow */
    if(input <= axis[0]) 
    {   //Serial.println("Underflow");
        return(data[0]);
    }
    else if(input >= axis[size-1]) 
    {   //Serial.println("Overflow");
        return(data[size-1]);
    }  
    else
    {   //Serial.println("Interpolation");
        for(int i=0; i < size; i++)
        {   
            if(input == axis[i]) 
            {   //Serial.println("Exact breakpoint");
                return(data[i]);
            }            
            else if(axis[i] > input)
            {   /* linear interpolation */

                // Serial.print("Between Breakpoints [");
                // Serial.print(axis[i-1],3);
                // Serial.print(", ");
                // Serial.print(axis[i],3);
                // Serial.println("]");

                factor = (input-axis[i-1]) / (axis[i]-axis[i-1]);
                output = factor * (data[i]-data[i-1]) + data[i-1];
                return(output);
            }
        }
    }

    /* should not be reached */
    return(input);
}

float saturate(float in, float LimLwr, float LimUpr)
{   float ret;

    ret = in;
    if(in >= LimUpr) ret = LimUpr;
    if(in <= LimLwr) ret = LimLwr;
    return(ret);
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/**
 * Check input against interval. If it is inside the range, return 1.
 * 
 * @param in [-], signal input
 * @param LimLwr [-], lower range limit
 * @param LimUpr [-], upper range limit
 */
boolean checkInterval(float in, float LimLwr, float LimUpr)
{   
    boolean ret = 1;
    if((in > LimUpr) || (in < LimLwr)) ret = 0;
    return(ret);
}

boolean getBit16(uint16_t BitWord, uint16_t PosnBit)
{
    if(BitWord & (1 << PosnBit))
    {
        return(1);
    }
    else
    {
        return(0);
    }
}

uint16_t setBit16(uint16_t BitWord, uint16_t PosnBit)
{
    BitWord |=  1 << PosnBit;
    return(BitWord);
}
 
uint16_t clearBit16(uint16_t BitWord, uint16_t PosnBit)
{
    BitWord &= ~(1 << PosnBit);
    return(BitWord);
}

uint16_t putBit16(uint16_t BitWord, uint16_t PosnBit, boolean BitNew)
{
    if(BitNew)
    {
        BitWord |=  1 << PosnBit;
    }
    else
    {
        BitWord &= ~(1 << PosnBit);
    }

    return(BitWord);
}
 
uint16_t toggleBit16(uint16_t BitWord, uint16_t PosnBit)
{
    if(BitWord & (1 << PosnBit)) 
    {
        BitWord &= ~(1 << PosnBit);
    }
    else 
    {   
        BitWord |=  1 << PosnBit;
    }
    return(BitWord);
}

/** 
 * Multiply two matrices with matching dimensions
 * 
 * @param A pointer to first matrix
 * @param B pointer to second matrix
 * @param m number of rows in A
 * @param p number of cols in A = number of rows in B
 * @param n number of columns in B
 * @param C pointer to output matrix (m x n)
 */
void MatrixMultiply(float* A, float* B, int m, int p, int n, float* C)
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

/** 
 * Print out matrix in console
 * 
 * @param A pointer to matrix
 * @param m number of rows
 * @param n number of columns
 */
void MatrixPrint(float* A, int m, int n)
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

/** 
 * Add two matrices of equal size
 * 
 * @param A pointer to first matrix
 * @param B pointer to second matrix
 * @param m number of rows
 * @param n number of columns
 * @param C pointer to output matrix
 */
void MatrixAdd(float* A, float* B, int m, int n, float* C)
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

/** 
 * Subtract two matrices of equal size
 * 
 * @param A pointer to first matrix
 * @param B pointer to second matrix
 * @param m number of rows
 * @param n number of columns
 * @param C pointer to output matrix
 */
void MatrixSubtract(float* A, float* B, int m, int n, float* C)
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

/** 
 * Scale a matrix of size (m x n)
 * 
 * @param A pointer to first matrix
 * @param m number of rows
 * @param n number of columns
 * @param k scalar
 * @param C pointer to output matrix
 */
void MatrixScale(float* A, int m, int n, float k, float* C)
{
	for (int i = 0; i < m; i++)
		for (int j = 0; j < n; j++)
			C[n * i + j] = A[n * i + j] * k;
}

/** 
 * Copy a matrix of size (m x n) to a new location
 * 
 * @param A pointer to matrix to be copied
 * @param m number of rows
 * @param n number of columns
 * @param C pointer to destination matrix
 */
void MatrixCopy(float* A, int n, int m, float* B)
{
	int i, j;
	for (i = 0; i < m; i++)
		for(j = 0; j < n; j++)
		{
			B[n * i + j] = A[n * i + j];
		}
}

/**
 * Transpose matrix (flip along main diagonal)
 * 
 * @param A pointer to input matrix (m x n)
 * @param m number of rows
 * @param n number of columns
 * @param C pointer to output matrix (n x m)
 */
void MatrixTranspose(float* A, int m, int n, float* C)
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

/** Matrix Inversion Routine
 * This function inverts a matrix based on the Gauss Jordan method.
 * Specifically, it uses partial pivoting to improve numeric stability.
 * The algorithm is drawn from those presented in NUMERICAL RECIPES: The Art of Scientific Computing.
 * The function returns 1 on success, 0 on failure.
 * NOTE: The argument is ALSO the result matrix, meaning the input matrix is REPLACED
 * 
 * @param A matrix input AND output of inversion
 * @param n number of rows = columns
 */
int MatrixInvert(float* A, int n)
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

/**
 * Transform from 3-phase coordinate system (A, B, C) to an orthogonal
 * Alpha/Beta coordinate system without math.sqrt(3)
 * 
 * @param Ia [A], current through phase A
 * @param Ib [A], current through phase B
 * @param Ic [A], current through phase C
 * @return Ialpha [A], current in transformed coordinates: alpha
 * @return Ibeta [A], current in transformed coordinates: beta
 */
void ClarkeTransform(float Ia, float Ib, float Ic, float *Ialpha, float *Ibeta)
{   
    /* Phase A = Alpha */
    /* sqrt(3) = 1.7320508076 */
    /* 1/sqrt(3) = 0.5773502692 */
    *Ialpha = Ia;
    *Ibeta = 0.5773502692 * Ia + 2*0.5773502692 * Ib;
}

/**
 * Transformation from alpha/beta frame (stationary) into I/Q frame (rotating) using
 * math functions (inefficient!)
 * 
 * Note: Execution time is not constant!
 * 
 * @param Ialpha [A], current in transformed coordinates: alpha
 * @param Ibeta [A], current in transformed coordinates: beta
 * @param Theta [rad], phase angle of the rotor (0..2pi)
 * @return Id [A], direct current
 * @return Iq [A], quadrature current (proportional to motor torque)
 */
void ParkTransform(float Ialpha, float Ibeta, float Theta, float *Id, float *Iq)
{   /* Transform from a stationary coordinate system, to a rotating 
       coordinate system, linked to rotation angle Theta */

    *Id = Ialpha * cos(Theta) + Ibeta * sin(Theta);
    *Iq = -Ialpha * sin(Theta) + Ibeta * cos(Theta);
}

/**
 * Transformation from I/Q frame (rotating) into alpha/beta frame (stationary) using
 * math functions (inefficient!)
 * 
 * Note: Execution time is not constant!
 * 
 * @param Vd [V], direct voltage
 * @param Vq [V], quadrature voltage
 * @param Theta [rad], phase angle of the rotor (0..2pi)
 * @return Valpha [V], voltage in transformed coordinates: alpha
 * @return Vbeta [V], voltage in transformed coordinates: beta
 */
void ParkTransformInverse(float Vd, float Vq, float Theta, float *Valpha, float *Vbeta)
{
    *Valpha = Vd * cos(Theta) - Vq * sin(Theta);
    *Vbeta = Vd * sin(Theta) + Vq * cos(Theta);
}

/**
 * Transform from alpha/beta, orthogonal frame into 3-phase coordinate 
 * system (A, B, C) without math.sqrt(3)
 * 
 * @param Valpha [V], voltage in transformed coordinates: alpha
 * @param Vbeta [V], voltage in transformed coordinates: beta
 * @return Va [V], voltage at phase A
 * @return Vb [V], voltage at phase B
 * @return Vc [V], voltage at phase C
 */
void ClarkeTransformInverse(float Valpha, float Vbeta, float *Va, float *Vb, float *Vc)
{
    /* sqrt(3) = 1.7320508076 */
    *Va = Valpha;
    *Vb = (-Valpha + 1.73205 * Vbeta)/2;
    *Vc = (-Valpha - 1.73205 * Vbeta)/2;
}

} /* namespace ProjectLib */