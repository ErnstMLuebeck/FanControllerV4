#include "KalmanFilter.h"

/* 
% MATLAB Implementation
function [x_kalm, P, K] = kalmanfilter(x_kn1, P_kn1, x_sens, u_kn1, A, B, Q, R, H)

   % Prediction for state vector and covariance
   x_hat = A*x_kn1 + B*u_kn1;
   P = A*P_kn1*A' + Q;

   % Compute Kalman gain factor
   S = (H*P*H'+R);
   K = P*H'*S^-1;

   % Correction based on observation
   x_kalm = x_hat + K*(x_sens-H*x_hat);
   P = P - K*H*P;
   
end
*/

KalmanFilter::KalmanFilter()
{   
    int row1 = NX;
    int col1 = NX;
    MatrixTranspose((float*) A, row1, col1, (float*) AT);

    row1 = NZ;
    col1 = NX;
    MatrixTranspose((float*) H, row1, col1, (float*) HT);
}


void KalmanFilter::calculate(float* x_sens, float* u_kn1)
{  
    /* x_hat = A*x_kn1 + B*u_kn1; */

    int row1 = NX;
    int col1 = NX;
    int row2 = NX;
    int col2 = 1;
    float A_x_kn1[row1][col2];
    MatrixMultiply((float*)A, (float*)x_kn1, row1, col1, col2, (float*)A_x_kn1); 
    //MatrixPrint((float*)A_x_kn1, row1, col2);

    row1 = NX;
    col1 = NU;
    row2 = NU;
    col2 = 1;
    float B_u_kn1[row1][col2];
    MatrixMultiply((float*)B, (float*)u_kn1, row1, col1, col2, (float*)B_u_kn1); 
    //MatrixPrint((float*)B_u_kn1, row1, col2);

    row1 = NX;
    col1 = 1;
    row2 = NX;
    col2 = 1;
    float x_hat[row1][col1]; 
    MatrixAdd((float*)A_x_kn1, (float*)B_u_kn1, row1, col1, (float*)x_hat);
    //MatrixPrint((float*) x_hat, row1, col1);

    /* P = A*P_kn1*A' + Q; */

    row1 = NX;
    col1 = NX;
    row2 = NX;
    col2 = NX;
    float A_P_kn1[row1][col2];
    MatrixMultiply((float*)A, (float*)P_kn1, row1, col1, col2, (float*)A_P_kn1); 
    //MatrixPrint((float*)A_P_kn1, row1, col2);

    row1 = NX;
    col1 = NX;
    row2 = NX;
    col2 = NX;
    float A_P_kn1_AT[row1][col2];
    MatrixMultiply((float*)A_P_kn1, (float*)AT, row1, col1, col2, (float*)A_P_kn1_AT); 
    //MatrixPrint((float*)A_P_kn1_AT, row1, col2);

    row1 = NX;
    col1 = NX;
    row2 = NX;
    col2 = NX;
    MatrixAdd((float*)A_P_kn1_AT, (float*)Q, row1, col1, (float*)P);
    //MatrixPrint((float*) P, row1, col1);

    /* S = (H*P*H'+R); */

    row1 = NX;
    col1 = NX;
    row2 = NX;
    col2 = NZ;
    float P_HT[row1][col2];
    MatrixMultiply((float*)P, (float*)HT, row1, col1, col2, (float*)P_HT); 
    //MatrixPrint((float*)P_HT, row1, col2);

    row1 = NZ;
    col1 = NX;
    row2 = NX;
    col2 = NZ;
    float H_P_HT[row1][col2];
    MatrixMultiply((float*)H, (float*)P_HT, row1, col1, col2, (float*)H_P_HT); 
    //MatrixPrint((float*)H_P_HT, row1, col2);

    row1 = NZ;
    col1 = NZ;
    row2 = NZ;
    col2 = NZ;
    float S[NZ][NZ];
    MatrixAdd((float*)H_P_HT, (float*)R, row1, col1, (float*)S);
    //MatrixPrint((float*) S, row1, col1);

    /* K = P*H'*S^-1; */

    MatrixInvert((float*) S, NZ); // Sinv
    //MatrixPrint((float*) S, row1, col1);

    row1 = NX;
    col1 = NZ;
    row2 = NZ;
    col2 = NZ;
    MatrixMultiply((float*)P_HT, (float*)S, row1, col1, col2, (float*)K); 
    //MatrixPrint((float*)K, row1, col2);

    /* x_kalm = x_hat + K*(x_sens-H*x_hat); */

    row1 = NZ;
    col1 = NX;
    row2 = NX;
    col2 = 1;
    float H_x_hat[row1][col2];
    MatrixMultiply((float*)H, (float*)x_hat, row1, col1, col2, (float*)H_x_hat); 
    //MatrixPrint((float*)H_x_hat, row1, col2);

    row1 = NZ;
    col1 = 1;
    row2 = NZ;
    col2 = 1;
    float Temp1[row1][col1]; 
    MatrixSubtract((float*)x_sens, (float*)H_x_hat, row1, col1, (float*) Temp1);
    //MatrixPrint((float*) Temp1, row1, col1);

    row1 = NX;
    col1 = NZ;
    row2 = NZ;
    col2 = 1;
    float Temp2[row1][col2];
    MatrixMultiply((float*)K, (float*)Temp1, row1, col1, col2, (float*)Temp2); 
    //MatrixPrint((float*)Temp2, row1, col2);

    row1 = NX;
    col1 = 1;
    MatrixAdd((float*)x_hat, (float*)Temp2, row1, col1, (float*)x_kn1); // = x_kalm
    //MatrixPrint((float*) x_kn1, row1, col1);
    
    /* P = P - K*H*P; */

    row1 = NZ;
    col1 = NX;
    row2 = NX;
    col2 = NX;
    float H_P[row1][col2];
    MatrixMultiply((float*)H, (float*)P, row1, col1, col2, (float*)H_P); 
    //MatrixPrint((float*)H_P, row1, col2);

    row1 = NX;
    col1 = NZ;
    row2 = NZ;
    col2 = NX;
    float K_H_P[row1][col2];
    MatrixMultiply((float*)K, (float*)H_P, row1, col1, col2, (float*)K_H_P); 
    //MatrixPrint((float*)K_H_P, row1, col2);

    row1 = NX;
    col1 = NX;
    MatrixSubtract((float*)P, (float*)K_H_P, row1, col1, (float*) P_kn1);
    //MatrixPrint((float*) P_kn1, row1, col1);

}

void KalmanFilter::getStates(float* x)
{
    int row1 = NX;
    int col1 = 1;
    MatrixCopy((float*)x_kn1, row1, col1, (float*)x);

    Serial.println("KalmanFilter states:");
    MatrixPrint((float*) x_kn1, row1, col1);
}

void KalmanFilter::MatrixMultiply(float* A, float* B, int m, int p, int n, float* C)
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

void KalmanFilter::MatrixPrint(float* A, int m, int n)
{
	// A = input matrix (m x n)
	int i, j;
	//printf("\n");
    Serial.println();
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
}

void KalmanFilter::MatrixAdd(float* A, float* B, int m, int n, float* C)
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
void KalmanFilter::MatrixSubtract(float* A, float* B, int m, int n, float* C)
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

void KalmanFilter::MatrixScale(float* A, int m, int n, float k, float* C)
{
	for (int i = 0; i < m; i++)
		for (int j = 0; j < n; j++)
			C[n * i + j] = A[n * i + j] * k;
}

void KalmanFilter::MatrixCopy(float* A, int n, int m, float* B)
{
	int i, j;
	for (i = 0; i < m; i++)
		for(j = 0; j < n; j++)
		{
			B[n * i + j] = A[n * i + j];
		}
}

void KalmanFilter::MatrixTranspose(float* A, int m, int n, float* C)
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
int KalmanFilter::MatrixInvert(float* A, int n)
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