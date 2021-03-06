#include "StateSpaceModel.h"

/*
% Plant model --------------------------------------------------------------    
w_kn1 = sigmaQ*randn(Nx,1); % process noise p(w)~N(0,Q)
x_plant = A*x_plant_kn1 + B*du(:,k) + w_kn1;
x_true(:,k) = x_plant_kn1;
x_plant_kn1 = x_plant;
*/

StateSpaceModel::StateSpaceModel()
{   
}


void StateSpaceModel::calculate(float* u_k)
{  
    /* x_k = A*x_kn1 + B*u_k; */

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
    float B_u_k[row1][col2];
    MatrixMultiply((float*)B, (float*)u_k, row1, col1, col2, (float*)B_u_k); 
    //MatrixPrint((float*)B_u_k, row1, col2);

    /* y_k = C*x_kn1; */

    MatrixCopy((float*)y_k, NY, 1, (float*)y_kn1);

    row1 = NY;
    col1 = NX;
    row2 = NX;
    col2 = 1;
    MatrixMultiply((float*)C, (float*)x_kn1, row1, col1, col2, (float*)y_k); 
    //MatrixPrint((float*)y_k, row1, col2);

    Serial.print("y_plant = ");
    Serial.println(y_k[0][0]);

    row1 = NX;
    col1 = 1;
    row2 = NX;
    col2 = 1;
    MatrixAdd((float*)A_x_kn1, (float*)B_u_k, row1, col1, (float*)x_kn1);
    //MatrixPrint((float*) x_kn1, row1, col1);

    Serial.println("x_plant = ");
    MatrixPrint((float*) x_kn1, NX, 1);

}

void StateSpaceModel::getStates(float* _x_k)
{
    int row1 = NX;
    int col1 = 1;
    MatrixCopy((float*)x_kn1, row1, col1, (float*)_x_k);

    Serial.println("StateSpaceModel states:");
    MatrixPrint((float*) x_kn1, row1, col1);
}

void StateSpaceModel::getOutputs(float* _y_k)
{
    int row1 = NY;
    int col1 = 1;
    MatrixCopy((float*)y_k, row1, col1, (float*)_y_k);

    //MatrixPrint((float*) y_k, row1, col1);
}

void StateSpaceModel::getOutputsKn1(float* _y_kn1)
{
    int row1 = NY;
    int col1 = 1;
    MatrixCopy((float*)y_kn1, row1, col1, (float*)_y_kn1);

}

void StateSpaceModel::MatrixMultiply(float* A, float* B, int m, int p, int n, float* C)
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

void StateSpaceModel::MatrixPrint(float* A, int m, int n)
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

void StateSpaceModel::MatrixAdd(float* A, float* B, int m, int n, float* C)
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
void StateSpaceModel::MatrixSubtract(float* A, float* B, int m, int n, float* C)
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

void StateSpaceModel::MatrixScale(float* A, int m, int n, float k, float* C)
{
	for (int i = 0; i < m; i++)
		for (int j = 0; j < n; j++)
			C[n * i + j] = A[n * i + j] * k;
}

void StateSpaceModel::MatrixCopy(float* A, int n, int m, float* B)
{
	int i, j;
	for (i = 0; i < m; i++)
		for(j = 0; j < n; j++)
		{
			B[n * i + j] = A[n * i + j];
		}
}

void StateSpaceModel::MatrixTranspose(float* A, int m, int n, float* C)
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
int StateSpaceModel::MatrixInvert(float* A, int n)
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