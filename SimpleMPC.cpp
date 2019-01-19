#include "SimpleMpc.h"


SimpleMpc::SimpleMpc()
{
    Serial.println("MPC Constructor:");
    
    //calcMpcF((float*)A, (float*)C, NX, NY, NP, (float*)F);
    MatrixPrint((float*)F, NP*NY, NX);
    
}


void SimpleMpc::calculate()
{
    
    /* Read in sensor: y_sensor = */
    //y_sensor = SI_TIn;
    
    /* Generate reference trajectory: Y_ref = */
    MatrixCopy((float*)Y_ref, NP, 1, (float*)Y_ref_kn1);
    float acc = Y_ref[0][1];
    for(int i=0; i<NP-1; i++)
    {   Y_ref[i][1] = Y_ref[i+1][1];
    }
    Y_ref[NP-1][1] = acc;
    
    MatrixPrint((float*)Y_ref, NP, 1);
    
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
    MatrixPrint((float*) Temp1, row1, col2);
       
    row1 = NP;
    col1 = 1;
    row2 = NP;
    col2 = 1;
    float Temp2[row1][col1]; 
    MatrixSubtract((float*) Y_ref, (float*) Temp1, row1, col1, (float*) Temp2); // Y_ref - Temp1
    MatrixPrint((float*) Temp2, row1, col1);
    
    row1 = NP;
    col1 = NP;
    row2 = NP;
    col2 = 1;
    //float deltaU_opt[row1][col2];
    MatrixCopy((float*)deltaU_opt, row1, col2, (float*)deltaU_opt_kn1); // deltaU_opt_kn1 = deltaU_opt;
    MatrixMultiply((float*)Hinv_PhiT, (float*)Temp2, row1, col1, col2, (float*)deltaU_opt); // Hinv_PhiT * Temp2 
    MatrixPrint((float*)deltaU_opt, row1, col2);
    
    float u_opt = deltaU_opt[0][0];
    //printf("\n%f\n",u_opt);
    Serial.println();
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
    
  
    /* Observer update:
     * x_hat = (A-L*C)*x_hat_kn1 + L*y_sensor + B*u_opt; 
     * x_hat_kn1 = x_hat; */
    
    row1 = NX;
    col1 = NU;
    row2 = 1;
    col2 = 1;
    float Temp3[row1][col1];
    MatrixScale((float*)B, row1, col1, u_opt, (float*)Temp3); // B*u_opt
    MatrixPrint((float*)Temp3, row1, col1);
    
    row1 = NX;
    col1 = NU;
    row2 = 1;
    col2 = 1;
    float Temp4[row1][col1];
    MatrixScale((float*)L, row1, col1, y_sensor, (float*)Temp4); // L*y_sensor
    MatrixPrint((float*)Temp4, row1, col1);
    

    row1 = NX;
    col1 = 1;
    row2 = NY;
    col2 = NX;
    float Temp5[row1][col2];
    MatrixMultiply((float*)L, (float*)C, row1, col1, col2, (float*)Temp5); // L*C
    MatrixPrint((float*)Temp5, row1, col2);
    
    row1 = NX;
    col1 = NX;
    row2 = NX;
    col2 = NX;
    float Temp6[row1][col1]; 
    MatrixSubtract((float*) A, (float*) Temp5, row1, col1, (float*) Temp6); // A-Temp5
    MatrixPrint((float*) Temp6, row1, col1);
    
    row1 = NX;
    col1 = NX;
    row2 = NX;
    col2 = 1;
    float Temp7[row1][col2];
    MatrixMultiply((float*)Temp6, (float*)x_hat_kn1, row1, col1, col2, (float*)Temp7); // Temp6*x_hat_kn1
    MatrixPrint((float*)Temp7, row1, col2);
    
    row1 = NX;
    col1 = 1;
    row2 = NX;
    col2 = 1;
    float Temp8[row1][col1]; 
    MatrixAdd((float*)Temp7, (float*)Temp4, row1, col1, (float*)Temp8); // Temp7 + Temp4
    MatrixPrint((float*) Temp8, row1, col1);
    
    row1 = NX;
    col1 = 1;
    row2 = NX;
    col2 = 1;
    MatrixAdd((float*)Temp8, (float*)Temp3, row1, col1, (float*)x_hat); // x_hat = Temp8 + Temp3
    MatrixPrint((float*) x_hat, row1, col1);
    
    MatrixCopy((float*)x_hat, row1, col1, (float*)x_hat_kn1); // x_hat_kn1 = x_hat
    MatrixPrint((float*)x_hat_kn1, row1, col1);
    
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

void SimpleMpc::calcMpcF(float* A, float* C, int nx, int ny, int np, float* F)
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

void SimpleMpc::calcMpcPhi(float* A, float* B, float* C, int nx, int ny, int nu, int np, float* Phi)
{
    float firstColPhi[np*ny][nu];
    
    // firstColPhi(1:NY,:) = C*B;
    float Temp;
    //MatrixMultiply((float*)C, (float*)B, ny, nx, nu, (float*)Temp);
    
    firstColPhi[0][0] = Temp;
    
    float Temp1[ny][nx];
    
    // Temp = C; % [NY x NX]
    MatrixCopy((float*)C, ny, nx, (float*)Temp1);
    
    /*
    for(int i=1; i<np; i++)
    {
        // Temp1 = Temp1 * A;
        MatrixMultiply((float*)Temp1, (float*)A, ny, nx, nx, (float*)Temp2);
        MatrixCopy((float*)Temp2, ny, nx, (float*)Temp1);
        
        // Temp2 = Temp1 * B;
        MatrixMultiply((float*)Temp3, (float*)B, ny, nx, nu, (float*)Temp4);
        MatrixCopy((float*)Temp4, ny, nx, (float*)Temp3);
    
        for j=0:NY-1
            firstColPhi(i*NY+j+1) = Temp2(j+1);
        end
    }
    
    offs = 0;
    for i=0:NP*NU-1
        for k=0:NP*NY-1
            if k>=offs
                % go along rows
                Phi(k*NP+(i+1)) = firstColPhi((k-offs+1));
            end
        end
        offs = offs + 1;
    end
    Phi'
    */
}






