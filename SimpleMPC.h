#include <stdio.h>
#include <math.h>

void MatrixMultiply(float* A, float* B, int m, int p, int n, float* C);
void MatrixPrint(float* A, int m, int n);
void MatrixSubtract(float* A, float* B, int m, int n, float* C);
void MatrixAdd(float* A, float* B, int m, int n, float* C);
void MatrixScale(float* A, int m, int n, float k, float* C);
void MatrixCopy(float* A, int n, int m, float* B);

const int NX = 2;
const int NY = 1;
const int NU = 1;
const int NP = 10;

float A[NX][NX] ={
    {0.990703820085258, 0.0929617991474188},
    {-0.185923598294838, 0.859235982948375}};

float B[NX][NU] = {
    {-0.0995351910042629},
    {0.00929617991474188}};

const float C[NY][NX] = {
    {-0.0929617991474188, 0.929617991474187}};

const float L[NX][NY] = {
    {-0.351798484809834},
    {0.427310988590526}};
    
float y_sensor = 1;

float x_hat[NX][1] = {
    {0},
    {0}};

float x_hat_kn1[NX][1] = {
    {1},
    {2}};
    
float Y_ref[NP*NY][1] = {
    {0},
    {0},
    {0},
    {0},
    {0},
    {2},
    {2},
    {2},
    {2},
    {2}};

float delatU_opt[NP][NU];

float qy = 1;
float ru = 0.005;

const float F[NP*NY][NX] = {
    {-0.264935531551847, 0.790119332570092},
    {-0.40937447257848, 0.65427007769624},
    {-0.527213100930816, 0.524116205827117},
    {-0.619757604003169, 0.401328824896418},
    {-0.688612725037007, 0.287222385441962},
    {-0.735612676665679, 0.182777130844754},
    {-0.762756870728493, 0.088664809783386},
    {-0.77215102610405, 0.00527674397218261},
    {-0.765954042470623, -0.0672465803064527},
    {-0.746330869701793, -0.128985147381839}};

const float Hinv_PhiT[NP][NP] = {
    {1.96061274041497, 2.58267917356673, 2.38224849941096, 1.82998414068785, 1.21569053626922, 0.685530952233222, 0.287410457496048, 0.0119127697933746, -0.175931843281325, -0.314019772668268},
    {-1.11128742435152, 0.499955351548055, 1.2402795622767, 1.35670956772848, 1.15279039831961, 0.831779976537497, 0.508214602332985, 0.230761685090743, 0.00676474151444515, -0.175931843281325},
    {-0.654749008529499, -1.97197160977792, -0.291188207871189, 0.635713285762424, 0.957376902950589, 0.926463168817326, 0.727725416001169, 0.47628142137607, 0.230761685090746, 0.0119127697933757},
    {-0.311533865574384, -1.06524904928621, -2.35080047939073, -0.582409824608272, 0.442094511920818, 0.848272153019076, 0.881341421813187, 0.727725416001169, 0.50821460233299, 0.28741045749605},
    {-0.0899722568924164, -0.433000852563402, -1.1817764951109, -2.44549354414444, -0.649059274781945, 0.406377217789091, 0.848272153019075, 0.926463168817325, 0.831779976537502, 0.685530952233226},
    {0.0290490995300603, -0.0587384162281111, -0.414893301793828, -1.18022103689012, -2.45338847116297, -0.649059274781945, 0.442094511920817, 0.957376902950587, 1.15279039831961, 1.21569053626922},
    {0.0740938242358583, 0.114181284888575, 0.000807919375631898, -0.391059881842274, -1.18022103689012, -2.44549354414444, -0.582409824608273, 0.635713285762424, 1.35670956772847, 1.82998414068785},
    {0.0730962476211768, 0.151610650198517, 0.157113033815135, 0.000807919375631794, -0.414893301793829, -1.18177649511091, -2.35080047939073, -0.291188207871189, 1.2402795622767, 2.38224849941096},
    {0.0491593835674408, 0.113416492398536, 0.151610650198517, 0.114181284888575, -0.0587384162281129, -0.433000852563402, -1.06524904928621, -1.97197160977792, 0.499955351548056, 2.58267917356673},
    {0.0201114747389763, 0.0491593835674409, 0.0730962476211764, 0.074093824235859, 0.0290490995300585, -0.0899722568924163, -0.311533865574383, -0.654749008529499, -1.11128742435152, 1.96061274041497}};

int main()
{
     
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
    float deltaU_opt[row1][col2];  
    MatrixMultiply((float*)Hinv_PhiT, (float*)Temp2, row1, col1, col2, (float*)deltaU_opt); // Hinv_PhiT * Temp2 
    MatrixPrint((float*)deltaU_opt, row1, col2);
    
    float u_opt = deltaU_opt[0][0];
    printf("\n%f\n",u_opt);
  
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
    
    MatrixCopy((float*)x_hat, row1, col1, (float*)x_hat_kn1);
    MatrixPrint((float*)x_hat_kn1, row1, col1);
    
    return(1);
    
}

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

void MatrixPrint(float* A, int m, int n)
{
	// A = input matrix (m x n)
	int i, j;
	printf("\n");
	for (i = 0; i < m; i++)
	{
		for (j = 0; j < n; j++)
		{
			printf("%f ", A[n * i + j]);
		}
		printf("\n");
	}
}

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


//Matrix Subtraction Routine
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

void MatrixScale(float* A, int m, int n, float k, float* C)
{
	for (int i = 0; i < m; i++)
		for (int j = 0; j < n; j++)
			C[n * i + j] = A[n * i + j] * k;
}

void MatrixCopy(float* A, int n, int m, float* B)
{
	int i, j;
	for (i = 0; i < m; i++)
		for(j = 0; j < n; j++)
		{
			B[n * i + j] = A[n * i + j];
		}
}
