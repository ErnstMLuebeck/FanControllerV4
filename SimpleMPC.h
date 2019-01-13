
/*
 
% MATLAB implementation of unconstrained MPC
 
function [u_opt, Y_opt, x_hat, y_hat] = SisoMpcSimple(Y_ref, y_sensor, A, B, C, L, F, Hinv_PhiT, Phi)
%#codegen
Y_ref = Y_ref(:);
nx = size(A,1);
persistent x_hat_kn1;
if isempty(x_hat_kn1)
% initial observer states
x_hat_kn1 = zeros(nx,1);
end
% d/d deltaU (J) = 0 -->
deltaU_opt = Hinv_PhiT * (Y_ref - F * x_hat_kn1);
Y_opt = F*x_hat_kn1 + Phi*deltaU_opt; % optional calculation
%receeding horizon control
u_opt = deltaU_opt(1);
% observer update
x_hat = (A-L*C)*x_hat_kn1 + L*y_sensor + B*u_opt;
y_hat = C*x_hat_kn1;
x_hat_kn1 = x_hat;
*/

void calcMPC()
{
    /*
    const int NX = 3;
    const int NY = 1;
    const int NU = 1;
    const int NP = 10;
    
    float Y_ref[][];
    float y_sensor;
    float u_opt;
    
    float A[NX][NX];
    float B[NX][NU];
    float C[1][NX];
    float L[][];
    
    static float x_hat_kn1[NX];
    float deltaU_opt[NU];
    float Phi[][];
    float Hinv_PhiT[][];
    float F[][];
    
    /* d/d deltaU (J) = 0
    deltaU_opt = Hinv_PhiT * (Y_ref - F * x_hat_kn1);
    
    /* optional calculation
    /* Y_opt = F*x_hat_kn1 + Phi*deltaU_opt;
    
    /* receeding horizon control
    u_opt = deltaU_opt[1];
    
    /* observer update
    x_hat = (A-L*C)*x_hat_kn1 + L*y_sensor + B*u_opt;
    y_hat = C*x_hat_kn1;
    x_hat_kn1 = x_hat;
    
    */
    
}
