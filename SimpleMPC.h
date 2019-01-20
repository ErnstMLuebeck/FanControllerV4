#ifndef SIMPLEMPC_H
#define SIMPLEMPC_H

#include <Arduino.h>
#include <stdio.h>
#include <math.h>

#define NX 2
#define NY 1
#define NU 1
#define NP 10

class SimpleMpc
{
public:
    SimpleMpc();
    void calculate();
    void setYrefReceeding(float _y_ref);
    void setYref(float* _Y_ref);
    void getYopt(float* _Y_opt);
    
private:
    void MatrixMultiply(float* A, float* B, int m, int p, int n, float* C);
    void MatrixPrint(float* A, int m, int n);
    void MatrixSubtract(float* A, float* B, int m, int n, float* C);
    void MatrixAdd(float* A, float* B, int m, int n, float* C);
    void MatrixScale(float* A, int m, int n, float k, float* C);
    void MatrixCopy(float* A, int n, int m, float* B);
    
    void calcMpcF(float* A, float* C, int nx, int ny, int np, float* F);
    void calcMpcPhi(float* A, float* B, float* C, int nx, int ny, int nu, int np, float* Phi);
    
    float A[NX][NX] ={
        {0.990703820085258, 0.0929617991474188},
        {-0.185923598294838, 0.859235982948375}};
    
    float B[NX][NU] = {
        {-0.0995351910042629},
        {0.00929617991474188}};
    
    float C[NY][NX] = {
        {-0.0929617991474188, 0.929617991474187}};
    
    float L[NX][NY] = {
        {-0.351798484809834},
        {0.427310988590526}};
    
    float y_sensor = 1;
    
    float x_hat[NX][1] = {
        {0},
        {0}};
    
    float x_hat_kn1[NX][1] = {
        {0},
        {0}};
    
    float Y_ref[NP] = {{0}};
    
    float Y_ref_kn1[NP][1] = {{0}};
    
    float Y_opt[NP] = {{0}};
    float Y_opt_kn1[NP] = {{0}};
    
    float deltaU_opt[NP][NU] = {{0}};
    
    float deltaU_opt_kn1[NP][NU] = {{0}};
    
    float qy = 1;
    float ru = 0.005;
    
    float F[NP*NY][NX] = {{0}};
    /*
    float F[NP*NY][NX] = {
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
    */
    
    float Phi[NP*NY][NP*NU] = {
        {0.0178948665349633, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0.0337155002065163, 0.0178948665349633, 0, 0, 0, 0, 0, 0, 0, 0},
        {0.0468293786754648, 0.0337155002065163, 0.0178948665349633, 0, 0, 0, 0, 0, 0, 0},
        {0.0573485352466993, 0.0468293786754648, 0.0337155002065163, 0.0178948665349633, 0, 0, 0, 0, 0, 0},
        {0.0654185164520088, 0.0573485352466993, 0.0468293786754648, 0.0337155002065163, 0.0178948665349633, 0, 0, 0, 0, 0},
        {0.0712112700851343, 0.0654185164520088, 0.0573485352466993, 0.0468293786754648, 0.0337155002065163, 0.0178948665349633, 0, 0, 0, 0},
        {0.0749184773697086, 0.0712112700851343, 0.0654185164520088, 0.0573485352466993, 0.0468293786754648, 0.0337155002065163, 0.0178948665349633, 0, 0, 0},
        {0.0767453948416271, 0.0749184773697086, 0.0712112700851343, 0.0654185164520088, 0.0573485352466993, 0.0468293786754648, 0.0337155002065163, 0.0178948665349633, 0, 0},
        {0.0769052534287336, 0.0767453948416271, 0.0749184773697086, 0.0712112700851343, 0.0654185164520088, 0.0573485352466993, 0.0468293786754648, 0.0337155002065163, 0.0178948665349633, 0},
        {0.0756142456086208, 0.0769052534287336, 0.0767453948416271, 0.0749184773697086, 0.0712112700851343, 0.0654185164520088, 0.0573485352466993, 0.0468293786754648, 0.0337155002065163, 0.0178948665349633}};
    
    float Hinv_PhiT[NP][NP] = {
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

    
};


#endif
















