#ifndef GLOBALS_H
#define GLOBALS_H

#include "WConstants.h"

#define SD_CS BUILTIN_SDCARD
// MOSI = 7, MISO = 12, SCK = 14
boolean SD_flgSdCard = 0; // SD card present
File SongFile;
File LogFile;
File SettingsFile;
File TrainingFile;

List SettingsList = List();

#define LDR_PIN A8

#define TCH_CS  8
// MOSI=11, MISO=12, SCK=13

#define TFT_DC  9
#define TFT_CS 15
// MOSI=7, MISO=12, SCK=14

#define TMP_PIN 18

#define BGCOLOR ILI9341_WHITE
#define FGCOLOR ILI9341_BLACK

#define SZ_SUNPROFILE (int)14
#define RES_SUNSTATUS (int)15                     // [min] use divisor of 60
#define SZ_SUNSTATUS (int)24*(60/RES_SUNSTATUS)   // every pixel
#define SET_RISE_LIM (int)5                       // [degrees]

#define UI_SETTINGS0 0
#define UI_SETTINGS1 6
#define UI_INFO 1
#define UI_PLOT 2
#define UI_KEYPAD 3
#define UI_CLOCK 4
#define UI_MENU 5
#define UI_CAN 7
#define UI_NN 8
#define UI_MPC 9
#define UI_KALM 10

int UI = UI_MENU;

#define PWD 1993
int NrWrongPwd = 0;
int StPwdLock = 0;
unsigned long TiPwdLockStart = 0;

// geographic position
//float Sys_Lati = 47.0651; // Graz
//float Sys_Longi = 15.4631;

float Sys_Lati = 48.232465; // Ranshofen
float Sys_Longi = 13.013861;

// window geometry
float win_height = 1.73;  // [m]
float win_len = 3.60;     // [m]
float roof_len = 1.22;    // [m]
// todo: azimut correction for roof angle
// only true fpr 90°
//float roof_angle = atan(roof_len/win_height)*180/PI;  // [deg] 
float roof_angle = 90;

unsigned long ms_counter = 0;

ILI9341_t3 tft = ILI9341_t3(TFT_CS, TFT_DC);

char keypad_str[10] = "";

float CAN_SpdEng = 0;
float CAN_TqEng = 0;
float CAN_TqEngRef = 200;

MovgAvgFilter TInMAFilter = MovgAvgFilter();
MovgAvgFilter HumInMAFilter = MovgAvgFilter();

SimpleMpc Mpc1 = SimpleMpc();

KalmanFilter Kalman1 = KalmanFilter();

StateSpaceModel PlantModel = StateSpaceModel();

float Y_ref_g[NP] = {0,0,0,0,0,10,10,10,10,10};



#endif /* GLOBALS_H */
