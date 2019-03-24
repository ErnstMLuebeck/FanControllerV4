#ifndef SUNMDL_H
#define SUNMDL_H

#include "Globals.h"
#include "ProjectLib.h"

// Sun Model
// livingroom window obstacles (neighbour building, orientation)
int SM_AziProfile[SZ_SUNPROFILE] = {0, 150, 151, 187, 188, 255, 256, 265, 266, 278, 279, 329, 330, 360};
int SM_EleProfile[SZ_SUNPROFILE] = {90, 90, 5, 5, 22, 30, 5, 5, 7, 7, 5, 5, 90, 90};
bool SM_SunStatusVect[SZ_SUNSTATUS];
bool SM_StSunMdl = 1;
int SM_TimeVect[SZ_SUNSTATUS];

int SM_RiseHour, SM_RiseMin, SM_SetHour, SM_SetMin;

float SM_Azimuth, SM_Elevation;
float SM_ArPwrDens = 0;

void calcSunAngle(float lati, float longi, int month, int day, int hour, int minute, float* azimut, float* elevation);
void calcSunriseTime(float lati, float longi, int month, int day, int& rise_h, int& rise_min);
void calcSunsetTime(float lati, float longi, int month, int day, int& set_h, int& set_min);
void calcSunstatusVect(float lati, float longi, int month, int day, bool sunstatus[], int timevect[]);
bool getSunStatus(int azimut_int, int elevation_int, float roof_len, int& blocker_angle, int& roof_angle);
bool getSunStatusFromVect(int minutes);
int date2day(int day, int month);

int date2day(int day, int month)
{
    int tage_pro_monat[13] = {0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    int days = 0;

    for(int i=0; i<month; i++) days += tage_pro_monat[i];
        days += day;

    return(days);
}


void calcSunAngle(float lati, float longi, int month, int day, int hour, int minute, float* azimut, float* elevation)
{ 
    int days = date2day(day,month);
  
    float pi = 3.1415926536;
    float K = pi/180.0;   
  
    float deklination = -23.45*cos(K*360*(days+10)/365); 
    float zeitgleichung = 60*(-0.171*sin(0.0337*days + 0.465) - 0.1299 * sin(0.01787*days - 0.168));
    float stundenwinkel = 15.0*(hour + minute/60.0 - (15.0-longi)/15.0 - 12.0 + zeitgleichung/60.0);
    float sin_hoehe = sin(K*lati)*sin(K*deklination) + cos(K*lati)*cos(K*deklination)*cos(K*stundenwinkel);
    float y = -(sin(K*lati) * sin_hoehe - sin(K*deklination)) / (cos(K*lati) * sin( acos(sin_hoehe)));
  
    *azimut = acos(y)/K;             
    *elevation = asin(sin_hoehe)/K;  
  
    if(hour>=12) *azimut = 360 - *azimut;
  
    // calculation validated agains MATLAB. Identical results.
  
    //Serial.print(deklination,4); Serial.println(" deklination");
    //Serial.print(zeitgleichung,4); Serial.println(" zeitgleichung");
    //Serial.print(stundenwinkel,4); Serial.println(" stundenwinkel");
    //Serial.print(sin_hoehe,4); Serial.println(" sin_hoehe");
    //Serial.print(y,4); Serial.println(" y");
    //Serial.print(days); Serial.println(" days");
}



void calcSunriseTime(float lati, float longi, int month, int day, int& rise_h, int& rise_min)
{
    float azimut, elevation;
  
    for(int i=0; i<1440; i++)
    {
        int h = floor(i/60);
        int m = i%60;
    
        //Serial.print(h); Serial.print(":");Serial.println(m);
    
        calcSunAngle(lati, longi, month, day, h, m, &azimut, &elevation);
    
        if(elevation > SET_RISE_LIM) 
        {   //Serial.print(h); Serial.print(":");Serial.println(m);
            rise_h = h;
            rise_min = m;
            break;
        }
    }
}

void calcSunsetTime(float lati, float longi, int month, int day, int& set_h, int& set_min)
{
    float azimut, elevation;

    // search from noon to evening
    for(int i=720; i<1440; i++)
    {
        int h = floor(i/60);
        int m = i%60;

        //Serial.print(h); Serial.print(":");Serial.println(m);

        calcSunAngle(lati, longi, month, day, h, m, &azimut, &elevation);

        if(elevation <= SET_RISE_LIM) 
        {   //Serial.print(h); Serial.print(":");Serial.println(m);
            set_h = h;
            set_min = m;
            break;
        }
    }
}

void calcSunstatusVect(float lati, float longi, int month, int day, bool sunstatus[], int timevect[])
{
    float azimut_tmp, elevation_tmp;
    int blocker_angle, azimut_int, elevation_int, roof_angle;
  
    for(int i=0; i<SZ_SUNSTATUS; i++) // every 15min
    {    
        int h = floor((i*RES_SUNSTATUS)/60);
        int m = (i*RES_SUNSTATUS)%60;
    
        calcSunAngle(lati, longi, month, day, h, m, &azimut_tmp, &elevation_tmp);
        azimut_int = (int)azimut_tmp;
        elevation_int = (int)elevation_tmp;
    
        sunstatus[i] = getSunStatus(azimut_int, elevation_int, roof_len, blocker_angle, roof_angle);
        timevect[i] = (i*RES_SUNSTATUS);
    
        // print sunstatus over time
        /*Serial.print(h); Serial.print(":"); printDigits(m);
        Serial.print(", azi: "); Serial.print(azimut_int);
        Serial.print(", ele: "); Serial.print(elevation_int);
        Serial.print(", blocker angle: "); Serial.print(blocker_angle);
        Serial.print(", roof angle: "); Serial.print(roof_angle);
        Serial.print(", sun: "); Serial.println(sunstatus[i]);
        */
    }
}

bool getSunStatus(int azimut_int, int elevation_int, float roof_len, int& blocker_angle, int& roof_angle)
{
    blocker_angle = LookupTableInt(SM_AziProfile, SM_EleProfile, SZ_SUNPROFILE, azimut_int);

//    // azimut correction for roof angle
//    int alpha_max = 90-atan(roof_len/(win_len/2))*180/PI;
//    int alpha_min = -alpha_max;
//
//    int alpha = azimut_int - orientation;
//    alpha = (alpha <= -90) ? -90 : alpha = alpha; 
//    alpha = (alpha >= 90) ? 90 : alpha = alpha;
// 
//    float hypo = sqrt(pow((win_len/2),2)+pow(roof_len,2));
//    //float a = abs((hypo-roof_len)/(alpha_max)*alpha)+roof_len;
//    float a = hypo-(hypo-roof_len)*cos(alpha*PI/180);
//    float b = win_height;
//  
//    roof_angle = atan(b/a)*180/PI;

//    Serial.print("alpha min: "); Serial.println(alpha_min);
//    Serial.print("alpha max: "); Serial.println(alpha_max);
//    Serial.print("alpha: "); Serial.println(alpha);
//    Serial.print("roof angle: "); Serial.println(roof_angle);

    if((elevation_int >= blocker_angle) && (elevation_int <= 90)) return(true);
    else return(false);
}

bool getSunStatusFromVect(int minutes)
{
    // requires global variables:
    // sunstatus[SZ_SUNSTATUS]
    // timevect[SZ_SUNSTATUS]

    int idx = SZ_SUNSTATUS-1;

    for(int i=0; i<SZ_SUNSTATUS; i++)
    {   
        if (SM_TimeVect[i] >= minutes)
        {   idx = i;
            break;
        }
    }
    return(SM_SunStatusVect[idx]);  
}

float calcSunArPwrDens(float azimuth, float elevation)
{   // area power density of sun in W/m^2

    if(elevation <= 0)
        return(0.0);
    
    float ArPwrDensNom = 1200;  // [W/m^2] nominal power per area

    float ArPwrDens = ArPwrDensNom * sin(elevation * PI/180.0);

    /* TODO:
     * consider season
     * 
     */

     return(ArPwrDens);
  
}






#endif /* SUNMDL_H */
