#ifndef SENSINP_H
#define SENSINP_H

#include "Globals.h"
#include "ProjectLib.h"

#include <OneWire.h>
//#include <DallasTemperature.h>
#include "DHT.h"

#define DHTPIN1 19
#define DHTPIN2 18  

#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321


// Connect pin 1 (on the left) of the sensor to +5V
// NOTE: If using a board with 3.3V logic like an Arduino Due connect pin 1
// to 3.3V instead of 5V!
// Connect pin 2 of the sensor to whatever your DHTPIN is
// Connect pin 4 (on the right) of the sensor to GROUND
// Connect a 10K resistor from pin 2 (data) to pin 1 (power) of the sensor

float SI_TIn, SI_TOut;
float SI_TInFilt, SI_TOutFilt;
float SI_HumIn, SI_HumOut;
float SI_HumInFilt, SI_HumOutFilt;
float SI_TDewIn, SI_TDewOut;
float SI_LvlSun;
uint8_t SI_StSun;

boolean SI_FlgTOutHotter = 0;
boolean SI_FlgTOutHotterDly = 0;

//OneWire oneWire(TMP_PIN);  // temp sensor bus
//DallasTemperature sensors(&oneWire);

DHT dht1(DHTPIN1, DHTTYPE);
DHT dht2(DHTPIN2, DHTTYPE);

void readSensors();
void initSensors();
float dewPoint(float celsius, float humidity);

void initSensors()
{
    dht1.begin();
    dht2.begin();
}

void readSensors()
{   
    SI_TIn = dht1.readTemperature();
    SI_TOut = dht2.readTemperature();
    SI_HumIn = dht1.readHumidity();
    SI_HumOut = dht2.readHumidity();;

    //todo: error handling
    if(isnan(SI_TIn) || isnan(SI_HumIn)) 
    {   
        SI_TIn = 0;
        SI_HumIn = 0;
    }
    else
    {
        SI_TIn = saturate(SI_TIn, -40, 50);
        SI_HumIn = saturate(SI_HumIn, 0, 100);
        SI_TInFilt = TInMAFilter.calculate(SI_TIn);
        SI_HumInFilt = HumInMAFilter.calculate(SI_HumIn);
        SI_TDewIn = dewPoint(SI_TInFilt, SI_HumInFilt);
    }
    
    if(isnan(SI_TOut) || isnan(SI_HumOut))
    {
        SI_TOut = 0;
        SI_HumOut = 0;
    } 
    else
    {
        SI_TOut = saturate(SI_TOut, -40, 50);
        SI_HumOut = saturate(SI_HumOut, 0, 100);
        SI_TOutFilt = TOutMAFilter.calculate(SI_TOut);
        SI_HumOutFilt = HumOutMAFilter.calculate(SI_HumOut);
        SI_TDewOut = dewPoint(SI_TOutFilt, SI_HumOutFilt);
    } 

    //TDiff = abs(SI_TOut-SI_TIn);

    // read sun level from LDS
    SI_LvlSun = 1024 - analogRead(LDR_PIN);  
    //LvlSun = 1000 - mapfloat(LvlSun,0,1024,0,1000);

    float axis[11] = {0, 102, 204, 306, 409, 511, 613, 716, 818, 920, 1023};
    //float data[11] = {0, 1, 4, 9, 16, 25, 36, 49, 64, 81, 100}; // x^2
    float data[11] = {1, 1.5849, 2.5119, 3.9811, 6.3096, 10, 15.8489, 25.1189, 39.8107, 63.0957, 100}; //exp(x)
  
    SI_LvlSun = LookupTable(axis, data, 11, SI_LvlSun);

    // sun status hysteresis
    if((SI_LvlSun >= 80) && (SI_StSun == 0)) SI_StSun = 1;
    if((SI_LvlSun <= 70) && (SI_StSun == 1)) SI_StSun = 0;

}

float dewPoint(float celsius, float humidity)
{
  // (1) Saturation Vapor Pressure = ESGG(T)
  float RATIO = 373.15 / (273.15 + celsius);
  float RHS = -7.90298 * (RATIO - 1);
  RHS += 5.02808 * log10(RATIO);
  RHS += -1.3816e-7 * (pow(10, (11.344 * (1 - 1/RATIO ))) - 1) ;
  RHS += 8.1328e-3 * (pow(10, (-3.49149 * (RATIO - 1))) - 1) ;
  RHS += log10(1013.246);

        // factor -3 is to adjust units - Vapor Pressure SVP * humidity
  float VP = pow(10, RHS - 3) * humidity;

        // (2) DEWPOINT = F(Vapor Pressure)
  float T = log(VP/0.61078);   // temp var
  return (241.88 * T) / (17.558 - T);
}


#endif
