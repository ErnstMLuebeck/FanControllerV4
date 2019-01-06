#ifndef SENSINP_H
#define SENSINP_H

#include "Globals.h"
#include "ProjectLib.h"

#include <OneWire.h>
//#include <DallasTemperature.h>
#include "DHT.h"

#define DHTPIN 18     // what digital pin we're connected to

// Uncomment whatever type you're using!
//#define DHTTYPE DHT11   // DHT 11
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
//#define DHTTYPE DHT21   // DHT 21 (AM2301)

// Connect pin 1 (on the left) of the sensor to +5V
// NOTE: If using a board with 3.3V logic like an Arduino Due connect pin 1
// to 3.3V instead of 5V!
// Connect pin 2 of the sensor to whatever your DHTPIN is
// Connect pin 4 (on the right) of the sensor to GROUND
// Connect a 10K resistor from pin 2 (data) to pin 1 (power) of the sensor

float SI_TIn, SI_TOut;
float SI_TInFilt;
float SI_HumIn, SI_HumOut;
float SI_HumInFilt;
float SI_TDewIn, SI_TDewOut;
float SI_LvlSun;
uint8_t SI_StSun;

//OneWire oneWire(TMP_PIN);  // temp sensor bus
//DallasTemperature sensors(&oneWire);

DHT dht(DHTPIN, DHTTYPE);

void readSensors();
void initSensors();
float dewPoint(float celsius, float humidity);

void initSensors()
{
    /*
    sensors.begin();
    sensors.setResolution(12);
  
    uint8_t addr0[8], addr1[8];
  
    uint8_t numSensors = sensors.getDeviceCount();
    sensors.getAddress(addr0, 0);
    sensors.getAddress(addr1, 1);
  
    Serial.println(numSensors);
    Serial.print("ADDR0 =");
    for(int i = 0; i < 8; i++) {
      Serial.write(' ');
      Serial.print(addr0[i], HEX);
    }
    Serial.println();
    
    Serial.print("ADDR1 =");
    for(int i = 0; i < 8; i++) {
      Serial.write(' ');
      Serial.print(addr1[i], HEX);
    }
    Serial.println();
     */
    
    dht.begin();
}

void readSensors()
{   
    /*
    sensors.requestTemperatures();
    SI_TIn = sensors.getTempCByIndex(1);
    SI_TOut = sensors.getTempCByIndex(0);
    */
     
    SI_TIn = dht.readTemperature();
    SI_TOut = 10.3 + random(2);

    //todo: error handling
    

    SI_TIn = saturate(SI_TIn, -40, 50);
    SI_TOut = saturate(SI_TOut, -40, 50);
    
    SI_TInFilt = TInMAFilter.calculate(SI_TIn);
    Serial.println(SI_TInFilt);

    //SI_TIn = 26.5 + random(3);
    //SI_TOut = 10.3 + random(5);
    SI_HumIn = dht.readHumidity();
    SI_HumOut = 26.6;
    
    SI_HumIn = saturate(SI_TIn, 0, 100);
    
    SI_HumInFilt = HumInMAFilter.calculate(SI_HumIn);

    SI_TDewIn = dewPoint(SI_TIn, SI_HumIn);
    SI_TDewOut = dewPoint(SI_TOut, SI_HumOut);

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

    //CAN_SpdEng = 1000 + random(100);
    //CAN_TqEng = 80 + random(50);

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
