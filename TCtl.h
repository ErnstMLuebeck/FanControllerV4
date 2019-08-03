#ifndef TCTL_H
#define TCTL_H

#include "Globals.h"

// controller states
#define AUTO 1
#define MANUAL 0

#define FANOFF 0
#define FANON 1

// shade controller states
#define LIGHT 0
#define SHADE 1


//int TEC_StFanCtlr = AUTO;
//int TEC_StFan = FANOFF;
//int TEC_StShadeCtlr = LIGHT;

float TEC_TInDmdDay = 21;
float TEC_TInDmdNight = 15;

float TEC_TInDmd = 22;

// Fan Controller
uint32_t FC_TiFanOn = 600; // [s]
uint32_t FC_TiFanOff = 600; // [s]
float FC_TOutMin = 12; // [degC]
float FC_TDewDiffOn = 3; // [degC]
float FC_TDewDiffOff = 2; // [degC]
int8_t FC_StFanCtlr = AUTO;//, AUTO
int8_t FC_StFan = FANON; //, FANOFF
uint32_t FC_TiFanOnTot = 123452; // (SD Karte)

uint32_t FC_TiFanStart = now();
uint32_t FC_TiFanStop = now();

void StmacFanCtlr();
void StmacShadeCtlr();

void StmacFanCtlr()
{
    if (FC_StFanCtlr == AUTO)
    {
        switch (FC_StFan)
        {
        case FANON:
            // ABSCHALTEN WENN:
            // Taupunkt innen <= (Taupunkt aussen + FC_TDewDiffOff), FC_TDewDiffOff = 1.5
            // ODER Temp. aussen <= FC_TOutMin; FC_TOutMin = 10
            // ODER Luefter ist FC_TiFanOn gelaufen; FC_TiFanOn = 10min
            // ODER Taupunkt Temp. aussen > Temp. innen - 5 
            // ODER Sensor Error
            if (SI_TDewIn <= (SI_TDewOut + FC_TDewDiffOff) ||
                SI_TOut <= FC_TOutMin || 
                (now() - FC_TiFanStart) >= FC_TiFanOn ||
                SI_TDewOut > (SI_TIn - 5))
            {
                Serial.println("Luefter AUS");
                FC_StFan = FANOFF;
                FC_TiFanStop = now();
                FC_TiFanOnTot += (now() - FC_TiFanStart);
            }
            break;

        case FANOFF:
            // EINSCHALTEN WENN:
            // Taupunkt aussen < (Taupunkt Innen - FC_TDewDiffOn); FC_TDewDiffOn = 1.5
            // UND Taupunkt aussen < Temp. innen - 5 (im Sommer fast nie der Fall!)
            // UND Temp. aussen > TOutMin
            // UND Luefter war TiOnOff aus
            // UND kein Sensor Error
            if (SI_TDewOut < (SI_TDewIn - FC_TDewDiffOn) && 
                SI_TDewOut < (SI_TIn - 5) && 
                SI_TOut > FC_TOutMin && 
                (now() - FC_TiFanStop) >= FC_TiFanOff)
            {
                Serial.println("Luefter EIN");
                FC_StFan = FANON;
                FC_TiFanStart = now();
            }
            break;
        }
    }
    else if (FC_StFanCtlr == MANUAL)
    {
    }
}

//void StmacFanCtlr()
//{
//    switch(TEC_StFanCtlr)
//    {
//        case FANOFF: 
//            if((SI_TIn > TEC_TInDmd) && (SI_TOut < (SI_TIn-2)))
//            {   
//                TEC_StFanCtlr = FANON_COOL;
//                Serial.println("FANON_COOL");
//                break;
//            }
//            if((SI_TIn < TEC_TInDmd) && (SI_TOut > (SI_TIn+2)))
//            {
//                TEC_StFanCtlr = FANON_HEAT;
//                Serial.println("FANON_HEAT");
//                break;
//            }
//            
//            break;
//
//        case FANON_COOL: 
//            if(SI_TIn <= TEC_TInDmd)
//            {   
//                TEC_StFanCtlr = FANOFF;
//                Serial.println("FANOFF");
//                break;
//            }
//            break;
//
//        case FANON_HEAT: 
//            if(SI_TIn >= TEC_TInDmd)
//            {   
//                TEC_StFanCtlr = FANOFF;
//                Serial.println("FANOFF");
//                break;
//            }
//            break;
//            
//        case VENT: break;
//      
//    }
//  
//}

//void StmacShadeCtlr()
//{
//    switch(TEC_StShadeCtlr)
//    {
//        case LIGHT: 
//            if((SI_TIn > TEC_TInDmd) && SI_StSun)
//            {   
//                TEC_StShadeCtlr = SHADE;
//                Serial.println("SHADE");
//                break;
//            }
//            
//            break;
//
//        case SHADE: 
//            if(SI_TIn < TEC_TInDmd && SI_StSun)
//            {   
//                TEC_StShadeCtlr = LIGHT;
//                Serial.println("LIGHT");
//                break;
//            }
//            break;
//      
//    }
//  
//}




#endif
