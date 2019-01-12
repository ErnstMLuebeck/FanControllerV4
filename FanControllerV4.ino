/* Keller Lüftung V4 (HW V2)
 * 
 * System Uhr (3V Batterie)
 * Sensor Innen/Außen Temp + Hum
 * Taupunkt Innen/Außen berechnen
 * Controller Mode: Automatik, Manuell
 * Datenlogger SD, jede Minute
 *   Datum: 2018-07-23
 *   Uhrzeit: 13:50:45
 *   TIn: 21.2
 *   HumIn: 45.2
 *   TDewIn: 15.2
 *   TOut: 24.1
 *   HumOut: 80.3
 *   TDewOut: 18.5
 *   StFan: 1
 *   TiFanOnTot: 13900
 * Diagramm der letzten 8h
 * TiFanOnTot von SD Karte lesen
 * 
 * Lüfter EIN:
 * TDewOut < (TDewIn - TDewDiffOn)
 * UND TDewOut < (TIn - 5)
 * UND TiFanOff > TiFanOffMin
 * UND SensDiag == 0
 * 
 * Lüfter AUS:
 * TDewIn <= (TDewOut + TDewDiffOff)
 * ODER TOut <= TOutMin
 * ODER TDewOut > (TIn - 5)
 * ODER (StFan == ON) && (TiFanOn >= TiFanOnMax)
 * ODER SensDiag != 0
 * 
 * Kalibration (SD Karte?)
 * TiFanOn = 600; // [s]
 * TiFanOff = 600; // [s]
 * TOutMin = 12; // [degC]
 * TTauDiffOn = 3; // [degC]
 * TTauDiffOff = 2; // [degC]
 * StFanCtlr = MANUAL, AUTO
 * StFan = ON, OFF
 * TiFanOnTot = 12345 (SD Karte)
 * 
 */

#include <stdlib.h>
#include <ILI9341_t3.h>
//#include <ILI9341_fonts.h>
#include <font_Arial.h> // from ILI9341_t3
#include <font_AwesomeF000.h>
#include <font_AwesomeF180.h>
#include <XPT2046_Touchscreen.h>
#include <SPI.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <TimeLib.h>
#include "TimerOne.h"  
#include <SD.h>
#include "List.h"
#include "MovgAvgFilter.h"
#include "SimpleNeurNet.h"

#include "ecu_reader.h"
#include <FlexCAN.h>
#include <Wire.h>

#include "Globals.h"

// Custom Components
#include "ProjectLib.h"
#include "SunMdl.h"
#include "SysClk.h"
#include "SensInp.h"
#include "TCtl.h"
#include "TchCtlr.h"
#include "EvntTmr.h"


ecu_reader_class ecu_reader;

void setup() 
{ 
    tft.begin();
    tft.setRotation(1);
    tft.fillScreen(BGCOLOR);
    
    initTouch();
  
    //initEvents();
  
    initSensors();

    //ecu_reader.init(500000);
    
    setSystemTime();
    getSystemTime(&SC_SysHour, &SC_SysMin, &SC_SysSec);
    getSystemDate(&SC_SysDay, &SC_SysMon, &SC_SysYear);
    
    // rethink this one!
    getDst(&SC_Dst);

    FC_TiFanStart = now();
    FC_TiFanStop = now();
    
    
    //handleSdCard();
    initSdCard();
    
    loadSdCardTrainingData();
  
    // start OS
    // timer for OS task exectution
    Timer1.initialize(10000);   // 10ms
    Timer1.attachInterrupt(OS_TaskTimer);

}

void loop() 
{ 
    int engine_data;

    /*
  
    if(ecu_reader.request(ENGINE_RPM,&engine_data))
    {    //Serial.print("RPM = ");
         //Serial.println(engine_data);
         CAN_SpdEng = engine_data;
    } 
    //else Serial.println("No RPM Data.");

      if(ecu_reader.request(ENGINE_TRQ_REF,&engine_data))
      {    //Serial.print("TRQ_REF = ");
           //Serial.println(engine_data);
           CAN_TqEngRef = engine_data;
      } 
      //else Serial.println("No TRQ_REF Data.");
    
    if(ecu_reader.request(ENGINE_TRQ_PERC,&engine_data))
      {   //Serial.print("TRQ_PERC = ");
          //Serial.println(engine_data);
          CAN_TqEng = CAN_TqEngRef * (float)engine_data/100.0;
          
      } 
      //else Serial.println("No TRQ_PERC Data.");
*/

    
    // display timeout 60s
    if((millis()-TiLastTouch) >= 60000 && TchTimeout == 0)
    {   //TchTimeout = 1;
        //Serial.println("Display backligh off.");
        //tft.sleep(1);
        //UI = UI_KEYPAD;
        //buildUI(UI);

        /* Todo:
         * disable touch controller
         */
    }

    // react on touch if display was off
    if(TC_IsTouched && TchTimeout == 1)
    {   TchTimeout = 0;
        tft.sleep(0);
        Serial.println("Display backlight on.");
        TC_WasTouched = false;     
    }

    // timout if pwd was entered wrong too often
    if(StPwdLock && (millis()-TiPwdLockStart) >= 5000)
    {   StPwdLock = 0;
        NrWrongPwd = 0;
        strcpy(keypad_str, "");
    }

  // react on touch if display is on
  if(TC_WasTouched && TchTimeout == 0)
  {         
      if(UI == UI_SETTINGS0)
      {
          if(TC_IsTouchedId == 0)
          {
              FC_TOutMin += 0.5;
              refreshUI(UI);
          }
          if(TC_IsTouchedId == 1)
          {
              FC_TOutMin -= 0.5;
              refreshUI(UI);
          }
          if(TC_IsTouchedId == 2)
          {
              FC_TDewDiffOn += 0.5;
              refreshUI(UI);
          }
          if(TC_IsTouchedId == 3)
          {
              FC_TDewDiffOn -= 0.5;
              refreshUI(UI);
          }

          if(TC_IsTouchedId == 4)
          {   // slider
              
          } 

          if(TC_IsTouchedId == 5)
          {   // home button
              UI = UI_MENU;
              buildUI(UI);
          }

          if(TC_IsTouchedId == 7)
          {
              UI = UI_SETTINGS1;
              buildUI(UI);
          }

      }
      if(UI == UI_SETTINGS1)
      {
          if(TC_IsTouchedId == 0)
          {
              FC_TiFanOn += 60;
              FC_TiFanOff = FC_TiFanOn;
              refreshUI(UI);
          }
          if(TC_IsTouchedId == 1)
          {
              FC_TiFanOn -= 60;
              FC_TiFanOff = FC_TiFanOn;
              refreshUI(UI);
          }
          if(TC_IsTouchedId == 2)
          {
              FC_TDewDiffOff += 0.5;
              refreshUI(UI);
          }
          if(TC_IsTouchedId == 3)
          {
              FC_TDewDiffOff -= 0.5;
              refreshUI(UI);
          }
        
          if(TC_IsTouchedId == 5)
          {
              UI = UI_MENU;
              buildUI(UI);
          }

          if(TC_IsTouchedId == 6)
          {
              UI = UI_SETTINGS0;
              buildUI(UI);
          }

      }
      else if(UI == UI_MENU)
      {
          if(TC_IsTouchedId == 0)
          {
              UI = UI_INFO;
              buildUI(UI);
          }
          if(TC_IsTouchedId == 1)
          {
              UI = UI_PLOT;
              buildUI(UI);
          }
          if(TC_IsTouchedId == 2)
          {
              UI = UI_CLOCK;
              buildUI(UI);
          }
          if(TC_IsTouchedId == 3)
          {
              UI = UI_SETTINGS0;
              buildUI(UI);
          }
          if(TC_IsTouchedId == 4)
          {
              UI = UI_CAN;
              buildUI(UI);
          }
          if(TC_IsTouchedId == 5)
          {
              UI = UI_NN;
              buildUI(UI);
          }

      }
      else if(UI == UI_INFO)
      {   
          if(TC_IsTouchedId == 0)
          {   
              UI = UI_MENU;
              buildUI(UI);   
          }
          if(TC_IsTouchedId == 1)
          {
              readSensors();
          }
        
      }
      else if(UI == UI_PLOT)
      {   
          if(TC_IsTouchedId == 0)
          {   
              UI = UI_MENU;
              buildUI(UI);   
          }
        
      }
      else if(UI == UI_CAN)
      {   
          if(TC_IsTouchedId == 0)
          {   
              UI = UI_MENU;
              buildUI(UI);   
          }
        
      }
      else if(UI == UI_NN)
      {
          if(TC_IsTouchedId == 0)
          {
              UI = UI_MENU;
              buildUI(UI);
          }
          
      }
      else if(UI == UI_KEYPAD)
      {   
          int keylen = strlen(keypad_str);

          if((keylen < 6) && !StPwdLock)
          {
              switch(TC_IsTouchedId) // 1
              {   case 0: strcat(keypad_str,"1"); break;            
                  case 1: strcat(keypad_str,"2"); break;
                  case 2: strcat(keypad_str,"3"); break;
                  case 3: strcat(keypad_str,"4"); break;
                  case 4: strcat(keypad_str,"5"); break;
                  case 5: strcat(keypad_str,"6"); break;
                  case 6: strcat(keypad_str,"7"); break;
                  case 7: strcat(keypad_str,"8"); break;
                  case 8: strcat(keypad_str,"9"); break;
                  case 9: strcat(keypad_str,"0"); break;
                  default: break;
              }
          }

          if(TC_IsTouchedId == 10) // delete
          {   strcpy(keypad_str, "");
          }
          if(TC_IsTouchedId == 11) // OK
          {
              if(atoi(keypad_str) == PWD)
              {   strcpy(keypad_str, "");
                  UI = UI_MENU;
                  buildUI(UI); 
                  NrWrongPwd = 0;
              }
              else
              {   strcpy(keypad_str, "");
                  NrWrongPwd++;

                  if(NrWrongPwd >= 5)
                  {   TiPwdLockStart = millis();
                      StPwdLock = 1;
                      strcpy(keypad_str, "WAIT!");
                  }
              }
              
          }

          refreshUI(UI);
        
      }
      else if(UI == UI_CLOCK)
      {   
          if(TC_IsTouchedId == 0)
          {   
              UI = UI_MENU;
              buildUI(UI);   
          }
        
      }
      
      TC_WasTouched = false;     
  }

}

void OS_TaskTimer()
{   
    noInterrupts();

    if(ms_counter == 0)
    {   
        //Serial.println("Init");
        OS_Init();
    }
    
    ms_counter++;

    // 10ms tasks
    OS_10ms();
    
    if(ms_counter%10 == 0)
    {   //Serial.println("100ms");
    }
    if(ms_counter%50 == 0)
    {   //Serial.println("500ms");
    }
    if(ms_counter%100 == 0)
    {   //Serial.println("1s");
        OS_1s();
        
    }
    if(ms_counter%1000 == 0)
    {   //Serial.println("10s");     
        OS_10s();       
    }
    if(ms_counter%6000 == 0)
    {   //Serial.println("60s");     
        OS_60s();       
    }
    
    interrupts();
}

void OS_Init()
{ 
    printBootLogo();

    tft.setTextColor(FGCOLOR);
    tft.setFont(Arial_8);
    tft.setCursor(1, 229);
    tft.print("Read Sensors..     ");
    
    readSensors();
    delay(1000);

    tft.setTextColor(BGCOLOR);
    tft.setFont(Arial_8);
    tft.setCursor(1, 229);
    tft.print("Read Sensors..     ");

    tft.setTextColor(FGCOLOR);
    tft.setFont(Arial_8);
    tft.setCursor(1, 229);
    tft.print("Calc. Sunprofile..");

    calcSunAngle(Sys_Lati, Sys_Longi, SC_SysMon, SC_SysDay, SC_SysHour, SC_SysMin, &SM_Azimuth, &SM_Elevation);
    calcSunriseTime(Sys_Lati, Sys_Longi, SC_SysMon, SC_SysDay, SM_RiseHour, SM_RiseMin);
    calcSunsetTime(Sys_Lati, Sys_Longi, SC_SysMon, SC_SysDay, SM_SetHour, SM_SetMin);
    calcSunstatusVect(Sys_Lati, Sys_Longi, SC_SysMon, SC_SysDay, SM_SunStatus, SM_TimeVect);
    //SM_ArPwrDens = calcSunArPwrDens(SM_Azimuth, SM_Elevation);

    tft.setTextColor(BGCOLOR);
    tft.setFont(Arial_8);
    tft.setCursor(1, 229);
    tft.print("Calc. Sunprofile..");

    delay(200);

    loadSdCardSettings();
    SettingsList.printListConsole();

    char query_name[] = "FC_TiFanOn";
    int item_id = SettingsList.getItemIdByName(query_name);
    float item_value = SettingsList.getItemValue(item_id);
    FC_TiFanOn = item_value;

    strcpy(query_name, "FC_TiFanOff");
    item_id = SettingsList.getItemIdByName(query_name);
    item_value = SettingsList.getItemValue(item_id);
    FC_TiFanOff = item_value;

    strcpy(query_name, "FC_TDewDiffOn");
    item_id = SettingsList.getItemIdByName(query_name);
    item_value = SettingsList.getItemValue(item_id);
    FC_TDewDiffOn = item_value;

    strcpy(query_name, "FC_TDewDiffOff");
    item_id = SettingsList.getItemIdByName(query_name);
    item_value = SettingsList.getItemValue(item_id);
    FC_TDewDiffOff = item_value;

    strcpy(query_name, "FC_TOutMin");
    item_id = SettingsList.getItemIdByName(query_name);
    item_value = SettingsList.getItemValue(item_id);
    FC_TOutMin = item_value;

   
    buildUI(UI);
}

void OS_10ms()
{
    TouchController();
}

void OS_1s()
{
    getSystemTime(&SC_SysHour, &SC_SysMin, &SC_SysSec);
    getSystemDate(&SC_SysDay, &SC_SysMon, &SC_SysYear);
    getDst(&SC_Dst);

    //EventTimer();
    readSensors();
    StmacFanCtlr();

    // sensor data MA filter

    refreshUI(UI);
}

void OS_10s()
{  
    calcSunAngle(Sys_Lati, Sys_Longi, SC_SysMon, SC_SysDay, SC_SysHour, SC_SysMin, &SM_Azimuth, &SM_Elevation);   
    refreshStatusBar();
    
    //SM_ArPwrDens = calcSunArPwrDens(SM_Azimuth, SM_Elevation);
    //StmacShadeCtlr();
}

void OS_60s()
{
    // save sensor data on SD card
    getSystemTime(&SC_SysHour, &SC_SysMin, &SC_SysSec);

    // hourly task
    if(SC_SysMin == 0)
    {
    }

    // dayly task
    if(SC_SysHour == 0 && SC_SysMin == 0)
    {   
        calcSunriseTime(Sys_Lati, Sys_Longi, SC_SysMon, SC_SysDay, SM_RiseHour, SM_RiseMin);
        calcSunsetTime(Sys_Lati, Sys_Longi, SC_SysMon, SC_SysDay, SM_SetHour, SM_SetMin);
    }

    /*
    * Datenlogger SD, jede Minute
    *   Datum: 2018-07-23
    *   Uhrzeit: 13:50:45
    *   TIn: 21.2
    *   HumIn: 45.2
    *   TDewIn: 15.2
    *   TOut: 24.1
    *   HumOut: 80.3
    *   TDewOut: 18.5
    *   StFan: 1
    *   TiFanOnTot: 13900
    */

    if (SD_flgSdCard) 
    {   LogFile = SD.open("LOGFILE.txt", FILE_WRITE); // no '-' in filename

        if(LogFile)
        {   //LogFile.println("2018-08-01; 15:27:23; 23.2; 48.3; 14.2; 34.5; 23.5; 16.4; 0; 12309;");
            
            LogFile.print(SC_SysYear);
            LogFile.print("-");
            if(SC_SysMon < 10) LogFile.print("0");
            LogFile.print(SC_SysMon);
            LogFile.print("-");
            if(SC_SysDay < 10) LogFile.print("0");
            LogFile.print(SC_SysDay);
            LogFile.print("; ");
            LogFile.print(SC_SysHour);
            LogFile.print(":");
            LogFile.print(SC_SysMin);
            LogFile.print(":");
            LogFile.print(SC_SysSec);
            LogFile.print("; ");
            LogFile.print(SI_TInFilt,3);
            LogFile.print("; ");
            LogFile.print(SI_HumInFilt,3);
            LogFile.print("; ");
            LogFile.print(SI_TDewIn,3);
            LogFile.print("; ");
            LogFile.print(SI_TOut,3);
            LogFile.print("; ");
            LogFile.print(SI_HumOut,3);
            LogFile.print("; ");
            LogFile.print(SI_TDewOut,3);
            LogFile.print("; ");
            LogFile.print(FC_StFan);
            LogFile.print("; ");
            LogFile.print((float)FC_TiFanOnTot/3600,2);
            LogFile.println("; ");
            
            
            
            LogFile.close();
            delay(100); // for SD card..
            Serial.println("Data written to SD Card.");
        }
        else
            Serial.println("Data storing failed!.");
        
    } 
}


void refreshStatusBar()
{
    tft.fillRect(20, 0, tft.width()-40, 19, BGCOLOR);
    tft.drawFastHLine(20,18,tft.width()-40, FGCOLOR);
    tft.setTextColor(FGCOLOR);
    tft.setFont(Arial_14);
    tft.setCursor(80, 2);
  
    char hour_str[2], minute_str[2], time_str[5];
    char day_str[2], month_str[2], year_str[4], date_str[10];
    char title_str[25];
  
    dtostrf(SC_SysDay, 2, 0, day_str);
    dtostrf(SC_SysMon, 2, 0, month_str);
    dtostrf(SC_SysYear, 4, 0, year_str);
    if(SC_SysDay < 10) day_str[0] = '0';
    if(SC_SysMon < 10) month_str[0] = '0';
  
    sprintf(date_str, "%s.%s.%s",day_str, month_str, year_str);
    //Serial.println(date_str);
    
    dtostrf(SC_SysHour+SC_Dst, 2, 0, hour_str);
    dtostrf(SC_SysMin, 2, 0, minute_str);
    if(SC_SysHour+SC_Dst < 10) hour_str[0] = '0';
    if(SC_SysMin < 10) minute_str[0] = '0';
  
    sprintf(time_str, "%s:%s",hour_str, minute_str);  
    //Serial.println(time_str);
  
    sprintf(title_str, "%s - %s",time_str, date_str);
    //Serial.println(title_str);
    
    tft.print(title_str);
 
}

void printBootLogo()
{
    char boot_str[] = "Keller Lueftung";
    char ver_str[] = "Ver. 2.0";

    printTextBox(tft.width()/2-140, tft.height()/2-25, ILI9341_BLACK, BGCOLOR, boot_str, 20, 20);
    printTextBox(tft.width()/2-140, tft.height()/2-25+24, ILI9341_BLACK, BGCOLOR, ver_str, 18, 18);
       
}

void plotLogData()
{

    int padding = 20;
    int plotWidth = tft.width()-2*padding;
    int plotHeight = tft.height()-2*padding;

    int numXTicks = 8;
    int spaceXTicks = plotWidth/numXTicks;

    int numYTicks = 6;
    int spaceYTicks = plotHeight/numYTicks;
    float YMax = 25;
    float YMin = -15;
    float YTick = (YMax-YMin)/numYTicks;
    
    // draw axes
    tft.drawFastVLine(padding,padding,plotHeight, FGCOLOR);
    tft.drawFastHLine(padding,tft.height()-padding,plotWidth, FGCOLOR);

    // draw x axis ticks
    for(int i=0; i<= numXTicks; i++)
    {   tft.drawFastVLine(padding+i*spaceXTicks,tft.height()-padding-2,5, FGCOLOR);   
    }

    // draw y axis ticks
    tft.setTextColor(FGCOLOR);
    tft.setFont(Arial_8);
    
    for(int j=0; j<= numYTicks; j++)
    {   tft.drawFastHLine(padding-2,padding+j*spaceYTicks,5, FGCOLOR);   
        
        // draw Y1 tick labels
        tft.setCursor(3, padding+j*spaceYTicks-4);
        tft.print(YMax-j*YTick,0);
    }

    // draw X tick labels
    tft.setTextColor(FGCOLOR);
    tft.setFont(Arial_8);

    //for(int i=1; i< plotWidth; i++)
    //{
        static int i = 1; 

    float data1 = SI_TDewIn;
    float data2 = SI_TDewOut;
    float data3 = FC_StFan;
    
    // draw data
    int data1_px, data2_px;

    static int data1_px_old; 
    static int data2_px_old; 
    
    if(i == 1)
    {   data1_px_old = plotHeight + padding - (int)mapfloat(data1, YMin, YMax, 0.0, plotHeight);
        data1_px_old = plotHeight + padding - (int)mapfloat(data2, YMin, YMax, 0.0, plotHeight);
    }    
      
        data1_px = plotHeight + padding - (int)mapfloat(data1, YMin, YMax, 0.0, plotHeight);
        data2_px = plotHeight + padding - (int)mapfloat(data2, YMin, YMax, 0.0, plotHeight);

        tft.drawLine(padding+i-1, data1_px_old, padding+i, data1_px, ILI9341_BLUE);
        tft.drawLine(padding+i-1, data2_px_old, padding+i, data2_px, ILI9341_RED);

        data1_px_old = data1_px;
        data2_px_old = data2_px;

        // draw sun status
        if(data3)
        {   tft.drawFastVLine(padding+i,padding, 4, ILI9341_ORANGE);

        }
        
        char str1[20], str2[20];
        sprintf(str1, "TTauIn [C]");
        sprintf(str2, "TTauOut [C]");
        
        printTextBox(210, 175, ILI9341_BLUE, BGCOLOR, str1,10, 13);
        printTextBox(210, 175+18, ILI9341_RED, BGCOLOR, str2,10, 13);
    
    i++;
    if(i >= plotWidth) i = 1;
    //}
  
}

void plotCanData(float data1, float data2)
{

    int padding = 30;
    int plotWidth = tft.width()-2*padding;
    float XMin = 0;
    float XMax = 5000;
    
    int plotHeight = tft.height()-2*padding;

    int numXTicks = 5;
    int spaceXTicks = plotWidth/numXTicks;
    float XTick = (XMax-XMin)/numXTicks;

    int numYTicks = 5;
    int spaceYTicks = plotHeight/numYTicks;
    float YMax = 200;
    float YMin = 0;
    float YTick = (YMax-YMin)/numYTicks;
    
    // draw axes
    tft.drawFastVLine(padding,padding,plotHeight, FGCOLOR);
    tft.drawFastHLine(padding,tft.height()-padding,plotWidth, FGCOLOR);

    tft.setTextColor(FGCOLOR);
    tft.setFont(Arial_8);

    // draw x axis ticks
    for(int i=0; i<= numXTicks; i++)
    {   tft.drawFastVLine(padding+i*spaceXTicks,tft.height()-padding-2,5, FGCOLOR);   
        // draw X1 tick labels
        tft.setCursor(padding+i*spaceXTicks-4, tft.height()-15);
        tft.print(XMin+i*XTick,0);
    }

    // draw y axis ticks
    tft.setTextColor(FGCOLOR);
    tft.setFont(Arial_8);
    
    for(int j=0; j<= numYTicks; j++)
    {   tft.drawFastHLine(padding-2,padding+j*spaceYTicks,5, FGCOLOR);   
        
        // draw Y1 tick labels
        tft.setCursor(3, padding+j*spaceYTicks-4);
        tft.print(YMax-j*YTick,0);
    }

    // draw X tick labels
    tft.setTextColor(FGCOLOR);
    tft.setFont(Arial_8);

    //float data1 = 1000; // rpm
    //float data2 = 120; // Nm
    
    // draw data
    int data1_px, data2_px;

    static int data1_px_old; 
    static int data2_px_old; 
        
    data1_px = padding + (int)mapfloat(data1, XMin, XMax, 0.0, plotWidth); // X
    data2_px = plotHeight + padding - (int)mapfloat(data2, YMin, YMax, 0.0, plotHeight); // Y

    //tft.drawLine(data1_px, data2_px, data1_px+5, data2_px+5, ILI9341_BLUE);
    //tft.drawLine(data1_px, data2_px+5, data1_px+5, data2_px, ILI9341_BLUE);
    tft.fillRect(data1_px_old-3, data2_px_old-3, 5, 5, ILI9341_WHITE);
    tft.fillRect(data1_px-3, data2_px-3, 5, 5, ILI9341_RED);

    data1_px_old = data1_px;
    data2_px_old = data2_px;

    // Legend
    char str1[20], str2[20];
    sprintf(str1, "SpdEng [rpm]");
    sprintf(str2, "TqEng [Nm]");
        
    printTextBox(210, 175, ILI9341_BLUE, BGCOLOR, str1,10, 13);
    printTextBox(210, 175+18, ILI9341_RED, BGCOLOR, str2,10, 13);
  
}

void plotSunProfile(uint16_t PosnY)
{ 
    char TiRiseH_num[2], TiRiseMin_num[2];
    char TiSetH_num[2], TiSetMin_num[2];
    char TiSunOnH_num[2], TiSunOnMin_num[2];
    char TiSunOffH_num[2], TiSunOffMin_num[2];
    char TiRise_str[6], TiSet_str[6];
    char TiSunOn_str[6], TiSunOff_str[6];
    
    dtostrf(SM_RiseHour+SC_Dst, 2, 0, TiRiseH_num);
    dtostrf(SM_RiseMin, 2, 0, TiRiseMin_num);
    if(SM_RiseHour+SC_Dst < 10) TiRiseH_num[0] = '0';
    if(SM_RiseMin < 10) TiRiseMin_num[0] = '0';
    dtostrf(SM_SetHour+SC_Dst, 2, 0, TiSetH_num);
    dtostrf(SM_SetMin, 2, 0, TiSetMin_num);
    if(SM_SetHour+SC_Dst < 10) TiSetH_num[0] = '0';
    if(SM_SetMin < 10) TiSetMin_num[0] = '0';
    
    sprintf(TiRise_str, "%s:%s", TiRiseH_num, TiRiseMin_num);
    sprintf(TiSet_str, "%s:%s", TiSetH_num, TiSetMin_num);

    // sunrise and sunset time in min after midnight
    int MinSunrise = 60*SM_RiseHour + SM_RiseMin;
    int MinSunset = 60*SM_SetHour + SM_SetMin;

    int idx_sunrise;
    int idx_sunset;

    // find sunrise time index of sunstatus
    for (int i=0; i<SZ_SUNSTATUS; i++)
    {   if (SM_TimeVect[i] >= MinSunrise)
        {   idx_sunrise = i;
            break;
        }
    }

    // find sunrise time index of sunstatus
    for (int i=0; i<SZ_SUNSTATUS; i++)
    {   if (SM_TimeVect[i] >= MinSunset)
        {   idx_sunset = i;
            break;
        }
    }

    // only plot between these two indices, only the day
    
    int MinTimerangeDay = SM_TimeVect[idx_sunset] - SM_TimeVect[idx_sunrise];

    int padding = 40;
    int plotWidth = tft.width()-2*padding;
    int plotHeight = tft.height()-2*padding;

    int numXTicks = 6;
    int spaceXTicks = plotWidth/numXTicks;

    int numYTicks = 5;
    int spaceYTicks = plotHeight/numYTicks;
    float YMax = 35;
    float YMin = 15;
    float YTick = (YMax-YMin)/numYTicks;

    int tickSunstatus = plotWidth/(24*4);

    float PxlTime = (float)MinTimerangeDay / (float)plotWidth;

    tft.fillRect(0, PosnY-16, tft.width(), 32, BGCOLOR);
    
    bool SunStatus = 0;
    bool SunStatus_old = 0;

    // plot sunstatus line
    for(int i=0; i < plotWidth; i++)
    {   
        int current_time = SM_TimeVect[idx_sunrise] + (float)i*PxlTime; 
        int x = padding+i;

        SunStatus = getSunStatusFromVect(current_time);
        if(SunStatus)
            tft.drawFastVLine(x,PosnY-2, 5, ILI9341_ORANGE);
        if(!SunStatus)
            tft.drawFastVLine(x,PosnY-2, 5, ILI9341_YELLOW);
        

        if((SunStatus == 1) && (SunStatus_old == 0))
        {   // print sun on time
            int h = current_time/(float)60;
            int m = current_time%60;
            dtostrf(h+SC_Dst, 2, 0, TiSunOnH_num);
            dtostrf(m, 2, 0, TiSunOnMin_num);
            if(h+SC_Dst < 10) TiSunOnH_num[0] = '0';
            if(m < 10) TiSunOnMin_num[0] = '0';
            sprintf(TiSunOn_str, "%s:%s", TiSunOnH_num, TiSunOnMin_num);
          
            tft.setTextColor(FGCOLOR);
            tft.setFont(Arial_8);
            tft.setCursor(x-13, PosnY-15);
            tft.print(TiSunOn_str);
        }
        if((SunStatus == 0) && (SunStatus_old == 1))
        {   // print sun off time
            int h = current_time/(float)60;
            int m = current_time%60;
            dtostrf(h+SC_Dst, 2, 0, TiSunOffH_num);
            dtostrf(m, 2, 0, TiSunOffMin_num);
            if(h+SC_Dst < 10) TiSunOffH_num[0] = '0';
            if(m < 10) TiSunOffMin_num[0] = '0';
            sprintf(TiSunOff_str, "%s:%s", TiSunOffH_num, TiSunOffMin_num);
          
            tft.setTextColor(FGCOLOR);
            tft.setFont(Arial_8);
            tft.setCursor(x-13, PosnY+7);
            tft.print(TiSunOff_str);
        }
        SunStatus_old = SunStatus;
        
    }

    // mark current system time on sun status bar
    int systemTimeMin = SC_SysMin + 60 * SC_SysHour;
    
    if(systemTimeMin >= MinSunrise && systemTimeMin <= MinSunset)
    {
        int x = map(systemTimeMin, MinSunrise, MinSunset, 0, plotWidth);
        tft.drawFastVLine(x+padding-1,PosnY-2, 5, ILI9341_BLACK);
    }
    
    
    // draw title
    tft.setTextColor(FGCOLOR);
    tft.setFont(Arial_14);
    //tft.setCursor(2*padding, 2);
    //tft.print("Temperature In/Out");

    // draw X tick labels
    tft.setTextColor(FGCOLOR);
    tft.setFont(Arial_8);
    
    tft.setCursor(4, PosnY-4);
    tft.print(TiRise_str);
    tft.setCursor(plotWidth+padding+5, PosnY-4);
    tft.print(TiSet_str);

    /* Symbole
    tft.setTextColor(FGCOLOR);
    tft.setCursor(150, 120);
    tft.setFont(AwesomeF000_24);
    tft.print((char)21);  // Haus
    tft.setFont(AwesomeF180_24);
    tft.print((char)5);  // Sonne
    tft.print((char)6);  // Mond
    tft.setFont(AwesomeF000_24);
    tft.print((char)1);  // Note
    */ 
}


void printTextBox(uint16_t x, uint16_t y, int fg_color, int bg_color, char* str, uint8_t FontSize, uint8_t numChar)
{   
    // CAUSES CPU FREEZE???
  
    //uint8_t FontSize = 14;
    uint8_t CharWidth = 10;

    switch(FontSize)
    {   case 8:  tft.setFont(Arial_8); break;
        case 10: tft.setFont(Arial_10); break;
        case 12: tft.setFont(Arial_12); break;
        case 14: tft.setFont(Arial_14); break;
        case 16: tft.setFont(Arial_16); break;
        case 20: tft.setFont(Arial_20); break;
        case 24: tft.setFont(Arial_24); break;
        default: tft.setFont(Arial_12);
    }
    
    tft.setTextColor(fg_color, bg_color);

    tft.fillRect(x, y, numChar*tft.strPixelLen("5"), FontSize, bg_color);
    tft.setCursor(x, y);
    tft.print(str);
 
}

void refreshUI(int ui_id)
{
    noInterrupts();
    
    switch(ui_id)
    {   case UI_MENU: drawMenu(); break;
        case UI_SETTINGS0: drawSettings(0); break;
        case UI_SETTINGS1: drawSettings(1); break;
        case UI_INFO: drawInfo(); break;
        case UI_PLOT: plotLogData(); break;
        case UI_KEYPAD: drawKeypad(); break;
        case UI_CLOCK:
            drawClock(SC_SysHour, SC_SysMin, SC_SysSec, 90, tft.height()/2);
            //drawGauge(0, 230, tft.height()/2);
            break;
        case UI_CAN: plotCanData(CAN_SpdEng, CAN_TqEng); break;
        case UI_NN: drawNeurNet(); break;
        default: break;
    }
    
    interrupts();
    
}

void buildUI(int ui_id)
{   
    noInterrupts();

    tft.fillScreen(BGCOLOR);
    
    clearList();

    if(ui_id == UI_MENU)
    {
        // draw a button, center label
        char label5[10] = "Monitor";
        addButton(0, 20, 40, 120, 40, label5);

        char label6[10] = "Verlauf";
        addButton(1, 170, 40, 120, 40, label6);

        char label4[10] = "Uhr";
        addButton(2, 20, 90, 120, 40, label4);

        char label3[10] = "Einstell.";
        addButton(3, 170, 90, 120, 40, label3);

        char label1[10] = "CAN";
        addButton(4, 20, 140, 120, 40, label1);
        
        strcpy(label1,"NeurNet");
        addButton(5, 170, 140, 120, 40, label1);
    }

    if(ui_id == UI_SETTINGS0)
    {
        // draw a button, center label
        char label[6] = "+";
        addButton(0, 230, 100, 60, 40, label);

        char label2[6] = "-";
        addButton(1, 20, 100, 60, 40, label2);

        char label3[6] = "+";
        addButton(2, 230, 175, 60, 40, label3);

        char label4[6] = "-";
        addButton(3, 20, 175, 60, 40, label4);

        //char label7[10] = "Slider1";
        //addSlider(4, 20, 200, 260, 20, label7, 50);

        char label5[5] = "H";
        addButton(5, 260, 25, 40, 40, label5);

        char label6[5] = "<";
        addButton(6, TFT_WIDTH/2-50, 25, 40, 40, label6);
        char label8[5] = ">";
        addButton(7, TFT_WIDTH/2+10, 25, 40, 40, label8);
    }
    
    if(ui_id == UI_SETTINGS1)
    {
        char label5[5] = "H";
        addButton(5, 260, 25, 40, 40, label5);

        char label6[5] = "<";
        addButton(6, TFT_WIDTH/2-50, 25, 40, 40, label6);
        char label8[5] = ">";
        addButton(7, TFT_WIDTH/2+10, 25, 40, 40, label8);

        char label[6] = "+";
        addButton(0, 230, 100, 60, 40, label);

        char label2[6] = "-";
        addButton(1, 20, 100, 60, 40, label2);

        char label3[6] = "+";
        addButton(2, 230, 175, 60, 40, label3);

        char label4[6] = "-";
        addButton(3, 20, 175, 60, 40, label4);

//        char label7[10] = "Luefter";
//        addSwitch(8, 20, 90, 40, 40, label7, 1);
//
//        char label9[15] = "Beschattung";
//        addSwitch(9, 20, 130, 40, 40, label9, 0);
//
//        char label10[10] = "TV + Wifi";
//        addSwitch(10, 20, 170, 40, 40, label10, 0);
    }
    
    if(ui_id == UI_INFO)
    {
        char label3[10] = "H";
        addButton(0, 260, 25, 40, 40, label3);

        char label4[10] = "R";
        addButton(1, 260, 75, 40, 40, label4);
    }

    if(ui_id == UI_PLOT)
    {
        char label7[10] = "H";
        addButton(0, 260, 25, 40, 40, label7);
    }
    if(ui_id == UI_CAN)
    {
        char label7[10] = "H";
        addButton(0, 260, 25, 40, 40, label7);
    }
    if(ui_id == UI_NN)
    {
        char label7[10] = "H";
        addButton(0, 260, 25, 40, 40, label7);
    }

    if(ui_id == UI_KEYPAD)
    {
        char label8[10] = "1";
        addButton(0, 20, 50, 50, 50, label8);
        char label9[10] = "2";
        addButton(1, 80, 50, 50, 50, label9);
        char label10[10] = "3";
        addButton(2, 140, 50, 50, 50, label10);
        char label17[10] = "<<";
        addButton(10, 200, 50, 50, 50, label17);

        char label11[10] = "4";
        addButton(3, 20, 110, 50, 50, label11);
        char label12[10] = "5";
        addButton(4, 80, 110, 50, 50, label12);
        char label13[10] = "6";
        addButton(5, 140, 110, 50, 50, label13);

        char label14[10] = "7";
        addButton(6, 20, 170, 50, 50, label14);
        char label15[10] = "8";
        addButton(7, 80, 170, 50, 50, label15);
        char label16[10] = "9";
        addButton(8, 140, 170, 50, 50, label16);
        char label18[10] = "0";
        addButton(9, 200, 170, 50, 50, label18);
        char label19[10] = "OK";
        addButton(11, 260, 170, 50, 50, label19);
    }

    if(ui_id == UI_CLOCK)
    {
        char label7[10] = "H";
        addButton(0, 260, 25, 40, 40, label7);
    }
    
    refreshStatusBar();
    refreshUI(UI);

    interrupts();

    // printList(); // terminal
}

void drawMenu()
{
  
}

void drawSettings(int page)
{
    if(page == 0)
    {   tft.setTextColor(FGCOLOR, BGCOLOR);
        tft.setFont(Arial_10);
        tft.setCursor(20, 20);
        tft.print("1/2");  
      
        tft.setTextColor(FGCOLOR, BGCOLOR);
        tft.setFont(Arial_10);
        tft.setCursor(20, 85);
        tft.print("Min Temperatur Aussen:");
    
        tft.setCursor(20, 160);
        tft.print("Tau-Temperatur Differenz ON:");
    
        char data_num[5];
        char data_str[20];
        dtostrf(FC_TOutMin, 3, 1, data_num);
        sprintf(data_str, " %s C ", data_num);
        printTextBox(100, 110, ILI9341_BLACK, BGCOLOR, data_str, 24, 6);   
         
        dtostrf(FC_TDewDiffOn, 3, 1, data_num);
        sprintf(data_str, " %s C ", data_num);
        printTextBox(100, 185, ILI9341_BLACK, BGCOLOR, data_str, 24, 6); 
    }
    else if(page == 1)
    {   tft.setTextColor(FGCOLOR, BGCOLOR);
        tft.setFont(Arial_10);
        tft.setCursor(20, 20);
        tft.print("2/2");
              
        tft.setTextColor(FGCOLOR, BGCOLOR);
        tft.setFont(Arial_10);
        tft.setCursor(20, 85);
        tft.print("ON/OFF Zeitintervall:"); 
    
        tft.setCursor(20, 160);
        tft.print("Tau-Temperatur Differenz OFF:");
    
        char data_num[5];
        char data_str[20];
        
        dtostrf(FC_TiFanOn/60, 3, 0, data_num);
        sprintf(data_str, " %s min  ", data_num);
        printTextBox(100, 110, ILI9341_BLACK, BGCOLOR, data_str, 24, 6);    
        
        dtostrf(FC_TDewDiffOff, 3, 1, data_num);
        sprintf(data_str, " %s C ", data_num);
        printTextBox(100, 185, ILI9341_BLACK, BGCOLOR, data_str, 24, 6); 
    }

}

void drawInfo()
{
    // Print temperature sensor values
    char TIn_num[5], TOut_num[5], HumIn_num[5], HumOut_num[5], DewIn_num[5], DewOut_num[5];
    char TIn_str[20], TOut_str[20], HumIn_str[20], HumOut_str[20], DewIn_str[25], DewOut_str[25];
    
    dtostrf(SI_TIn, 3, 1, TIn_num);
    sprintf(TIn_str, "%s C ", TIn_num);
    dtostrf(SI_TOut, 3, 1, TOut_num);
    sprintf(TOut_str, "%s C", TOut_num);

    dtostrf(SI_HumIn, 3, 1, HumIn_num);
    sprintf(HumIn_str, "%s %% ", HumIn_num);
    dtostrf(SI_HumOut, 3, 1, HumOut_num);
    sprintf(HumOut_str, "%s %%", HumOut_num);

     
    printTextBox(60, 30, ILI9341_BLUE, BGCOLOR, TIn_str, 24, 7);
    printTextBox(165, 30, ILI9341_BLUE, BGCOLOR, HumIn_str, 16, 6);
    tft.setFont(AwesomeF000_24);
    tft.setCursor(20, 26);
    tft.print((char)21);  // Haus
    
    printTextBox(60, 30+28, ILI9341_RED, BGCOLOR, TOut_str, 24, 7);
    printTextBox(165, 30+28, ILI9341_RED, BGCOLOR, HumOut_str, 16, 6);
    tft.setFont(AwesomeF000_24);
    tft.setCursor(22, 26+28);
    tft.print((char)27);  // Pfeil up

    dtostrf(SI_TDewIn, 3, 1, DewIn_num);
    sprintf(DewIn_str, "Taup. innen = %s C ", DewIn_num);
    dtostrf(SI_TDewOut, 3, 1, DewOut_num);
    sprintf(DewOut_str, "Taup. aussen = %s C", DewOut_num);
    //sprintf(tmp1, "Taup. aussen = %s %% ", LOut_num);
    //sprintf(tmp1, "AZ = %s deg", Azi_num);
    //sprintf(tmp2, "EL = %s deg", Ele_num);
    //sprintf(tmp3, "P = %s W/m2", Pwr_num);

    printTextBox(20, 105, ILI9341_BLACK, BGCOLOR, DewIn_str, 16, 20);    
    printTextBox(20, 127, ILI9341_BLACK, BGCOLOR, DewOut_str, 16, 20);

    char Win_str[30];
    uint32_t time_sec = 0;
    if(FC_StFan == FANON)
    {   time_sec = now()-FC_TiFanStart;
    }
    
    sprintf(Win_str, "Luefter: %i (%imin / %.1fh)", FC_StFan, time_sec/60, (float)FC_TiFanOnTot/3600);
    printTextBox(20, 149, ILI9341_BLACK, BGCOLOR, Win_str, 16, 25);

    plotSunProfile(210);
  
}


void drawKeypad()
{   //char del_str[10] = "        ";
    //printTextBox(20, 25, ILI9341_BLACK, BGCOLOR, del_str, 20, 13); 
    printTextBox(20, 25, ILI9341_BLACK, BGCOLOR, keypad_str, 20, 13); 
}

void drawNeurNet()
{   char str[32];
    strcpy(str,"Simple Neuronal Net");
    printTextBox(20, 25, ILI9341_BLACK, BGCOLOR, str, 32, 13);
    
    /* Draw NN layout */
    for(int i=0; i<NrInputNodes; i++)
    {   tft.drawCircle(30+(i*25), 130, 10, FGCOLOR);
        //tft.drawLine(30+(i*25), 130, 30+(i*25), 160, FGCOLOR);
    }
    for(int i=0; i<NrHiddenNodes; i++)
    {   tft.drawCircle(30+(i*25), 160, 10, FGCOLOR);
    }
    for(int i=0; i<NrOutputNodes; i++)
    {   tft.drawCircle(30+(i*25), 190, 10, FGCOLOR);
    }
    
    strcpy(str,"Init NN..");
    printTextBox(20, 50, ILI9341_BLACK, BGCOLOR, str, 12, 13);
    
    initNeurNet();
    //printTrainingData();
    
    strcpy(str,"Init NN.. OK");
    printTextBox(20, 50, ILI9341_BLACK, BGCOLOR, str, 12, 13);
    
    strcpy(str,"Norm. training data..");
    printTextBox(20, 65, ILI9341_BLACK, BGCOLOR, str, 12, 13);
    
    normTrainingData();
    
    strcpy(str,"Norm. training data.. OK");
    printTextBox(20, 65, ILI9341_BLACK, BGCOLOR, str, 12, 13);
    
    strcpy(str,"Train NN..");
    printTextBox(20, 80, ILI9341_BLACK, BGCOLOR, str, 12, 13);
    
    long TrainingCycles = trainNeurNet(20000);
    
    sprintf(str,"Train NN.. %d OK",TrainingCycles);
    printTextBox(20, 80, ILI9341_BLACK, BGCOLOR, str, 12, 32);
    
}

void drawClock(int h, int m, int s, int center_x, int center_y)
{   //char label[10] = "Time";
    //printTextBox(20, 25, ILI9341_BLACK, BGCOLOR, label, 20, 13);
    
    //int center_x = tft.width()/2;
    //int center_y = tft.height()/2;
    int radius = 70;
    
    tft.drawCircle(center_x, center_y, radius+1, FGCOLOR); 
    tft.drawCircle(center_x, center_y, radius, FGCOLOR); 
    tft.drawCircle(center_x, center_y, radius-1, FGCOLOR); 
    tft.fillCircle(center_x, center_y, radius-5, BGCOLOR);

    // seconds
    float alpha_s = 180 - s*6;
    tft.drawLine(center_x, center_y, center_x+(radius-5)*sin(alpha_s/180*PI), center_y+(radius-5)*cos(alpha_s/180*PI), ILI9341_RED);
    tft.fillCircle(center_x+(radius-15)*sin(alpha_s/180*PI), center_y+(radius-15)*cos(alpha_s/180*PI), 3, ILI9341_RED);

    // minutes
    float alpha_m = 180 - m*6;
    Serial.println(alpha_m);
    tft.drawLine(center_x, center_y, center_x+(radius*0.8)*sin(alpha_m/180*PI), center_y+(radius*0.8)*cos(alpha_m/180*PI), FGCOLOR);


    /* Todo
     * move hour hand according to minutes
     */

    // hours
    float alpha_h = 180 - h*30 - 30;
    tft.drawLine(center_x, center_y, center_x+(radius*0.4)*sin(alpha_h/180*PI), center_y+(radius*0.4)*cos(alpha_h/180*PI), FGCOLOR);

    // 60 devisions
    for(int i=0; i < 60; i++)
    {
        int x_outer = center_x + radius * sin((float)i*6/180*PI);
        int y_outer = center_y + radius * cos((float)i*6/180*PI);

        int len = 5;
        if((i%5) == 0) len = 10;

        int x_inner = center_x + (radius - len) * sin((float)i*6/180*PI);
        int y_inner = center_y + (radius - len) * cos((float)i*6/180*PI);
        
        tft.drawLine(x_inner, y_inner, x_outer, y_outer, FGCOLOR);
    }

    // axcel
    tft.fillCircle(center_x, center_y, 3, FGCOLOR);

}

void drawGauge(float percent, int center_x, int center_y)
{   
    int radius = 50;
    
    tft.drawCircle(center_x, center_y, radius+1, FGCOLOR); 
    tft.drawCircle(center_x, center_y, radius, FGCOLOR); 
    tft.drawCircle(center_x, center_y, radius-1, FGCOLOR); 
    tft.fillCircle(center_x, center_y, radius-5, BGCOLOR);

    // hand
    float alpha_s = 315 - mapfloat(percent,0,100,0,270);
    tft.drawLine(center_x, center_y, center_x+(radius-5)*sin(alpha_s/180*PI), center_y+(radius-5)*cos(alpha_s/180*PI), ILI9341_RED);
    //tft.drawLine(center_x+1, center_y+1, center_x+(radius-5)*sin(alpha_s/180*PI)+1, center_y+(radius-5)*cos(alpha_s/180*PI)+1, ILI9341_RED);
    //tft.drawLine(center_x-1, center_y-1, center_x+(radius-5)*sin(alpha_s/180*PI)-1, center_y+(radius-5)*cos(alpha_s/180*PI)-1, ILI9341_RED);
    
    //tft.fillCircle(center_x+(radius-15)*sin(alpha_s/180*PI), center_y+(radius-15)*cos(alpha_s/180*PI), 3, ILI9341_RED);

    // 60 devisions
    for(int i=0; i < 11; i++)
    {
        float alpha = ((float)i*28.6 + 26)/180*PI;
      
        int x_outer = center_x + radius * sin(alpha);
        int y_outer = center_y + radius * cos(alpha);

        int len = 10;
        //if((i%2) == 0) len = 10;

        int x_inner = center_x + (radius - len) * sin(alpha);
        int y_inner = center_y + (radius - len) * cos(alpha);
        
        tft.drawLine(x_inner, y_inner, x_outer, y_outer, FGCOLOR);
    }

    // axcel
    tft.fillCircle(center_x, center_y, 3, FGCOLOR);

}



void initSdCard()
{
    Serial.print("Looking for SD card.. ");
    delay(100); // for SD card..
    
    if (!SD.begin(SD_CS))  
    {   Serial.println("no card found.");
        SD_flgSdCard = 0;
    }
    else
    {   Serial.println("found.");
        SD_flgSdCard = 1;
        delay(100); // for SD card..

        // delete old template file and write a new one
        if(SD.remove("ReadMe.txt")) 
            Serial.println("Old ReadMe.txt removed.");
        else 
        {   Serial.println("Old ReadMe.txt could not be removed.");
        }


        // write the new template file
        Serial.print("Writing ReadMe.txt file.. ");
        LogFile = SD.open("README.txt", FILE_WRITE); // no '-' in filename

   /*
    * Datenlogger SD, jede Minute
    *   Datum: 2018-07-23
    *   Uhrzeit: 13:50:45
    *   TIn: 21.2
    *   HumIn: 45.2
    *   TDewIn: 15.2
    *   TOut: 24.1
    *   HumOut: 80.3
    *   TDewOut: 18.5
    *   StFan: 1
    *   TiFanOnTot: 13900
    */

        // if the file opened okay, write to it:
        if (LogFile) 
        {   LogFile.println("Datum; Zeit; TIn; HumIn; TDewIn; TOut; HumOut; TDewOut; StFan; TiFanOnTot;");

            LogFile.close();
            delay(100); // for SD card..
            Serial.println("done.");
        } 
        else 
        {
            // if the file didn't open, print an error:
            LogFile.close();
            Serial.println("failed!");
        }
    }
      
}

void loadSdCardSettings()
{   Serial.println("Loading settings from SD card.. ");  
  
    if (!SD_flgSdCard)  
    {   Serial.println("no card found!");
    }
    else
    {   Serial.println("found.");
        delay(100); // for SD card..

        Serial.print("Open file Settings.txt.. ");
        SettingsFile = SD.open("Settings.txt",FILE_READ);
        
        char c = 0;
        char str[32];
        int i = 0;
        int token = 1;
        char tempName[32];
        float tempValue = 0;
        int8_t tempLight[4];

        strcpy(str,"                             ");
    
        // Settings parser
        if (SettingsFile) 
        {   Serial.println("done."); 
            // read from the file until there's nothing else in it:
            while (SettingsFile.available()) 
            {   c = SettingsFile.read();

                if(c == '\n') 
                {   token = 1;
                    i = 0;
                }
                                                                     
                switch(token)
                {   // label name
                    case 1: 
                        if(c != '=' && c != ' ' && c != '\n') 
                        {   tempName[i++] = c;
                            if(i>30) i = 30;
                            //Serial.write(c);
                        }
                        else if(c == '=')
                        {   tempName[i] = '\0';
                            i = 0;
                            token++;   
                            //Serial.print(" ");              
                        }   
                        break;
                        
                    // value
                    case 2: 
                        
                        if(c == ';')
                        {   tempValue = atof(str);
                            i = 0;
                            strcpy(str,"                               ");
                            
                            token++;    
                            //Serial.print(" ");    
                            //Serial.print(tempValue,3);   
                            
                            SettingsList.addItemHead(tempName, tempValue);        
                        }
                        else if(c != ' ')
                        {   str[i++] = c;
                            if(i>30) i = 30;
                            //Serial.write(c);
                        }      
                        break;
   
                    // end of song entry, start with token 1
                    case 3:
                        if(c == '\n')
                        {   token = 1;
                            i = 0;
                            //Serial.println();
                        }
                        break;
                }
            }
            
            SettingsFile.close();
        } 
        else 
        {
            // if the file didn't open, print an error:
            SettingsFile.close();
            Serial.println("failed!");
            // load default song list
            //defaultSongList();
        }
    }
}

void loadSdCardTrainingData()
{
    Serial.println("Loading training data from SD card.. ");
    
    if (!SD_flgSdCard)
    {   Serial.println("no card found!");
    }
    else
    {   Serial.println("found.");
        delay(100); // for SD card..
        
        Serial.print("Open file Train.txt.. ");
        TrainingFile = SD.open("Train.txt",FILE_READ);
        
        char c = 0;
        char str[32];
        int i = 0;
        int token = 1;
        int numData = 0;
        int numPattern = 0;
        char tempName[32];
        float tempValue = 0;
        int8_t tempLight[4];
        
        strcpy(str,"                             ");
        
        // Settings parser
        if (TrainingFile)
        {   Serial.println("done.");
            // read from the file until there's nothing else in it:
            while (TrainingFile.available())
            {   c = TrainingFile.read();
                
                if(c == '\n')
                {   token = 1;
                    i = 0;
                }
                
                switch(token)
                {   // input value
                    case 1:
                        if(c == ';')
                        {   tempValue = atof(str);
                            i = 0;
                            strcpy(str,"                               ");
                            
                            numData++;
                            
                            //Input[numPattern][numData] = tempValue;
                            
                            Serial.print(tempValue);
                            Serial.print(" ");
                            
                            if(numData >= NrInputNodes)
                            {   token++;
                                numData = 0;
                                Serial.print(" O: ");
                            }
                        }
                        else if(c != ' ')
                        {   str[i++] = c;
                            if(i>30) i = 30;
                        }
                        break;
                        
                        // output value
                    case 2:
                        
                        if(c == ';')
                        {   tempValue = atof(str);
                            i = 0;
                            strcpy(str,"                               ");
                            
                            numData++;
                            
                            //Output[numPattern][numData] = tempValue;
                            
                            Serial.print(tempValue);
                            Serial.print(" ");
                            
                            if(numData >= NrOutputNodes)
                            {   token++;
                                numData = 0;
                                Serial.println();
                            }
                        }
                        else if(c != ' ')
                        {   str[i++] = c;
                            if(i>30) i = 30;
                        }
                        break;
                        
                    case 3:
                        if(c == '\n')
                        {   token = 1;
                            i = 0;
                            numData = 0;
                            Serial.println();
                            
                            if(numPattern < (NrPattern-1)) numPattern++;
                        }
                        break;
                }
            }
            
            TrainingFile.close();
            Serial.println("Training data loaded.");
        }
        else
        {
            // if the file didn't open, print an error:
            TrainingFile.close();
            Serial.println("failed!");
            // load default song list
            //defaultSongList();
        }
    }
}



