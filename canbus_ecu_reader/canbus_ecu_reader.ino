/* CAN Bus ECU Reader
 * www.skpang.co.uk
 * V1.0 Jan 2017
 *   
 * Uses Teensy CAN-Bus breakout board:
 * http://skpang.co.uk/catalog/teensy-canbus-breakout-board-include-teensy-32-p-1507.html
 * 
 * and 128x64 OLED display:
 * http://skpang.co.uk/catalog/oled-128x64-display-for-teensy-breakout-board-p-1508.html
 * 
 * Also requres new FlexCAN libarary
 * https://github.com/collin80/FlexCAN_Library
 * 
 * 
 */
 
#include "ecu_reader.h"
#include <FlexCAN.h>
#include <SPI.h>
#include <Wire.h>

ecu_reader_class ecu_reader;

ecu_t old_ecu;

void setup() 
{ 
  ecu_reader.init(500000);
  delay(1000);
  Serial.println(F("OBD II Reader:"));

  old_ecu.engine_rpm = -1;
  old_ecu.coolant_temp = 0;
  old_ecu.vehicle_speed = -1;
  old_ecu.throttle_position = -1;
}

void loop() 
{

  int engine_data;

  // OBD READER
  if(0)
  {
      if(ecu_reader.request(ENGINE_RPM,&engine_data))
      {    Serial.print("RPM = ");
           Serial.println(engine_data);
      } 
      else Serial.println("No RPM Data.");

    
      if(ecu_reader.request(ENGINE_TRQ_PERC,&engine_data))
      {   Serial.print("TRQ_PERC = ");
          Serial.println(engine_data);
      } 
      else Serial.println("No TRQ_PERC Data.");

      
      if(ecu_reader.request(ENGINE_TRQ_REF,&engine_data))
      {    Serial.print("TRQ_REF = ");
           Serial.println(engine_data);
      } 
      else Serial.println("No TRQ_REF Data.");

      if(ecu_reader.request(ENGINE_COOLANT_TEMP,&engine_data))
      {    Serial.print("TCOOLT = ");
           Serial.println(engine_data);
      } 
      else Serial.println("No TCOOLT Data.");

      if(ecu_reader.request(VEHICLE_SPEED,&engine_data))
      {    Serial.print("SPDVEH = ");
           Serial.println(engine_data);
      } 
      else Serial.println("No SPDVEH Data.");

      

  }
  else // ECU
  {   
      ecu_reader.reply();
  }
}
