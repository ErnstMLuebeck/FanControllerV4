


#include "ecu_reader.h"
#include <FlexCAN.h>


/* C++ wrapper */
ecu_reader_class::ecu_reader_class() 
{

 
}

uint8_t ecu_reader_class::init(uint32_t baud) 
{

  Can0.begin(baud);
  return 0;
}

uint8_t ecu_reader_class::request(uint8_t pid,  int *engine_data) 
{

  CAN_message_t can_MsgRx,can_MsgTx;
  
  can_MsgTx.buf[0] = 0x02;  
  can_MsgTx.buf[1] = 0x01;
  can_MsgTx.buf[2] = pid;  
  can_MsgTx.buf[3] = 0;
  can_MsgTx.buf[4] = 0;  
  can_MsgTx.buf[5] = 0;
  can_MsgTx.buf[6] = 0;  
  can_MsgTx.buf[7] = 0;
  can_MsgTx.len = 8; 
  can_MsgTx.ext = 0; 
//  can_MsgTx.flags.extended = 0; 
//  can_MsgTx.flags.remote = 0;
  can_MsgTx.id = PID_REQUEST;
//  can_MsgTx.timeout = 500;

  Can0.write(can_MsgTx);  
/*

  elapsedMillis waiting;     // "waiting" starts at zero
  
  while (waiting < 5) 
  {  

    if(Can0.read(can_MsgRx)) 
    { 
        if((can_MsgRx.id == PID_REPLY) && (can_MsgRx.buf[2] == pid))
        { 
            switch(can_MsgRx.buf[2])
            {   // Details from http://en.wikipedia.org/wiki/OBD-II_PIDs
                case ENGINE_RPM:              //   ((A*256)+B)/4    [RPM]
                    *engine_data =  ((can_MsgRx.buf[3]*256) + can_MsgRx.buf[4])/4;
                    //sprintf(buffer,"%d  ",(int) engine_data);
                    break;
                
                case ENGINE_COOLANT_TEMP:     //     A-40              [degree C]
                    *engine_data =  can_MsgRx.buf[3] - 40;
                    //sprintf(buffer,"%d degC ",(int) engine_data);
                
                break;
                
                case VEHICLE_SPEED:         // A                  [km]
                    *engine_data =  can_MsgRx.buf[3];
                    //sprintf(buffer,"%d km ",(int) engine_data);
                
                break;

                case MAF_SENSOR:               // ((256*A)+B) / 100  [g/s]
                    *engine_data =  ((can_MsgRx.buf[3]*256) + can_MsgRx.buf[4])/100;
                    //sprintf(buffer,"%d g/s",(int) engine_data);
                
                break;

                case O2_VOLTAGE:            // A * 0.005   (B-128) * 100/128 (if B==0xFF, sensor is not used in trim calc)
                    *engine_data = can_MsgRx.buf[3]*0.005;
                    //sprintf(buffer,"%d v ",(int) engine_data);
                    break;

                case THROTTLE:            //
                    *engine_data = int((can_MsgRx.buf[3]*100)/255);
                    //sprintf(buffer,"%d %% ",(int) engine_data);
                    break;

                case ENGINE_TRQ_REF:
                    *engine_data = int(can_MsgRx.buf[3]*256 + can_MsgRx.buf[4]);
                    break;

                case ENGINE_TRQ_PERC:
                    *engine_data = int(can_MsgRx.buf[3]-125);
     
                break;
            }
            return 1;
       }     
    }

 
  } // while
  */
  return 0;

}

uint8_t ecu_reader_class::reply() 
{

  CAN_message_t can_MsgRx,can_MsgTx;
  
  can_MsgTx.buf[0] = 0x02;  
  can_MsgTx.buf[1] = 0x01;
  can_MsgTx.buf[2] = 0;  
  can_MsgTx.buf[3] = 0;
  can_MsgTx.buf[4] = 0;  
  can_MsgTx.buf[5] = 0;
  can_MsgTx.buf[6] = 0;  
  can_MsgTx.buf[7] = 0;
  can_MsgTx.len = 8; 
  can_MsgTx.ext = 0; 
//  can_MsgTx.flags.extended = 0; 
//  can_MsgTx.flags.remote = 0;
  can_MsgTx.id = PID_REPLY;
//  can_MsgTx.timeout = 500;

    uint16_t SpdEng = 666;
    uint16_t TqEngRef = 220;
    uint16_t TqEngPerc = 13;
    uint16_t TCoolt = 85;
    uint16_t SpdVhcl = 52;
    uint16_t PosnThrottle = 7;


    if(Can0.read(can_MsgRx)) 
    { 
        if(can_MsgRx.id == PID_REQUEST)
        { 
            switch(can_MsgRx.buf[2])
            {   /* Details from http://en.wikipedia.org/wiki/OBD-II_PIDs */
                case ENGINE_RPM:              //   ((A*256)+B)/4    [RPM]
                    //*engine_data =  ((can_MsgRx.buf[3]*256) + can_MsgRx.buf[4])/4;
                    can_MsgTx.buf[2] = can_MsgRx.buf[2]; 
                    can_MsgTx.buf[3] = (4*SpdEng & 0x0000FF00) >> 8;
                    can_MsgTx.buf[4] = (4*SpdEng & 0x000000FF);
                    //sprintf(buffer,"%d  ",(int) engine_data);
                    break;
                
                case ENGINE_COOLANT_TEMP:     //     A-40              [degree C]
                    //*engine_data =  can_MsgRx.buf[3] - 40;
                    //sprintf(buffer,"%d degC ",(int) engine_data);
                    can_MsgTx.buf[2] = can_MsgRx.buf[2]; 
                    can_MsgTx.buf[3] = (TCoolt + 40);
                
                break;
                
                case VEHICLE_SPEED:         // A                  [km]
                    //*engine_data =  can_MsgRx.buf[3];
                    //sprintf(buffer,"%d km ",(int) engine_data);
                    can_MsgTx.buf[2] = can_MsgRx.buf[2]; 
                    can_MsgTx.buf[3] = SpdVhcl;
                
                break;

                case MAF_SENSOR:               // ((256*A)+B) / 100  [g/s]
                    //*engine_data =  ((can_MsgRx.buf[3]*256) + can_MsgRx.buf[4])/100;
                    //sprintf(buffer,"%d g/s",(int) engine_data);
                
                break;

                case O2_VOLTAGE:            // A * 0.005   (B-128) * 100/128 (if B==0xFF, sensor is not used in trim calc)
                    //*engine_data = can_MsgRx.buf[3]*0.005;
                    //sprintf(buffer,"%d v ",(int) engine_data);
                    break;

                case THROTTLE:            //
                    //*engine_data = int((can_MsgRx.buf[3]*100)/255);
                    //sprintf(buffer,"%d %% ",(int) engine_data);
                    can_MsgTx.buf[2] = can_MsgRx.buf[2]; 
                    can_MsgTx.buf[3] = PosnThrottle/100*265;
                    break;

                case ENGINE_TRQ_REF:
                    //*engine_data = int(can_MsgRx.buf[3]*256 + can_MsgRx.buf[4]);
                    can_MsgTx.buf[2] = can_MsgRx.buf[2]; 
                    can_MsgTx.buf[3] = (TqEngRef & 0x0000FF00) >> 8;
                    can_MsgTx.buf[4] = (TqEngRef & 0x000000FF);
                    break;

                case ENGINE_TRQ_PERC:
                    //*engine_data = int(can_MsgRx.buf[3]-125);
                    can_MsgTx.buf[2] = can_MsgRx.buf[2]; 
                    can_MsgTx.buf[3] = TqEngPerc + 125;
     
                break;
            }
            Can0.write(can_MsgTx); 
            Serial.println("Data sent!");
            return 1;
       }     
  }

  
  return 0;

}
