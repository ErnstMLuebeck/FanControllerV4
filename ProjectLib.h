#ifndef PROJECTLIB_H
#define PROJECTLIB_H

#include "WConstants.h"
#include <math.h>

float LookupTable(float axis[], float data[], uint8_t dim, float input);
int LookupTableInt(int axis[], int data[], int dim, int input);
float saturate(float in, float LimLwr, float LimUpr);
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max);

float LookupTable(float axis[], float data[], uint8_t dim, float input)
{   
    float output = 0;
    float factor = 0;

    // saturate under-/overflow
    if(input < axis[0]) return(data[0]);

    if(input > axis[dim-1]) return(data[dim-1]);
  
    for(int i=0; i < dim; i++)
    {   //Serial.println(axis[i]);
        if(axis[i] == input) return(data[i]);
        
        if(axis[i] > input)
        {   // linear interpolation
            factor = (float)(input-axis[i-1])/(float)(axis[i]-axis[i-1]);
            output = factor*(data[i]-data[i-1])+data[i-1];
            //Serial.println(factor);  
            return(output);
        }
    }
}

int LookupTableInt(int axis[], int data[], int dim, int input)
{   
    int output = 0;
    float factor = 0;

    // saturate overflow
    if(input < axis[0]) return(data[0]);

    if(input > axis[dim-1]) return(data[dim-1]);
  
    for(int i=0; i < dim; i++)
    {   //Serial.println(axis[i]);
        if(axis[i] == input) return(data[i]);
        
        if(axis[i] > input)
        {   // linear interpolation
            factor = (float)(input-axis[i-1])/(float)(axis[i]-axis[i-1]);
            output = factor*(data[i]-data[i-1])+data[i-1];
            //Serial.println(factor);  
            return(output);
        }
    }
}


float saturate(float in, float LimLwr, float LimUpr)
{
    if(in >= LimUpr) return(LimUpr);
    if(in <= LimLwr) return(LimLwr);
    return(in);
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

#endif
