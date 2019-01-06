

#include "MovgAvgFilter.h"


// Class Constructor
MovgAvgFilter::MovgAvgFilter()
{
    Len = 60; /* hardcoded! */
    
    for(int i=0; i<Len; i++)
    {   buffer[i] = 0;
    }
    
    alpha = 1/(float)Len;
    
    pointer = 0;
}

float MovgAvgFilter::calculate(float _x_0) 
{
    float y = 0;
    
    /* write new value at pointer position */
    buffer[pointer] = _x_0;
    
    /* increment pointer to oldest value */
    pointer++;
    if(pointer >= Len) pointer = 0;
    
    /* calculate the sum of the buffer */
    for(int i=0; i<Len; i++)
    {   y += buffer[i];
    }
    
    /* aplpy weighting */
    y *= alpha;
    
    return(y);
}



