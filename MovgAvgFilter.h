

#ifndef MOVGAVGFILTER_H
#define MOVGAVGFILTER_H

#include <Arduino.h>

class MovgAvgFilter
{
    public:
        MovgAvgFilter();
        float calculate(float);

    private:
        uint8_t Len;
        float buffer[60];
        float alpha;
        uint8_t pointer;

};

#endif


