

#ifndef SIGNALHYSTERESIS_H
#define SIGNALHYSTERESIS_H

#include <Arduino.h>

/* The SignalHysteresis is watching a signal between two levels */
 
class SignalHysteresis 
{
    public:
        SignalHysteresis(int _State);
        boolean update(float _x_k , float _LvlLwr, float _LvlUpr);
        
    private:
        boolean State;
};

#endif


