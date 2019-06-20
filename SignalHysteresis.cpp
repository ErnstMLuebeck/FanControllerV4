#include "SignalHysteresis.h"


SignalHysteresis::SignalHysteresis(int _State)
{
    State = _State;
}

/* returns 1 if signal is above the upper level
   0 if the signal is below the lower level */
boolean SignalHysteresis::update(float _x_k, float _LvlLwr, float _LvlUpr)
{
    if(State == 0 && _x_k > _LvlUpr) State = 1;
    if(State == 1 && _x_k < _LvlLwr) State = 0;

    return(State);
}








