

#ifndef SIGNALMONITOR_H
#define SIGNALMONITOR_H

#include <Arduino.h>

/* The SignalMonitor is watching a signal and detects changes */
 
class SignalMonitor 
{
    public:
        SignalMonitor(float _x_kn1);
        boolean detectIncrease(float _x_k);
        boolean detectDecrease(float _x_k);
        int detectChange(float _x_k);
        
    private:
        float x_kn1;
};

#endif


