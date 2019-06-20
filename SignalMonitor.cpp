#include "SignalMonitor.h"


SignalMonitor::SignalMonitor(float _x_kn1)
{
    x_kn1 = _x_kn1;
}


/* returns 1 if signal increased */
boolean SignalMonitor::detectIncrease(float _x_k)
{
    boolean result = 0;
    if(_x_k > x_kn1) result = 1;

    x_kn1 = _x_k;
    return(result);
}

/* returns 1 if signal decreased */
boolean SignalMonitor::detectDecrease(float _x_k)
{
    boolean result = 0;
    if(_x_k < x_kn1) result = 1;

    x_kn1 = _x_k;
    return(result);
}

/* returns 1 if signal has is bigger and
   -1 if signal is smaller */
int SignalMonitor::detectChange(float _x_k)
{
    int result = 0;
    if(_x_k > x_kn1) result = 1;
    if(_x_k < x_kn1) result = -1;

    x_kn1 = _x_k;
    return(result);
}








