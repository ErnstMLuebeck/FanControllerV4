

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

/*----------------------------------------------------------------------------------------*/

/* The SignalHysteresis is watching a signal between two levels */
 class SignalHysteresis 
{
    public:
        SignalHysteresis(boolean y_kn1);
        boolean update(float _x_k , float _LvlLwr, float _LvlUpr);
        
    private:
        boolean y_kn1;
};

/*----------------------------------------------------------------------------------------*/

/* Turn on and off delay of a boolean signal */
class TurnOnOffDelay 
{
    public:
        TurnOnOffDelay(boolean _x_kn1);
        boolean updateTurnOnDelay(boolean _x_k, float Td, float Ts);
        boolean updateTurnOffDelay(boolean _x_k, float Td, float Ts);
        
    private:
        float x_kn1;
        unsigned int NumOnCntr;
        unsigned int NumOffCntr;
        int StCntrAcv;
};

/*----------------------------------------------------------------------------------------*/

/* Signal debouncing */
class SigDebounce 
{
    public:
        SigDebounce(uint16_t _numDeBnceCycles, boolean _stActv);
        void update(boolean _x_k);
        boolean risingEdge();
        boolean fallingEdge();
        boolean anyEdge();
        boolean getState();
        unsigned long getHighTime();
        unsigned long getLowTime();
        void setTiHighLim(unsigned long _TiHighLim);
        void setTiLowLim(unsigned long _TiLowLim);
        boolean highLong();
        boolean lowLong();
        void ignoreNxtRisngEdge();
        void ignoreNxtFallngEdge();
        
    private:
        boolean stActv;         // which state is considered HIGH, 1 or 0
        uint16_t numDeBnceCycles;
        uint16_t cntrDeBnce;    // debounce counter
        boolean y_k, y_kn1;    // y[k], y[k-1]
        boolean flgRisngEdgePndng;
        boolean flgFallngEdgePndng;
        boolean flgEdgePndng;
        unsigned long TiRisngEdge;      // absolute time of last rising edge
        unsigned long TiFallngEdge;     // absolute time of last falling edge
        unsigned long TiHighLim;   // limit to detect long high time
        unsigned long TiLowLim;  // limit to detect long low time
        boolean flgIgnoreNxtRisngEdge;
        boolean flgIgnoreNxtFallngEdge;
        
};


#endif


