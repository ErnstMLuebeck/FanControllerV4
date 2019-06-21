#include "SignalMonitor.h"


SignalMonitor::SignalMonitor(float _x_kn1)
{   /* Set initial state */
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

/* returns 1 if signal is bigger and
   -1 if signal is smaller than before */
int SignalMonitor::detectChange(float _x_k)
{
    int result = 0;
    if(_x_k > x_kn1) result = 1;
    if(_x_k < x_kn1) result = -1;

    x_kn1 = _x_k;
    return(result);
}

/*----------------------------------------------------------------------------------------*/

SignalHysteresis::SignalHysteresis(boolean _y_kn1)
{   /* Set initial state */
    y_kn1 = _y_kn1;
}

/* returns 1 if signal is above the upper level
   0 if the signal is below the lower level */
boolean SignalHysteresis::update(float _x_k, float _LvlLwr, float _LvlUpr)
{
    if(y_kn1 == 0 && _x_k > _LvlUpr) y_kn1 = 1;
    if(y_kn1 == 1 && _x_k < _LvlLwr) y_kn1 = 0;

    return(y_kn1);
}

/*----------------------------------------------------------------------------------------*/

TurnOnOffDelay::TurnOnOffDelay(boolean _x_kn1)
{
    x_kn1 = _x_kn1;
    NumOnCntr = 0;
    StCntrAcv = 0;
}

/* delays rising edge for Td seconds */
boolean TurnOnOffDelay::updateTurnOnDelay(boolean _x_k, float Td, float Ts)
{
    /* Falling edge */
    if(x_kn1 == 1 && _x_k == 0)
    {   StCntrAcv = 0;
        NumOffCntr = 0;
        x_kn1 = _x_k;
        return(0);
    } 

    if(x_kn1 == 0 && _x_k == 1) 
    {   StCntrAcv = 1;
    }
    
    /* State remains */
    x_kn1 = _x_k;

    if(StCntrAcv == 1) NumOnCntr++;
    else NumOnCntr = 0;

    if((NumOnCntr*Ts) >= Td && StCntrAcv) return(1);
    else return(0);
}

/* delays falling edge for Td seconds */
boolean TurnOnOffDelay::updateTurnOffDelay(boolean _x_k, float Td, float Ts)
{
    /* Rising edge */
    if(x_kn1 == 0 && _x_k == 1) 
    {   StCntrAcv = 0;
        NumOffCntr = 0;
        x_kn1 = _x_k;
        return(1);
    }

    /* Falling edge */
    if(x_kn1 == 1 && _x_k == 0) 
    {   StCntrAcv = 1;
    }

    /* State remains at zero */
    x_kn1 = _x_k;
    
    if(StCntrAcv == 1) NumOffCntr++;
    else NumOffCntr = 0;

    if((NumOffCntr*Ts) >= Td && StCntrAcv) return(0);
    else return(1);
}

/*----------------------------------------------------------------------------------------*/

SigDebounce::SigDebounce(uint16_t _numDeBnceCycles, boolean _stActv)
{
    numDeBnceCycles = _numDeBnceCycles;
    stActv = _stActv;

    cntrDeBnce = 0;
    y_k = 0;
    y_kn1 = 0;

    flgRisngEdgePndng = 0;
    flgFallngEdgePndng = 0;
    flgEdgePndng = 0;

    TiRisngEdge = 0;
    TiFallngEdge = 0;

    TiHighLim = 0;
    TiLowLim = 0;

    flgIgnoreNxtRisngEdge = 0;
    flgIgnoreNxtFallngEdge = 0;

}

/* sets the time limit to detect "long" states [ms] */
void SigDebounce::setTiHighLim(unsigned long _TiHighLim)
{   TiHighLim = _TiHighLim;
}

/* sets the time limit to detect "long" states [ms] */
void SigDebounce::setTiLowLim(unsigned long _TiLowLim)
{   TiLowLim = _TiLowLim;
}

/* This function needs to be called regularly (e.g. in an interrupt routine)
 * The pin is sampled and a debounce counter is incremented/decremented according
 * to the pin state. If the debounce threshold is reached, the state is set.
 * All kind of edges are detected. There is an inhibit flag to skip the next edge.
 * This is used after a long press, so the button release does not trigger an
 * additional action. 
 */
void SigDebounce::update(boolean _x_k)
{
    /* update counter */
    if(_x_k == stActv)
    {   if(cntrDeBnce < numDeBnceCycles) cntrDeBnce++;
    }
    else
    {   if(cntrDeBnce > 0) cntrDeBnce--;
    }

    /* detect rising edge */
    if((cntrDeBnce >= numDeBnceCycles) && (y_kn1 == 0))
    {   
        if(!flgIgnoreNxtRisngEdge) flgRisngEdgePndng = 1;
        else flgIgnoreNxtRisngEdge = 0;
        
        flgEdgePndng = 1;
        y_k = 1;

        /* save time of rising edge */
        TiRisngEdge = millis();
        TiFallngEdge = 0;
    }

    /* detect falling edge */
    if((cntrDeBnce <= 0) && (y_kn1 == 1))
    {   
        if(!flgIgnoreNxtFallngEdge) flgFallngEdgePndng = 1;
        else flgIgnoreNxtFallngEdge = 0;
        
        flgEdgePndng = 1;
        y_k = 0;
     
        /* save time of rising edge */
        TiFallngEdge = millis();
        TiRisngEdge = 0;
    }
    
    y_kn1 = y_k;

}

/* returns high if a rising edge occured */
boolean SigDebounce::risingEdge()
{
    if(flgRisngEdgePndng)
    {   flgRisngEdgePndng = 0;
        flgEdgePndng = 0;
        
        if(!flgIgnoreNxtRisngEdge)
        {   return(1);
        }
        else
        {   return(0);
        }
    }
    else return(0);
}

/* returns high if a falling edge occured */
boolean SigDebounce::fallingEdge()
{
    if(flgFallngEdgePndng)
    {   flgFallngEdgePndng = 0;
        flgEdgePndng = 0;
        
        if(!flgIgnoreNxtFallngEdge)
        {   return(1);
        }
        else
        {   return(0);
        }
    }
    else return(0);
}

/* returns high if any edge occured */
boolean SigDebounce::anyEdge()
{
    if(flgEdgePndng)
    {   flgEdgePndng = 0;
        return(1);
    }
    else return(0);
}

/* returns the current, debounced state */
boolean SigDebounce::getState()
{
    return(y_k);
}

/* returns the time the pin spent being high [ms] */
unsigned long SigDebounce::getHighTime()
{
    if(y_k == 1) return(millis()-TiRisngEdge);
    else return(0);
}

/* returns the time the pin spent being low [ms] */
unsigned long SigDebounce::getLowTime()
{
    if(y_k == 0) return(millis()-TiFallngEdge);
    else return(0);
}

/* returns high if the high time is longer than the high threshold */
boolean SigDebounce::highLong()
{
    if((TiRisngEdge != 0) && (millis()-TiRisngEdge) >= TiHighLim) 
    {   TiRisngEdge = 0; 
        return(1);
    }
    else return(0);
}

/* returns high if the low time is longer than the high threshold */
boolean SigDebounce::lowLong()
{
    if((TiFallngEdge != 0) && (millis()-TiFallngEdge) >= TiLowLim) 
    {   TiFallngEdge = 0; 
        return(1);
    }
    else return(0);
}

/* sets the inhibit flag to ignore the next rising edge */
void SigDebounce::ignoreNxtRisngEdge()
{
    flgIgnoreNxtRisngEdge = 1;
}

/* sets the inhibit flag to ignore the next falling edge */
void SigDebounce::ignoreNxtFallngEdge()
{
    flgIgnoreNxtFallngEdge = 1;
}





