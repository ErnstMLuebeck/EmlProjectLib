#include "PinMonitor.h"

/**
 * @brief Construct pin monitor object
 *  
 * @param _numPin [-], pin number
 * @param _numDeBnceCycles [-], number of debounce cycles
 * @param _stActv [-], pin state HIGH = active, or LOW = active
 * @param _pullUpResistor [-], use input pull-up resistor or not
 */
PinMonitor::PinMonitor(int _numPin, uint16_t _numDeBnceCycles, boolean _stActv, boolean _pullUpResistor)
{
    numPin = _numPin;
    numDeBnceCycles = _numDeBnceCycles;
    stActv = _stActv;

    cntrDeBnce = 0;
    y_0 = 0;
    y_n1 = 0;

    flgRisngEdgePndng = 0;
    flgFallngEdgePndng = 0;
    flgEdgePndng = 0;

    TiRisngEdge = 0;
    TiFallngEdge = 0;

    TiHighLim = 0;
    TiLowLim = 0;

    flgIgnoreNxtRisngEdge = 0;
    flgIgnoreNxtFallngEdge = 0;

    if(_pullUpResistor)
    {
        pinMode(numPin, INPUT_PULLUP);
    }
    else
    {
        pinMode(numPin, INPUT);
    }
}

/**
 * @brief Set the time limit to detect "long" states 
 * 
 * @param _TiHighLim [ms], time limit to detect long HIGH states
 */
void PinMonitor::setTiHighLim(unsigned long _TiHighLim)
{
    TiHighLim = _TiHighLim;
}

/**
 * @brief Set the time limit to detect "long" LOW states 
 * 
 * @param _TiLowLim [ms], time limit to detect long LOW states
 */
void PinMonitor::setTiLowLim(unsigned long _TiLowLim)
{
    TiLowLim = _TiLowLim;
}

/**
 * @brief Set number of debounce cycles
 * 
 * @param _numDeBnceCycles [-], number of debounce cycles
 */
void PinMonitor::setNumDeBnceCyckes(uint16_t _numDeBnceCycles)
{
    numDeBnceCycles = _numDeBnceCycles;
}

/**
 * @brief Update pin monitor states.
 * 
 * This function needs to be called regularly. The pin is sampled and a debounce counter is 
 * incremented/decremented according to the pin state. If the debounce threshold is reached, 
 * the state is set. All kind of edges are detected. There is an inhibit flag to skip the next edge.
 * This is used after a long press, so the button release does not trigger an additional action. 
 */
void PinMonitor::update()
{
    /* update counter */
    if(digitalRead(numPin) == stActv)
    {   if(cntrDeBnce < numDeBnceCycles) cntrDeBnce++;
    }
    else
    {   if(cntrDeBnce > 0) cntrDeBnce--;
    }

    /* detect rising edge */
    if((cntrDeBnce >= numDeBnceCycles) && (y_n1 == 0))
    {   
        if(!flgIgnoreNxtRisngEdge) flgRisngEdgePndng = 1;
        else flgIgnoreNxtRisngEdge = 0;
        
        flgEdgePndng = 1;
        y_0 = 1;

        // save time of rising edge
        TiRisngEdge = millis();
        TiFallngEdge = 0;
    }

    /* detect falling edge */
    if((cntrDeBnce <= 0) && (y_n1 == 1))
    {   
        if(!flgIgnoreNxtFallngEdge) flgFallngEdgePndng = 1;
        else flgIgnoreNxtFallngEdge = 0;
        
        flgEdgePndng = 1;
        y_0 = 0;
     
        /* save time of falling edge */
        TiFallngEdge = millis();
        TiRisngEdge = 0;
    }
    
    y_n1 = y_0;

}

/**
 * @brief Check if a rising edge occured.
 * 
 * @return Flag [-], rising edge occured
 */
boolean PinMonitor::risingEdge()
{
    if(flgRisngEdgePndng)
    {   flgRisngEdgePndng = 0;
        return(1);
    }
    else return(0);
}

/**
 * @brief Check if a falling edge occured.
 * 
 * @return Flag [-], falling edge occured
 */
boolean PinMonitor::fallingEdge()
{
    if(flgFallngEdgePndng)
    {   flgFallngEdgePndng = 0;
        return(1);
    }
    else return(0);
}

/**
 * @brief Check if any edge occured.
 * 
 * @return Flag [-], any edge occured
 */
boolean PinMonitor::anyEdge()
{
    if(flgEdgePndng)
    {   flgEdgePndng = 0;
        return(1);
    }
    else return(0);
}

/**
 * @brief Get the current state of the pin
 * 
 * @return Flag [-], pin state
 */
boolean PinMonitor::getState()
{
    return(y_0);
}

/**
 * @brief Get the time the pin spent being high
 * 
 * @return TiPinHigh [ms], time pin was HIGH
 */
unsigned long PinMonitor::getHighTime()
{
    if(y_0 == 1) return(millis()-TiRisngEdge);
    else return(0);
}

/**
 * @brief Get the time the pin spent being LOW
 * 
 * @return TiPinHigh [ms], time pin was LOW
 */
unsigned long PinMonitor::getLowTime()
{
    if(y_0 == 0) return(millis()-TiFallngEdge);
    else return(0);
}

/**
 * @brief Check if pin was HIGH longer than the TiHighLim time
 * 
 * @return Flag [-], pin was HIGH for a long time
 */
boolean PinMonitor::highLong()
{
    if((TiRisngEdge != 0) && (millis()-TiRisngEdge) >= TiHighLim) 
    {   TiRisngEdge = millis(); 
        return(1);
    }
    else return(0);
}

/**
 * @brief Check if pin was LOW longer than the TiLowLim time
 * 
 * @return Flag [-], pin was LOW for a long time
 */
boolean PinMonitor::lowLong()
{
    if((TiFallngEdge != 0) && (millis()-TiFallngEdge) >= TiLowLim) 
    {   TiFallngEdge = 0; 
        return(1);
    }
    else return(0);
}

/**
 * @brief Set the inhibit flag to ignore the next rising edge
 */
void PinMonitor::ignoreNxtRisngEdge()
{
    flgIgnoreNxtRisngEdge = 1;
}

/**
 * @brief Set the inhibit flag to ignore the next falling edge
 */
void PinMonitor::ignoreNxtFallngEdge()
{
    flgIgnoreNxtFallngEdge = 1;
}
















