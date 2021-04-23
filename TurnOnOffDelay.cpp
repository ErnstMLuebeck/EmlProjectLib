

#include "TurnOnOffDelay.h"


// Class Constructor
TurnOnOffDelay::TurnOnOffDelay(unsigned long _TiOnDly, unsigned long _TiOffDly)
{
    TiOnDly = _TiOnDly;
    TiOffDly = _TiOffDly;
    TiRisngEdge = 0;
    TiFallngEdge = 0;
    x_kn1 = 0;
    y = 0;
}

void TurnOnOffDelay::setTiDly(unsigned long _TiOnDly, unsigned long _TiOffDly)
{
    TiOnDly = _TiOnDly;
    TiOffDly = _TiOffDly;
}

uint8_t TurnOnOffDelay::update(uint8_t x)
{
    unsigned long TiEval = millis();

    /* Detect rising edge */
    if(x > x_kn1)
    {   TiRisngEdge = TiEval;
    }

    /* Detect falling edge */
    if(x < x_kn1)
    {   TiFallngEdge = TiEval;
    }

    x_kn1 = x;

    if(x && ((TiEval-TiRisngEdge) >= TiOnDly)) y = 1;

    if(!x && ((TiEval-TiFallngEdge) >= TiOffDly)) y = 0;

    /* else */
    return(y);
    
}



