

#ifndef TURNONOFFDELAY_H
#define TURNONOFFDELAY_H

#include <Arduino.h> 

class TurnOnOffDelay 
{
    public:
        TurnOnOffDelay(unsigned long _TiOnDly, unsigned long _TiOffDly);
        void setTiDly(unsigned long _TiOnDly, unsigned long _TiOffDly);
        uint8_t update(uint8_t x);

    private:
        uint16_t x_kn1, y; /* states */
        unsigned long TiOnDly; /* [ms], Turn on delay time */
        unsigned long TiOffDly; /* [ms], Turn off delay time */
        unsigned long TiRisngEdge;  /* [ms], absolute time of last rising edge */
        unsigned long TiFallngEdge; /* [ms], absolute time of last falling edge */
};

#endif


