#ifndef IIRFILTER_H
#define IIRFILTER_H

#include <Arduino.h>

/**
 * IIR Filter: NOT FINISHED!!
 * 
 * @author E. M. Luebeck
 * @date 2021-04-23
 */ 
class IIRFilter 
{
    public:
        IIRFilter();
        float calculate(float x);
        
    private:
        int order = 2;
        float input_buffer[2];
        float output_buffer[2];
        float num[2] = {0.052004382885834, 0.008913673017008};
        float den[2] = {1.000000000000000, -0.939081944097158};
};

#endif /* IIRFILTER_H */


