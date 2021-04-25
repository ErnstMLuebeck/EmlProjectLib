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
        IIRFilter(float _num[], float _den[], uint8_t _length);
        void printFilterData();
        float calculate(float x);
        
    private:
        int order;
        float* input_buffer;
        float* output_buffer;
        int IdxWrite;
        float* num;
        float* den;

        //float num[2] = {0.052004382885834, 0.008913673017008};
        //float den[2] = {1.000000000000000, -0.939081944097158};
};

#endif /* IIRFILTER_H */


