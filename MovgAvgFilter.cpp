#include "MovgAvgFilter.h"

/** 
 * @brief Constructor MA filter object
 * 
 * Note that the maximum buffer length is 255
 */
MovgAvgFilter::MovgAvgFilter(uint8_t _Len)
{    
    Len = _Len;

    buffer = (float*)calloc(sizeof(float), Len);
    
    alpha = 1/(float)Len;
    
    pointer = 0;
}

/**
 * @brief Calculates filter output based on new input
 * 
 * The input is stored in a ring-buffer. The output is the sum of
 * all buffered input samples, divided by the number of samples in the buffer.
 * 
 * @param x_k input signal
 * @return y output signal
 */
float MovgAvgFilter::calculate(float x_k) 
{
    float y = 0;
    
    /* write new value at pointer position */
    buffer[pointer] = x_k;
    
    /* increment pointer to oldest value */
    pointer++;
    if(pointer >= Len) pointer = 0;
    
    /* calculate the sum of the buffer */
    for(int i=0; i<Len; i++)
    {   y += buffer[i];
    }
    
    /* apply weighting */
    y *= alpha;
    
    return(y);
}



