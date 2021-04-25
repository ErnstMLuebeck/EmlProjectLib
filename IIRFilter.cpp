#include "IIRFilter.h"

/** Construct IIR filter object
 * 
 * @param _num numerator coefficient vector
 * @param _den denominator coefficient vector
 * @param _length number of elements in num and den (filter order)
 */
IIRFilter::IIRFilter(float _num[], float _den[], uint8_t _length)
{   
    order = _length;
    IdxWrite = 0;

    num = (float*)malloc(sizeof(float) * order);
    den = (float*)malloc(sizeof(float) * order);
    input_buffer = (float*)malloc(sizeof(float) * order);
    output_buffer = (float*)malloc(sizeof(float) * order);

    for(int i=0; i<order; i++)
    {   
        *(num+i) = _num[i];
        *(den+i) = _den[i];
        *(input_buffer+i) = 0;
        *(output_buffer+i) = 0;
    }
}

/** Print out filter coefficients and buffer content for debugging
 */
void IIRFilter::printFilterData()
{   
    Serial.println(".");
    for(int i=0; i<order; i++)
    {   Serial.print("num[");
        Serial.print(i);
        Serial.print("] = ");
        Serial.print(*(num+i),6);
        Serial.print(", ");

        Serial.print("den[");
        Serial.print(i);
        Serial.print("] = ");
        Serial.print(*(den+i),6);
        Serial.print(", ");

        Serial.print("input_buffer[");
        Serial.print(i);
        Serial.print("] = ");
        Serial.print(*(input_buffer+i),6);
        Serial.print(", ");

        Serial.print("output_buffer[");
        Serial.print(i);
        Serial.print("] = ");
        Serial.print(*(output_buffer+i),6);
        Serial.print(", ");
        Serial.println();
    }
}

/** Calculate IIR filter algorithm
 * 
 * @param x [-], input signal
 * @return y [-], hysteresis state
 */
float IIRFilter::calculate(float x)
{   
    /* input buffer holds the order-last input values,
       output_buffer holds the order-last output values,
       order specifies the number of coefficients which are stored in
       the global variables num and den. The output buffer gets updated
       and the out value is returned */

    /* Todo: implement ringbuffer */
    input_buffer[IdxWrite] = x;

    if(IdxWrite < (order-1)) IdxWrite++;
    else IdxWrite = 0;
    
    float out = 0.0;
    
    for(int i = 0; i<order; i++)
    {
        out += num[i] * input_buffer[i];
        out += den[i] * output_buffer[i];
    }
    

    output_buffer[1] = output_buffer[0];
    output_buffer[0] = out;
    
    return(out);
  
}







