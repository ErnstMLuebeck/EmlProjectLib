#include "IIRFilter.h"

/** Construct hysteresis object
 * 
 * @param _ThrLeft [-], left threshold (turn off)
 * @param _ThrRight [-], right threshold (turn on)
 * @param _y_kn1 [-], state of hysteresis block y[k-1]
 */
IIRFilter::IIRFilter(float _ThrLeft, float _ThrRight, boolean _y_kn1)
{

}

/** Update hysteresis block
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







