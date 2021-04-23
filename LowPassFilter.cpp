

#include "LowPassFilter.h"


// Class Constructor
LowPassFilter::LowPassFilter(float _Ts, float _Tc, float _y_n1) 
{
    Ts = _Ts;
    Tc = _Tc;
    alpha = 1-Ts/Tc;
    if(alpha > 1.0) alpha = 1.0;
    if(alpha < 0.0) alpha = 0.0;
    y_n1 = _y_n1;

}

float LowPassFilter::calculate(float _x_0) 
{
    y_n1 = alpha * y_n1 + (1-alpha) * _x_0;
    return(y_n1);
}

void LowPassFilter::setValue(float _y_n1)
{   y_n1 = _y_n1;
}



