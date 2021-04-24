#include "LowPassFilter.h"

/**
 * Construct filter object with parameters and initial values. The filter coefficient is calculated by:
 * 
 * \f$ \alpha = 1 - \frac{Ts}{Tc} \f$
 * 
 * The coefficient is then limited between 0 and 1 in case the user input is incorrect.
 *  
 * @param _Ts [s], sample time
 * @param _Tc [s], filter time constant
 * @param _y_kn1 [-], initial value of filter state y[k-1]
 */
LowPassFilter::LowPassFilter(float _Ts, float _Tc, float _y_kn1) 
{
    Ts = _Ts;
    Tc = _Tc;
    alpha = 1-Ts/Tc;
    if(alpha > 1.0) alpha = 1.0;
    if(alpha < 0.0) alpha = 0.0;
    y_kn1 = _y_kn1;
}

/**
 * Calculate filter output according to the following difference equation:
 * 
 * \f$ y[k] = \alpha \cdot y[k-1] + (1-\alpha) \cdot x[k] \f$
 *  
 * @param _x [-], filter input
 * @return _y [-], filter output
 */
float LowPassFilter::calculate(float _x) 
{
    y_kn1 = alpha * y_kn1 + (1-alpha) * _x;
    return(y_kn1);
}

/**
 * Set filter state, y[k-1].
 *  
 * @param _y_kn1 [-], new filter sate
 */
void LowPassFilter::setValue(float _y_kn1)
{   
    y_kn1 = _y_kn1;
}

/**
 * The filter coefficient is updated by:
 * 
 * \f$ \alpha = 1 - \frac{Ts}{Tc} \f$
 * 
 * The coefficient is then limited between 0 and 1 in case the user input is incorrect.
 *  
 * @param _Tc [s], filter time constant
 */
void LowPassFilter::setTc(float _Tc) 
{
    Tc = _Tc;
    alpha = 1-Ts/Tc;
    if(alpha > 1.0) alpha = 1.0;
    if(alpha < 0.0) alpha = 0.0;

}



