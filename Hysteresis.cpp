#include "Hysteresis.h"

/** Construct hysteresis object
 * 
 * @param _ThrLeft [-], left threshold (turn off)
 * @param _ThrRight [-], right threshold (turn on)
 * @param _y_kn1 [-], state of hysteresis block y[k-1]
 */
Hysteresis::Hysteresis(float _ThrLeft, float _ThrRight, boolean _y_kn1)
{
    ThrLeft = _ThrLeft;
    ThrRight = _ThrRight;
    y_kn1 = _y_kn1;
}

/** Update hysteresis block
 * 
 * @param x [-], input signal
 * @return y [-], hysteresis state
 */
boolean Hysteresis::update(float x)
{
    if((y_kn1 == true) && (x < ThrLeft))
    {
        y_kn1 = false;
    }
    if((y_kn1 == false) && (x > ThrRight))
    {
        y_kn1 = true;
    }

    return(y_kn1);
}

void Hysteresis::setThresholds(float _ThrLeft, float _ThrRight)
{
    ThrLeft = _ThrLeft;
    ThrRight = _ThrRight;

}

void Hysteresis::setState(boolean _y_kn1)
{
    y_kn1 = _y_kn1;
}









