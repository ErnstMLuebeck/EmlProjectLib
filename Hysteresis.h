#ifndef HYSTERESIS_H
#define HYSTERESIS_H

#include <Arduino.h>

/**
 * @brief Standard hysteresis with left and right thresholds
 * 
 * @author E. M. Luebeck
 * @date 2021-04-23
 */ 
class Hysteresis 
{
    public:
        Hysteresis(float _ThrLeft, float _ThrRight, boolean _y_kn1);
        boolean update(float x);
        void setThresholds(float _ThrLeft, float _ThrRight);
        void setState(boolean _y_kn1);
        
    private:
        /** [-], hysteresis state */
        boolean y_kn1;
        /** [-], left threshold (turn off) */
        float ThrLeft;
        /** [-], right threshold (turn on) */
        float ThrRight;
};

#endif /* HYSTERESIS_H */


