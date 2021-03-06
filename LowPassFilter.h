#ifndef LOWPASSFILTER_H
#define LOWPASSFILTER_H

#include <Arduino.h>

/**
 * @brief Standard first order low-pass filter with time constant input
 * 
 * @author E. M. Luebeck
 * @date 2021-04-23
 */
class LowPassFilter 
{
    public:
        LowPassFilter(float _Ts, float _Tc, float _y_kn1);
        float calculate(float _x);
        void setValue(float _y_kn1);
        void setTc(float _Tc);
        void setTs(float _Ts);

    private:     
        /** previous state y[k-1] */
        float y_kn1;
        /** [s], sample time */
        float Ts;   
        /** [s], time constant */
        float Tc;   
        /** [-], filter constant */
        float alpha; 
};

#endif /* LOWPASSFILTER_H */


