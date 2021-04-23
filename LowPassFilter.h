/**
 * First order low-pass filter
 * 
 * @author E. M. Luebeck
 * @date 2021-04-23
 */

#ifndef LOWPASSFILTER_H
#define LOWPASSFILTER_H

class LowPassFilter 
{
    public:
        LowPassFilter(float _Ts, float _Tc, float _y_kn1);
        float calculate(float);
        void setValue(float _y_kn1);

    private:
        float y_kn1;  // y[k-1]
        float Ts;   // sample time
        float Tc;   // time constant
        float alpha;  // filter constant
};

#endif


