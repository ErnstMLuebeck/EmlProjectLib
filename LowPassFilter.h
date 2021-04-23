

#ifndef LOWPASSFILTER_H
#define LOWPASSFILTER_H

class LowPassFilter 
{
    public:
        LowPassFilter(float _Ts, float _Tc, float _y_n1) ;
        float calculate(float);
        void setValue(float _y_n1);

    private:
        float y_n1;  // y[k-1]
        float Ts;   // sample time
        float Tc;   // time constant
        float alpha;  // filter constant
};

#endif


