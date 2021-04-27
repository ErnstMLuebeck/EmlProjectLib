#ifndef IIRFILTERBIQUAD_H
#define IIRFILTERBIQUAD_H

#include <Arduino.h>

/**
 * 2 Stage, Biquad IIR Filter (Bessel, Butterworth, Custom)
 * 
 * Source: https://www.micromodeler.com/dsp/
 * 
 * @author E. M. Luebeck
 * @date 2021-04-27
 */ 
class IIRFilterBiquad 
{
    public:
        IIRFilterBiquad(int _ModeFilter);
        void setCoeff(float fc, float fs);
        float calculate(float x_k);
        
    private:
        /** Filter mode: 0=custom coef., 1 = butterworth fc=0.1 */
        int ModeFilter;

        float x_kn1[2] = {{0}};
        float x_kn2[2] = {{0}};
        float y_kn2[2] = {{0}};
        float y_kn1[2] = {{0}};
        float* coeff;

        float butterworth_fc0p1[10] = 
        {
            // Scaled for floating point
            0.07718949372345962, 0.15437898744691925, 0.07718949372345962, 1.0485995763626117, -0.2961403575616696,// b0, b1, b2, a1, a2
            0.0625, 0.125, 0.0625, 1.3209134308194261, -0.6327387928852763// b0, b1, b2, a1, a2
        };

        float butterworth_fc0p25[10] = 
        {
            // Scaled for floating point
            0.3759234057351777, 0.7518468114703554, 0.3759234057351777, 1.1102230246251568e-16, -0.03956612989658006,// b0, b1, b2, a1, a2
            0.25, 0.5, 0.25, 2.2204460492503136e-16, -0.44646269217168966 // b0, b1, b2, a1, a2

        };

        float butterworth_fc0p4[10] = 
        {
            // Scaled for floating point
            0.8656932899805831, 1.7313865799611663, 0.8656932899805831, -1.0485995763626115, -0.2961403575616695,// b0, b1, b2, a1, a2
            0.5, 1, 0.5, -1.320913430819426, -0.6327387928852763 // b0, b1, b2, a1, a2

        };

        float bessel_fc0p1[10] = 
        {
            // Scaled for floating point
            0.06859872468666957, 0.13719744937333914, 0.06859872468666957, 1.0770123857826925, -0.3009430418813964,// b0, b1, b2, a1, a2
            0.0625, 0.125, 0.0625, 1.1409612555521207, -0.44730039589935716 // b0, b1, b2, a1, a2

        };

        float bessel_fc0p25[10] = 
        {
            // Scaled for floating point
            0.31455007673059665, 0.6291001534611933, 0.31455007673059665, 0.058362540165184475, -0.022279556690303206,// b0, b1, b2, a1, a2
            0.25, 0.5, 0.25, -0.07049567021299492, -0.23480373352421335// b0, b1, b2, a1, a2

        };
};

#endif /* IIRFILTERBIQUAD_H */


