#include "IIRFilterBiquad.h"

/** Construct IIR filter object
 * 
 * @param _ModeFilter, filter mode
 */
IIRFilterBiquad::IIRFilterBiquad(int _ModeFilter)
{   
    ModeFilter = _ModeFilter;

    switch(ModeFilter)
    {
        /* Butterworth LPF, fc = 0.1*fs */
        case 1: 
            coeff = butterworth_fc0p1;
            break;

        /* Butterworth LPF, fc = 0.25*fs */
        case 2: 
            coeff = butterworth_fc0p25;
            break;

        /* Butterworth LPF, fc = 0.4*fs */
        case 3: 
            coeff = butterworth_fc0p4;
            break;

        /* Bessel LPF, fc = 0.1*fs */
        case 4: 
            coeff = bessel_fc0p1;
            break;

        /* Bessel LPF, fc = 0.25*fs */
        case 5: 
            coeff = bessel_fc0p25;
            break;
        
        /* Coefficients to be set manually */
        default: 
            coeff = bessel_fc0p25;
            break;

    }

}

/**
 * Calculate biquad coefficients (single stage) based on cut-off frequency
 * Source: https://www.cankosar.com/digitales-tiefpassfilter-lpf/
 * 
 * @param fc [Hz], cut-off frequency of LPF
 * @param fs [Hz], sample rate
 */
void IIRFilterBiquad::setCoeff(float fc, float fs)
{
    /* 2021-04-27 EML: Not really sure if correct.. gain, Q..?*/

    float A, w0, cosw0, sinw0, alpha, a0_inv;
    float gain, Q;

    gain = 5;
    Q = 0.1;
 
    A=pow(10,gain/40);
    w0=2*PI*(fc/fs);
    cosw0=cos(w0);
    sinw0=sin(w0);
    alpha=sinw0/(2*Q);
    
    /* Normalization */
    a0_inv = 1/(1+alpha);
    
    /* Coefficient calculation */
    float b0, b1, b2, a1, a2;
    b0=(float)((1-cosw0)*0.5*a0_inv);
    b1=(float)((1-cosw0)*a0_inv);
    b2=b0;
    a1=(float)((-2*cosw0)*a0_inv);
    a2=(float)((1-alpha)*a0_inv);

    *(coeff+0) = b0;
    *(coeff+1) = b1;
    *(coeff+2) = b2;
    *(coeff+3) = a1;
    *(coeff+4) = a2;

    /* Bypass second stage */
    *(coeff+5+0) = 1.0;
    *(coeff+5+1) = 0.0;
    *(coeff+5+2) = 0.0;
    *(coeff+5+3) = 0.0;
    *(coeff+5+4) = 0.0;

}


float IIRFilterBiquad::calculate(float x_k)
{
    float b0, b1, b2, a1, a2;
    int IdStage = 0;

    while(IdStage < 2)
    {
        b0 = *(coeff + IdStage*5 + 0);
        b1 = *(coeff + IdStage*5 + 1);
        b2 = *(coeff + IdStage*5 + 2);
        a1 = *(coeff + IdStage*5 + 3);
        a2 = *(coeff + IdStage*5 + 4);

        //Serial.println(x_k,4);

        float accumulator = 0;

        accumulator += x_k * b0;
        accumulator += x_kn1[IdStage] * b1;
        accumulator += x_kn2[IdStage] * b2;

        /* Shuffle left history buffer */
        x_kn2[IdStage] = x_kn1[IdStage];		
        x_kn1[IdStage] = x_k;

        accumulator += y_kn1[IdStage] * a1;
        accumulator += y_kn2[IdStage] * a2;
        
        /* Shuffle right history buffer */
        y_kn2[IdStage] = y_kn1[IdStage];		
        y_kn1[IdStage] = accumulator;

        x_k = y_kn1[IdStage];

        IdStage++;
    }

    return(x_k);
}







