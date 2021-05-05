#ifndef SIGNALMONITOR_H
#define SIGNALMONITOR_H

#include <Arduino.h>

/** 
 * @brief Monitoring of a software signal to detect increase, decrease or change. 
 * 
 * The SignalMonitor is watching an integer software-signal and detects changes
 * 
 * @author E. M. Luebeck
 * @date 2021-04-23
 */ 
class SignalMonitor 
{
    public:
        SignalMonitor(float _x_kn1);
        boolean detectIncrease(float _x_k);
        boolean detectDecrease(float _x_k);
        boolean detectChange(float _x_k);
        
    private:
        float x_kn1;
};

#endif /* SIGNALMONITOR_H */


