#ifndef SIGNALMONITOR_H
#define SIGNALMONITOR_H

#include <Arduino.h>

/* The SignalMonitor is watching an integer software-signal and detects changes
 * 
 * @author E. M. Luebeck
 * @date 2021-04-23
 */ 
class SignalMonitor 
{
    public:
        SignalMonitor(int _x_kn1);
        boolean detectIncrease(int _x_k);
        boolean detectDecrease(int _x_k);
        boolean detectChange(int _x_k);
        
    private:
        int x_kn1;
};

#endif /* SIGNALMONITOR_H */


