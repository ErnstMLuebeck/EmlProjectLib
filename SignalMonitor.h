/* The SignalMonitor is watching an integer software-signal and detects changes
 * 
 * $Author: E. M. Luebeck$
 * $Date: 2021-04-23$
 */

#ifndef SIGNALMONITOR_H
#define SIGNALMONITOR_H

#include <Arduino.h>
 
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

#endif


