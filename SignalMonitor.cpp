#include "SignalMonitor.h"

/**
 * Construct the signal monitor object
 */
SignalMonitor::SignalMonitor(float _x_kn1)
{
    x_kn1 = _x_kn1;
}

/**
 * @brief Detect increase
 * 
 * @return 1 if input signal has increased 
 */
boolean SignalMonitor::detectIncrease(float _x_k)
{
    boolean result = 0;
    if(_x_k > x_kn1) result = 1;

    x_kn1 = _x_k;
    return(result);
}

/**
 * @brief Detect decrease
 * 
 * @return 1 if input signal has decreased 
 */
boolean SignalMonitor::detectDecrease(float _x_k)
{
    boolean result = 0;
    if(_x_k < x_kn1) result = 1;

    x_kn1 = _x_k;
    return(result);
}

/**
 * @brief Detect change
 * 
 * @return 1 if input signal has changed 
 */
boolean SignalMonitor::detectChange(float _x_k)
{
    boolean result = 0;
    if(_x_k != x_kn1) result = 1;

    x_kn1 = _x_k;
    return(result);
}