#include "PidCtrlr.h"

PidCtrlr::PidCtrlr(float _Kp, float _Ki, float _Kd, float _Kaw, float _TcDpart, float _Ts)
{
    Ts = _Ts;
    Kp = _Kp;
    Ki = _Ki;
    Kd = _Kd;
    Kaw = _Kaw;
    TcDpart = _TcDpart;

    /* Calculate filter coefficient from time constant */
    CoeffFiltDpart = 1 - Ts/TcDpart;
    if(CoeffFiltDpart > 1.0) CoeffFiltDpart = 1.0;
    if(CoeffFiltDpart < 0.0) CoeffFiltDpart = 0.0;

    /* Initialize states with zero */
    error_kn1 = 0;
    derivative_kn1 = 0;
    integral = 0;
}

void PidCtrlr::setTs(float _Ts)
{   /* Update sample time (recommended to keep it fixed) */
    Ts = _Ts;
}

void PidCtrlr::setCtrlrGains(float _Kp, float _Ki, float _Kd, float _Kaw)
{   /* Update the controller gains during run-time in case gain-scheduling
    is used */
    Kp = _Kp;
    Ki = _Ki;
    Kd = _Kd;
    Kaw = _Kaw;
}

void PidCtrlr::setIpart(float _IpartInit)
{   /* Initialize integrator with defined value */
    integral = _IpartInit;
}

float PidCtrlr::calculate(float y_sp, float y, float u_true, float u_ff)
{
    /* Calculate PID control algorithm 
        
        inputs:
        y_sp = Plant output setpoint
        y = Actual/Measured plant output
        u_true = Implemented actuator position after saturation
        u_ff = feed-forward actuator position, keep 0 if no FF control is used
        outputs:
        u = Unlimited actuator position

        Note that the actuator is typically limited to physical limits (e.g. 0..100%).
        If not, u_true can simply be set to u_kn1, the last controller output.

    */

    float error_k, integral, derivative;
    float up, ui, ud;
    
    error_k = y_sp - y;
    integral = integral + (error_k * Ts) + (Kaw * (u_true - u_kn1));
    
    /* Todo: implement filtering of D-part */
    derivative = (error_k - error_kn1) / Ts;
    error_kn1 = error_k;

    /* Filter derivative part with first order low-pass */
    derivative = CoeffFiltDpart * derivative_kn1 + (1-CoeffFiltDpart) * derivative;
    derivative_kn1 = derivative;
    
    up = error_k * Kp;
    ui = integral * Ki;
    ud = derivative * Kd;
    
    u_kn1 = up + ui + ud + u_ff;

    return(u_kn1);
}



