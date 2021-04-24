#include "PidCtrlr.h"

/**
 * Construct object with controller parameters. 
 * 
 * @param _Kp [-], proportional controller gain
 * @param _Ki [-], integral controller gain
 * @param _Kd [-], derivative controller gain
 * @param _Kaw [-], anti windup gain
 * @param _TcDpart [s], filter time constant of derivative filter
 * @param _Ts [s], sample time
 */
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

/**
 * Update sample time (it is recommended to keep it fixed) 
 * 
 * @param _Ts [s], controller sample time
 */
void PidCtrlr::setTs(float _Ts)
{   
    Ts = _Ts;
}

/**
 * Update the controller gains during run-time in case gain-scheduling is used
 * 
 * @param _Kp [-], proportional controller gain
 * @param _Ki [-], integral controller gain
 * @param _Kd [-], derivative controller gain
 * @param _Kaw [-], anti windup gain
 */
void PidCtrlr::setCtrlrGains(float _Kp, float _Ki, float _Kd, float _Kaw)
{   
    Kp = _Kp;
    Ki = _Ki;
    Kd = _Kd;
    Kaw = _Kaw;
}

/**
 * Update filter constant of derivative part low-pass filter
 * 
 * @param _TcDpart [s], filter time constant of derivative filter
 */
void PidCtrlr::setTcDpart(float _TcDpart)
{   
    TcDpart = _TcDpart;

    /* Calculate filter coefficient from time constant */
    CoeffFiltDpart = 1 - Ts/_TcDpart;
    if(CoeffFiltDpart > 1.0) CoeffFiltDpart = 1.0;
    if(CoeffFiltDpart < 0.0) CoeffFiltDpart = 0.0;
}

/**
 * Initialize integrator with defined value
 * 
 * @param _IpartInit [-], reset value for integrator
 */
void PidCtrlr::setIpart(float _IpartInit)
{   
    integral = _IpartInit;
}

/**
 * Calculate PID control algorithm.
 * Note that the actuator is typically limited to physical limits (e.g. 0..100%).
 * If not, u_true can simply be set to u_kn1, the last controller output.
 * If feed-foorward is not used, set it to 0.
 * 
 * @param y_sp plant output setpoint
 * @param y actual/measured plant output
 * @param u_true implemented actuator position after saturation
 * @param u_ff feed-forward actuator position, keep 0 if no FF control is used
 * @return u unlimited actuator position
 */
float PidCtrlr::calculate(float y_sp, float y, float u_true, float u_ff)
{
    float error_k, derivative;
    float up, ui, ud;
    
    error_k = y_sp - y;
    integral = integral + (error_k * Ts) + (Kaw * (u_true - u_kn1));

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



