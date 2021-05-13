#include "PidCtrlr.h"

/**
 * @brief Construct object with controller parameters and time constants
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
    EnaIpart = 1;

    /** \bug 2021-05-13 MFO: this is a test bug! */
    /** \todo this is a test todo! */
    /** \todo this is second test todo! */

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
 * @brief Update controller sample time
 * 
 * However, it is recommended to keep it fixed 
 * 
 * @param _Ts [s], controller sample time
 */
void PidCtrlr::setTs(float _Ts)
{   
    Ts = _Ts;
}

/**
 * @brief Update the controller gains during run-time in case gain-scheduling is used
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
 * @brief Update filter constant of derivative part low-pass filter
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
 * @brief Initialize integrator with a defined value
 * 
 * Typically this function is used to reset the integrator to 0 in case the conditions 
 * change drastically (e.g. during fuel cut-off in a lambda controller)
 * 
 * @param _IpartInit [-], reset value for integrator
 */
void PidCtrlr::setIpart(float _IpartInit)
{   
    integral = _IpartInit;
}

/**
 * @brief Calculate the next PID controller output
 * 
 * Note that the actuator is typically limited to physical limits (e.g. 0..100%).
 * If not, u_true can simply be set to u_kn1, the last controller output.
 * If feed-foorward is not used, simply set it to 0.
 * 
 * \f$ \text{integral}[k] = \text{integral[k-1]} + (e[k] \cdot Ts) + K_{AW} \cdot (u_\text{true} - u[k-1]) \f$
 * 
 * \f$ u[k] = e[k] \cdot K_P + \text{integral}[k] \cdot K_I \f$
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

    if(EnaIpart)
    {
        integral = integral + (error_k * Ts) + (Kaw * (u_true - u_kn1));
    }
    
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

/**
 * @brief Freeze integral part with current value but continue all other parts
 */
void PidCtrlr::freezeIpart()
{
    EnaIpart = 0;
}

/**
 * @brief Resume calculation of integral part from last frozen value onwards
 */
void PidCtrlr::resumeIpart()
{
    EnaIpart = 1;
}



