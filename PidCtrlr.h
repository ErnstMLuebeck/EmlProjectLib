#ifndef PIDCTRLR_H_
#define PIDCTRLR_H_

/** 
 * Parallel PID controller with anti-windup and filtered D-part. The class also features
 * functions for gain scheduling (update controller gains during run-time), set value of integrator,
 * as well as an input for feed-forward control.
 * 
 * @author E. M. Luebeck
 * @date 2021-04-23
 */
class PidCtrlr
{
    public:
    PidCtrlr(float _Kp, float _Ki, float _Kd, float _Kaw, float _TcDpart, float _Ts);
    float calculate(float y_sp, float y, float u_true, float u_ff);
    void setTs(float _Ts);
    void setCtrlrGains(float _Kp, float _Ki, float _Kd, float _Kaw);
    void setIpart(float _IpartInit);

    private:
    float Ts; /* [s], sample time */
    float Kp, Ki, Kd, Kaw; /* [-], controller gains */
    float error_kn1; /* [-], control error of last sample kn1 = [k-1], k being the time index*/
    float u_kn1; /* [-], controller output of last sample */
    float derivative_kn1; /* [-], previous derivative part for low-pass filter */
    float integral; /* [-], integrator of integral part */
    float TcDpart; /* [s], filter time constant of derivative part filter */
    float CoeffFiltDpart; /* [-], filter coefficient of derivative part filter */
};

#endif /*  PIDCTRLR_H_ */


