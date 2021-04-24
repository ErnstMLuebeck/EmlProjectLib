#include <Arduino.h>

#include "LowPassFilter.h"
#include "TurnOnOffDelay.h"
#include "StateSpaceModel.h"
#include "PidCtrlr.h"
#include "ProjectLib.h"

#define NUM_TESTCASE_SAMPLES 500
#define NUM_PRE_SAMPLES 100

float Ts = 0.01; /* [s], testcase sample time */

LowPassFilter LPF1 = LowPassFilter(Ts, 0.1, 0);
TurnOnOffDelay TOOD1 = TurnOnOffDelay(0, 0);
StateSpaceModel SSM1 = StateSpaceModel();
PidCtrlr PIDC1 = PidCtrlr(1.0, 1.0, 0.0, 0.0, 0.0, Ts);

/* Testcase stimuli */
float Sigma[NUM_TESTCASE_SAMPLES];
float Delta[NUM_TESTCASE_SAMPLES];
float Square[NUM_TESTCASE_SAMPLES];


void setup()
{
    Serial.begin(9600);
    delay(100);

    /* Generate testcase stimuli */
    for(int i=0; i<NUM_PRE_SAMPLES; i++)
    {
        Sigma[i] = 0;
        Delta[i] = 0;
    }

    boolean state = 0;
    for(int i=NUM_PRE_SAMPLES; i<NUM_TESTCASE_SAMPLES; i++)
    {
        Sigma[i] = 1;
        Delta[i] = 0;

        if(((i-NUM_PRE_SAMPLES)%75) == 0) state = !state;
        if(state) Square[i] = 1.0;
        else Square[i] = -1.0;
    }

    Delta[NUM_PRE_SAMPLES] = 1;
}

void loop()
{
    float x1 = 0;
    float x2 = 0;
    float y1 = 0;
    float y2 = 0;
    float y3 = 0; 

    /* Execut testcases */
    for(int i=0; i<NUM_TESTCASE_SAMPLES; i++)
    {
        /* TC001 Lowpass filter */
        // LPF1.setTc(0.1);
        // x1 = Sigma[i];
        // y1 = LPF1.calculate(x1);

        /* TC002 Lowpass filter */
        // LPF1.setTc(0.03);
        // x1 = Delta[i];
        // y1 = LPF1.calculate(x1);

        /* TC003 Turn on delay */
        // TOOD1.setTiDly(100, 0);
        // x1 = Square[i];
        // y1 = TOOD1.update(x1);

        /* TC004 Turn off delay */
        // TOOD1.setTiDly(0, 100);
        // x1 = Square[i];
        // y1 = TOOD1.update(x1);

        /* TC005 Turn on/off delay */
        // TOOD1.setTiDly(50, 100);
        // x1 = Square[i];
        // y1 = TOOD1.update(x1);

        /* TC006 State Space Model */
        // x1 = Square[i]; /* Vq */
        // x2 = 0; /* TqLoa */
        // float u[2][1], y[1][1];
        // u[0][0] = x1;
        // u[1][0] = x2;
        // SSM1.calculate((float*)u);
        // SSM1.getStates((float*)y);
        // y1 = y[0][0];
        // y2 = y[1][0];
        // y3 = y[2][0];

        /* TC007 PID Controller (using State Space Model) */
        // PIDC1.setCtrlrGains(2.0, 1.2, 0.0, 0.0);
        // PIDC1.setTcDpart(0.2);

        // x1 = Sigma[i]; /* y_sp */
        // x2 = PIDC1.calculate(x1, y2, x2, 0.0); /* u1 */

        // float u[2][1], y[1][1];
        // u[0][0] = x2;
        // u[1][0] = 0;
        // SSM1.calculate((float*)u);
        // SSM1.getStates((float*)y);
        // y1 = y[0][0]; /* not used */
        // y2 = y[1][0]; /* controlled variable */
        // y3 = y[2][0]; /* not used */

        /* TC008 PID Controller (using State Space Model) with anti-windup */
        PIDC1.setCtrlrGains(2.0, 1.2, 0.0, 0.0);
        PIDC1.setTcDpart(0.2);

        x1 = Sigma[i]; /* y_sp */
        x2 = PIDC1.calculate(x1, y2, x2, 0.0); /* u1 */

        x2 = saturate(x2, -1.5, 1.5);

        float u[2][1], y[1][1];
        u[0][0] = x2;
        u[1][0] = 0;
        SSM1.calculate((float*)u);
        SSM1.getStates((float*)y);
        y1 = y[0][0]; /* not used */
        y2 = y[1][0]; /* controlled variable */
        y3 = y[2][0]; /* not used */

        Serial.print(x1, 4);
        Serial.print(", ");
        Serial.print(x2, 4);
        Serial.print(", ");
        // Serial.print(y1, 4);
        // Serial.print(", ");
        Serial.print(y2, 4);
        Serial.print(", ");
        // Serial.print(y3, 4);
        // Serial.print(", ");
        Serial.println();

        delay(Ts*1000);
    }

    /* Stop execution */
    while(1);    
}

