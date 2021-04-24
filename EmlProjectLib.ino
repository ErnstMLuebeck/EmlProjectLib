#include <Arduino.h>

#include "LowPassFilter.h"
#include "TurnOnOffDelay.h"

#define NUM_TESTCASE_SAMPLES 400
#define NUM_PRE_SAMPLES 100

float Ts = 0.01; /* [s], testcase sample time */

LowPassFilter LPF1 = LowPassFilter(Ts, 0.1, 0);
TurnOnOffDelay TOOD1 = TurnOnOffDelay(0, 0);

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

        if((i%75) == 0) state = !state;
        Square[i] = (float)state;
    }

    Delta[NUM_PRE_SAMPLES] = 1;
}

void loop()
{
    /* Execut testcases */
    for(int i=0; i<NUM_TESTCASE_SAMPLES; i++)
    {
        float x1, x2, y1, y2; 

        /* TC001 */
        // LPF1.setTc(0.1);
        // x1 = Sigma[i];
        // y1 = LPF1.calculate(x1);

        /* TC002 */
        // LPF1.setTc(0.03);
        // x1 = Delta[i];
        // y1 = LPF1.calculate(x1);

        /* TC003 */
        // TOOD1.setTiDly(100, 0);
        // x1 = Square[i];
        // y1 = TOOD1.update(x1);

        /* TC004 */
        // TOOD1.setTiDly(0, 100);
        // x1 = Square[i];
        // y1 = TOOD1.update(x1);

        /* TC005 */
        // TOOD1.setTiDly(50, 100);
        // x1 = Square[i];
        // y1 = TOOD1.update(x1);

        Serial.print(x1, 4);
        Serial.print(", ");
        // Serial.print(x2, 4);
        // Serial.print(", ");
        Serial.print(y1, 4);
        Serial.print(", ");
        Serial.println();

        delay(Ts*1000);
    }

    /* Stop execution */
    while(1);    
}

