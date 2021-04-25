/**@file EmlProjectLib.ino */

/*! \mainpage Collection of useful classes and functions
 *
 * \section intro_sec Introduction
 *
 * This collection of classes and functions is aimed to be used in microcontroller hardware
 * projected, e.g Arduino, Teensy, ESP32, etc.
 * 
 * Most of them are tested in the EmlProjectLib.ino sketch, where (very) simple testcases are executed.
 *
 * \section license_sec License
 *
 * Feel free to use and adapt these function as you wish. If your rocket crashes, please do not blame me.
 *
 */

#include <Arduino.h>

#include "LowPassFilter.h"
#include "TurnOnOffDelay.h"
#include "StateSpaceModel.h"
#include "PidCtrlr.h"
#include "ProjectLib.h"
#include "SignalMonitor.h"
#include "Hysteresis.h"
#include "IIRFilter.h"

#define NUM_TESTCASE_SAMPLES 500
#define NUM_PRE_SAMPLES 100

float Ts = 0.01; /* [s], testcase sample time */

LowPassFilter LPF1 = LowPassFilter(Ts, 0.1, 0);
TurnOnOffDelay TOOD1 = TurnOnOffDelay(0, 0);
StateSpaceModel SSM1 = StateSpaceModel();
PidCtrlr PIDC1 = PidCtrlr(1.0, 1.0, 0.0, 0.0, 0.0, Ts);
SignalMonitor SM1 = SignalMonitor(0);
Hysteresis HYS1 = Hysteresis(-0.5, 0.5, 0);

float test_num[3] = {0.052004382885834, 0.008913673017008, 0.666};
float test_den[3] = {1.000000000000000, -0.939081944097158, 0.666};
IIRFilter IIR1 = IIRFilter(test_num, test_den, 3);

/* Testcase stimuli */
float Sigma[NUM_TESTCASE_SAMPLES];
float Delta[NUM_TESTCASE_SAMPLES];
float Square[NUM_TESTCASE_SAMPLES];
float Saw[NUM_TESTCASE_SAMPLES];
float Triangle[NUM_TESTCASE_SAMPLES];

void setup()
{
    Serial.begin(9600);
    while(!Serial); /* does not work without PC connected! */

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

        int Len = 75;
        int LenMod = (i-NUM_PRE_SAMPLES)%Len;

        if(LenMod == 0) state = !state;
        
        if(state) 
        {
            Square[i] = 1.0;
            Triangle[i] = 2*LenMod/(float)Len - 1.0;
        }
        else 
        {
            Square[i] = -1.0;
            Triangle[i] = 2*(Len-LenMod)/(float)Len - 1.0;
        }
        Saw[i] = LenMod/(float)Len;
    }

    Delta[NUM_PRE_SAMPLES] = 1;

    /* TC006 State Space Model */
    float A[3][3] = {{0.70970, -0.03520, 0.00000}, {0.05609, 0.99858, 0.00000}, {0.00003, 0.00100, 1.00000}};
    float B[3][2] = {{0.10057, 0.00220}, {0.00330, -0.12491}, {0.00000, -0.00006}};
    float C[1][3] = {{0.00001, 0.00050, 1.00000}};

    SSM1.initStateSpaceModel((float*)A, (float*)B, (float*)C, 3, 2, 1, Ts);

    float p[1][3] = {{0.80000, 0.81000, 0.82000}};
    float L[3][1] = {{-22.65057}, {25.86176}, {0.26568}};

    SSM1.initStateObserver((float*)L, (float*)p);


    /* TC015 IIR Filter debugging */
    // IIR1.printFilterData();
    // IIR1.calculate(1);
    // IIR1.printFilterData();
    // IIR1.calculate(2);
    // IIR1.printFilterData();
    // IIR1.calculate(3);
    // IIR1.printFilterData();
    // IIR1.calculate(4);
    // IIR1.printFilterData();
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
        x1 = Square[i]; /* Vq */
        x2 = 0; /* TqLoa */
        float u[2][1], y[1][1];
        u[0][0] = x1;
        u[1][0] = x2;
        SSM1.calculate((float*)u);
        SSM1.getStates((float*)y);
        y1 = y[0][0];
        y2 = y[1][0];
        y3 = y[2][0];

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
        // PIDC1.setCtrlrGains(2.0, 1.2, 0.0, 0.0);
        // PIDC1.setTcDpart(0.2);

        // x1 = Sigma[i]; /* y_sp */
        // x2 = PIDC1.calculate(x1, y2, x2, 0.0); /* u1 */

        // x2 = saturate(x2, -1.5, 1.5);

        // float u[2][1], y[1][1];
        // u[0][0] = x2;
        // u[1][0] = 0;
        // SSM1.calculate((float*)u);
        // SSM1.getStates((float*)y);
        // y1 = y[0][0]; /* not used */
        // y2 = y[1][0]; /* controlled variable */
        // y3 = y[2][0]; /* not used */

        /* TC009 Signal Monitor (change) */
        // x1 = Square[i];
        // y1 = SM1.detectChange(x1);

        /* TC010 Signal Monitor (increase) */
        // x1 = Square[i];
        // y1 = SM1.detectIncrease(x1);

        /* TC011 Signal Monitor (decrease) */
        // x1 = Square[i];
        // y1 = SM1.detectDecrease(x1);

        /* TC012 Hysteresis */
        // HYS1.setThresholds(-0.4, 0.8);
        // x1 = Triangle[i];
        // y1 = HYS1.update(x1);

        /* TC013 LookupTable */
        // x1 = Triangle[i];
        // float axis[3] = {0, 1, 2};
        // float data[3] = {2, 1, 0};
        // y1 = ProjectLib::LookupTable(axis, data, 3, x1);

        /* TC014 mapfloat */
        // x1 = Triangle[i];
        // y1 = ProjectLib::mapfloat(x1, -1, 1, -2, 2);
        
        /* TC015 Bit operations */
        // uint16_t Status = 0;
        // Status = ProjectLib::setBit16(Status, 1);
        // Serial.println(Status, BIN); /* Must be: 0b10*/
        // Status = ProjectLib::toggleBit16(Status, 1);
        // Serial.println(Status, BIN); /* Must be: 0b0*/
        // Status = ProjectLib::setBit16(Status, 7);
        // Serial.println(Status, BIN); /* Must be: 0b10000000 */
        // Serial.println(ProjectLib::getBit16(Status, 7)); /* Must be: 1 */
        // Status = ProjectLib::putBit16(Status, 6, 0);
        // Serial.println(Status, BIN); /* Must be: 0b10000000 */
        // Status = ProjectLib::putBit16(Status, 5, 1);
        // Serial.println(Status, BIN); /* Must be: 0b10100000 */
        // Status = ProjectLib::putBit16(Status, 7, 0);
        // Serial.println(Status, BIN); /* Must be: 0b100000 */

        /* TC016 Matrix operations */
        // float A[3][3] = {{1, 2, 3}, {4, 5, 6}, {7, 8, 9}};
        // float x[3][1] = {{1}, {2}, {3}};
        // float B[3][3] = {{0}};
        // float C[3][3] = {{0}};
        // int rows = 3;
        // int cols = 3;

        // ProjectLib::MatrixPrint((float*)A, rows, cols);
        // ProjectLib::MatrixTranspose((float*)A, rows, cols, (float*)B);
        // ProjectLib::MatrixPrint((float*)B, rows, cols);
        // ProjectLib::MatrixAdd((float*)A, (float*)B, rows, cols, (float*)C);
        // ProjectLib::MatrixPrint((float*)C, rows, cols);
        // ProjectLib::MatrixSubtract((float*)C, (float*)B, rows, cols, (float*)C);
        // ProjectLib::MatrixPrint((float*)C, rows, cols);
        // ProjectLib::MatrixScale((float*)A, rows, cols, 0.5,(float*)C);
        // ProjectLib::MatrixPrint((float*)C, rows, cols);
        // ProjectLib::MatrixCopy((float*)A, rows, cols, (float*)C);
        // ProjectLib::MatrixPrint((float*)C, rows, cols);
        // ProjectLib::MatrixMultiply((float*)A, (float*)x, rows, cols, 1, (float*)C);
        // ProjectLib::MatrixPrint((float*)C, rows, 1);
        // ProjectLib::MatrixInvert((float*)A, rows);
        // ProjectLib::MatrixPrint((float*)A, rows, cols);

        /* TC015 IIR Filter */
        //IIR1.printFilterData();

        /* Plot Signals */
        Serial.print(x1, 4);
        Serial.print(", ");
        Serial.print(x2, 4);
        Serial.print(", ");
        Serial.print(y1, 4);
        Serial.print(", ");
        Serial.print(y2, 4);
        Serial.print(", ");
        Serial.print(y3, 4);
        Serial.print(", ");
        Serial.println();

        delay(Ts*1000);
    }

    /* Stop execution */
    while(1);    
}

