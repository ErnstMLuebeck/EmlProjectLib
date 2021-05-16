/**@file EmlProjectLib.ino */

#include <Arduino.h>

#include "ProjectLib.h"
#include "LowPassFilter.h"
#include "TurnOnOffDelay.h"
#include "StateSpaceModel.h"
#include "PidCtrlr.h"
#include "SignalMonitor.h"
#include "Hysteresis.h"
#include "IIRFilterBiquad.h"
#include "SoftTimer.h"
#include "BatteryMonitor.h"
#include "DataList.h"
#include "MovgAvgFilter.h"

namespace PL=ProjectLib;

#define NUM_TESTCASE_SAMPLES 500
#define NUM_PRE_SAMPLES 100

float Ts = 0.01; /* [s], testcase sample time */

LowPassFilter LPF1 = LowPassFilter(Ts, 0.1, 0);
TurnOnOffDelay TOOD1 = TurnOnOffDelay(0, 0);
StateSpaceModel SSM1 = StateSpaceModel();
PidCtrlr PIDC1 = PidCtrlr(1.0, 1.0, 0.0, 0.0, 0.0, Ts);
SignalMonitor SM1 = SignalMonitor(0);
Hysteresis HYS1 = Hysteresis(-0.5, 0.5, 0);
SoftTimer ST1 = SoftTimer(1);
IIRFilterBiquad IIR1 = IIRFilterBiquad(1);
BatteryMonitor BM1 = BatteryMonitor(1800, 300, A2, 10, 1);
DataList DL1 = DataList();
MovgAvgFilter MAF1 = MovgAvgFilter(30);

/* Testcase stimuli */
float Sigma[NUM_TESTCASE_SAMPLES];
float Delta[NUM_TESTCASE_SAMPLES];
float Square[NUM_TESTCASE_SAMPLES];
float Saw[NUM_TESTCASE_SAMPLES];
float Triangle[NUM_TESTCASE_SAMPLES];
float Random[NUM_TESTCASE_SAMPLES];

void setup()
{
    /* Hold power supply MOSFET switched on */
    pinMode(6, OUTPUT);
    digitalWrite(6, HIGH);

    Serial.begin(9600);
    while(!Serial); /* does not work without PC connected! */

    /* Generate testcase stimuli */
    for(int i=0; i<NUM_PRE_SAMPLES; i++)
    {
        Sigma[i] = 0;
        Delta[i] = 0;
    }

    boolean state = 0;
    float temp = 0;
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

        /* Pseudo random sequence */
        //if(SM1.detectChange(Square[i]))
        {   temp = 2 * ( (float)PL::genPrbs7(5)/(float)127 - 0.5);
        }
        Random[i] = temp;
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

    /* TC017 SoftTimer */
    // ST1.start();

    //IIR1.setCoeff(10, 100);

    /* TC024 Battery Monitor */
    // BM1.setVBattFilt(Ts, 1);
    // BM1.calVBattAdc(539.0, 3.25, 622.0, 3.97);
    // BM1.init();

    /* TC026 DataList */
    // char name[] = "Item1";
    // DL1.addItemHead(name, 666);
    // DL1.addItemHead(name, 667);
    // Serial.println(DL1.getNumItems());
    // DL1.printListConsole();
    // Serial.println(DL1.getItemName(1));
    // Serial.println(DL1.getItemData(1));

    // Serial.println(DL1.getItemData(2));

    /* TC027 Sunset and sunrise time model */
    // Coordinates Graz Kossgasse
    float lati = 47.0651;
    float longi = 15.4631;
    int month, daym, h, min;
    month = 5;
    day = 14;
    PL::calcSunriseTime(lati, longi, month, day, 8.0, &h, &min);
    Serial.print(h);
    Serial.print(":");
    Serial.println(min);

    PL::calcSunsetTime(lati, longi, month, day, 0.0, &h, &min);
    Serial.print(h);
    Serial.print(":");
    Serial.println(min);
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
        // y1 = PL::LookupTable(axis, data, 3, x1);

        /* TC014 mapfloat */
        //x1 = Triangle[i];
        // x1 = 500;
        // //(533, 3.70, 622, 3.97)
        // y1 = PL::mapfloat(x1, 539, 622, 3.25, 3.97);
        
        /* TC015 Bit operations */
        // uint16_t Status = 0;
        // Status = PL::setBit16(Status, 1);
        // Serial.println(Status, BIN); /* Must be: 0b10*/
        // Status = PL::toggleBit16(Status, 1);
        // Serial.println(Status, BIN); /* Must be: 0b0*/
        // Status = PL::setBit16(Status, 7);
        // Serial.println(Status, BIN); /* Must be: 0b10000000 */
        // Serial.println(PL::getBit16(Status, 7)); /* Must be: 1 */
        // Status = PL::putBit16(Status, 6, 0);
        // Serial.println(Status, BIN); /* Must be: 0b10000000 */
        // Status = PL::putBit16(Status, 5, 1);
        // Serial.println(Status, BIN); /* Must be: 0b10100000 */
        // Status = PL::putBit16(Status, 7, 0);
        // Serial.println(Status, BIN); /* Must be: 0b100000 */

        /* TC016 Matrix operations */
        // float A[3][3] = {{1, 2, 3}, {4, 5, 6}, {7, 8, 9}};
        // float x[3][1] = {{1}, {2}, {3}};
        // float B[3][3] = {{0}};
        // float C[3][3] = {{0}};
        // int rows = 3;
        // int cols = 3;

        // PL::MatrixPrint((float*)A, rows, cols);
        // PL::MatrixTranspose((float*)A, rows, cols, (float*)B);
        // PL::MatrixPrint((float*)B, rows, cols);
        // PL::MatrixAdd((float*)A, (float*)B, rows, cols, (float*)C);
        // PL::MatrixPrint((float*)C, rows, cols);
        // PL::MatrixSubtract((float*)C, (float*)B, rows, cols, (float*)C);
        // PL::MatrixPrint((float*)C, rows, cols);
        // PL::MatrixScale((float*)A, rows, cols, 0.5,(float*)C);
        // PL::MatrixPrint((float*)C, rows, cols);
        // PL::MatrixCopy((float*)A, rows, cols, (float*)C);
        // PL::MatrixPrint((float*)C, rows, cols);
        // PL::MatrixMultiply((float*)A, (float*)x, rows, cols, 1, (float*)C);
        // PL::MatrixPrint((float*)C, rows, 1);
        // PL::MatrixInvert((float*)A, rows);
        // PL::MatrixPrint((float*)A, rows, cols);

        /* TC017 Software Timer: pause/resume */
        // x1 = Square[i];
        // if(x1 == 1.0) ST1.pause();
        // else ST1.resume();
        // y1 = ST1.getTime()/(float)1000;

        /* TC018 Software Timer: reset */
        // x1 = Square[i];
        // if(x1 == 1.0) ST1.reset();
        // else ST1.start();
        // y1 = ST1.getTime()/(float)1000;

        /* TC019 IIR Biquad Filter */
        // x1 = Square[i];
        // y1 = IIR1.calculate(x1);

        /* TC020 FOC Inverse Park transformation */
        // x1 = Saw[i];
        // PL::ParkTransformInverse(0, 1, x1*2*PI, &y1, &y2);

        /* TC021 FOC Inverse Park/Clarke transformation */
        // x1 = Saw[i];
        // float Valpha, Vbeta;
        // PL::ParkTransformInverse(0, 1, x1*2*PI, &Valpha, &Vbeta);
        // PL::ClarkeTransformInverse(Valpha, Vbeta, &y1, &y2, &y3);

        /* TC022 FOC Inverse- and forward Park/Clarke transformation */
        // x1 = Saw[i];
        // float Valpha, Vbeta, Va, Vb, Vc;
        // PL::ParkTransformInverse(1, 0, x1*2*PI, &Valpha, &Vbeta);
        // PL::ClarkeTransformInverse(Valpha, Vbeta, &Va, &Vb, &Vc);
        // PL::ClarkeTransform(Va, Vb, Vc, &Valpha, &Vbeta);
        // PL::ParkTransform(Valpha, Vbeta, x1*2*PI, &y1, &y2);

        /* TC023 Performance: FOC Inverse- and forward Park/Clarke transformation */
        // x1 = Saw[i];
        // int N = 10000; /* number of calculations */
        // ST1.reset();
        // ST1.start();
        // for(int i=0; i<N; i++)
        // {
        //     float Valpha, Vbeta, Va, Vb, Vc;
        //     PL::ParkTransformInverse(1, 0, x1*2*PI, &Valpha, &Vbeta);
        //     PL::ClarkeTransformInverse(Valpha, Vbeta, &Va, &Vb, &Vc);
        //     PL::ClarkeTransform(Va, Vb, Vc, &Valpha, &Vbeta);
        //     PL::ParkTransform(Valpha, Vbeta, x1*2*PI, &y1, &y2);
        // }
        // y3 = ST1.getTime(); /* Calculation time in [us] */

        /* TC024 Battery Monitor */
        // BM1.update();
        // BM1.readAdcValue(&y1, &y2);
        // y3 = BM1.getVBattRaw();
        // y2 = BM1.getVBattFilt();
        // y3 = BM1.getSocBatt();

        /* TC025 Check interval */
        // x1 = Saw[i];
        // y1 = PL::checkInterval(x1, 0.2, 0.8);

        /* TC026 Pseudo random sequence */
        // y1 = PL::genPrbs7(3);

        /* TC027 Moving average filter */
        // x1 = Random[i];
        // y1 = MAF1.calculate(x1);

        /* TC027 Sun model */
        // Coordinates Graz Kossgasse
        // float lati = 47.0651;
        // float longi = 15.4631;
        // x1 = Saw[i]*12;
        // PL::calcSunAngle(lati, longi, x1, 1, 12, 0, &y1, &y2);

        /*-----------------------------------------------------------------------------------*/
        /* Plot Signals */
        // Serial.print(x1, 4);
        // Serial.print(", ");
        // Serial.print(x2, 4);
        // Serial.print(", ");
        // Serial.print(y1, 4);
        // Serial.print(", ");
        // Serial.print(y2, 4);
        // Serial.print(", ");
        // Serial.print(y3, 4);
        // Serial.print(", ");
        // Serial.println();

        delay(Ts*1000);
    }

    /* Stop execution */
    while(1);    
}

