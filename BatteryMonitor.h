#ifndef BATTERYMONITOR_H
#define BATTERYMONITOR_H

#include <Arduino.h>
#include "LowPassFilter.h"
#include "ProjectLib.h"

namespace PL = ProjectLib;

/**
 * Monitoring for a single cell LiPo battery
 * 
 * @author E. M. Luebeck
 * @date 2021-05-03
 */ 
class BatteryMonitor 
{
    public:
        BatteryMonitor::BatteryMonitor(
            float _CapBattery, 
            float _TiOpBattFull,
            int _PinVBatt, 
            uint16_t _NrMeasAvg, 
            uint16_t _TiMeasDlyMs);

        void init();
        void readAdcValue(float* ValAdcRaw, float* ValAdcFilt);
        void calVBattAdc(float _ValAdcCal1, float _VTrueAdcCal1, float _ValAdcCal2, float _VTrueAdcCal2);
        float readBattVoltage();
        void update();
        void setVBattFilt(float _TsFilterVBatt, float _TcFilterVBatt);
        float getVBattRaw();
        float getVBattFilt();
        float getSocBatt();
        float getTiOpRemain();
        
    private:
        uint16_t PinVBatt;
        float CapBattery;
        float VBattRaw;
        float VBattFilt;
        float SocBatt;
        uint16_t NrMeasAvg;
        uint16_t TiMeasDlyMs;

        /* 2-point ADC calibration default values (1:1 voltage divider) */
        float ValAdcCal1 = 0;
        float VTrueAdcCal1 = 0.0;
        float ValAdcCal2 = 1024;
        float VTrueAdcCal2 = 6.6;

        boolean EnaVBattFilt = false;
        float TsFilterVBatt;
        float TcFilterVBatt;

        LowPassFilter LpfVBatt = LowPassFilter(1.0, 1.0, 0.0); /* state initial value set later */
        LowPassFilter LpfAdc = LowPassFilter(1.0, 10.0, 0.0); /* only for calibration */

        float TiOpBattFull;
        float TiOpRemain;

        // /* 201114 Tolino battery calibration 1500mAh */
        // const uint8_t VBat2Soc_N = 12; /* Mapsize */
        // float VBat2Soc_A[VBat2Soc_N] = {3.4407, 3.5631, 3.6016, 3.6675, 3.7014, 3.7264, 3.7635, 3.8166, 3.8836, 3.9422, 4.0342, 4.1293}; /* [V], battery voltage */
        // float VBat2Soc_M[VBat2Soc_N] = {0.0, 5.0, 10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 70.0, 80.0, 90.0, 100.0}; /* [%], state of charge */
        // float TiOpBattFull = 360; /* [min], Tolino operation time with a full battery */
        // float TiOpBatRemain = 360;

        /* 210305 iPhone 4 battery 1420mAh */
        const uint8_t VBat2Soc_N = 15; /* Mapsize */
        float VBat2Soc_A[15] = {2.9992, 3.3595, 3.5165, 3.6003, 3.6251, 3.6825, 3.7179, 3.7422, 3.7763, 3.8177, 3.8854, 3.9396, 4.0160, 4.0800, 4.1205};
        float VBat2Soc_M[15] = {0.0, 1.0, 3.0, 6.0, 10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 70.0, 80.0, 90.0, 98.0, 100.0};
        // float TiOpBattFull = 290; /* [min], iPhone4 operation time with a full battery */
        // float TiOpBatRemain = 290;

};

#endif /* BATTERYMONITOR_H */


