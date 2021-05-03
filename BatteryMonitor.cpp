#include "BatteryMonitor.h"

/** 
 * Construct battery monitor object
 * 
 * @param _CapBattery [mAh], Battery capacity
 * @param _TiOpBattFull [s], Time the device operates with a full battery
 * @param _PinVBatt [-], ADC pin used to read the battery voltage
 * @param _NrMeasAvg [#], number of voltage reading which are averaged for each measurement 
 * @param _TiMeasDlyMs [ms], time between voltage readings before averaging
 */
BatteryMonitor::BatteryMonitor(
    float _CapBattery, 
    float _TiOpBattFull,
    int _PinVBatt, 
    uint16_t _NrMeasAvg, 
    uint16_t _TiMeasDlyMs)
{
    CapBattery = _CapBattery;
    TiOpBattFull = _TiOpBattFull;
    PinVBatt = _PinVBatt;
    NrMeasAvg = _NrMeasAvg;
    TiMeasDlyMs = _TiMeasDlyMs;

    /* Set pin mode to input */
    pinMode(PinVBatt, INPUT);
}

/**
 * Set filter time constants (Ts, Tc) and enable battery voltage filtering (default is disabled)
 */
void BatteryMonitor::setVBattFilt(float _TsFilterVBatt, float _TcFilterVBatt)
{
    /* by default the filter is inactive */
    EnaVBattFilt = true;

    TsFilterVBatt = _TsFilterVBatt;
    TcFilterVBatt = _TcFilterVBatt;

    LpfVBatt.setTs(TsFilterVBatt);
    LpfVBatt.setTc(TcFilterVBatt);

}

/**
 * Set the 2-point calibration values for the VBatt ADC (ADC values and true voltages at two points)
 * 
 * @param _ValAdcCal1 [-], ADC value at lower voltage (default: 0)
 * @param _VTrueAdcCal1 [V], corresponding true voltage measured with multimeter (default: 0.0)
 * @param _ValAdcCal2 [-], ADC value at higher voltage (default: 1024)
 * @param _VTrueAdcCal2 [V], corresponding true voltage measured with multimeter (default: 6.6)
 */ 
void BatteryMonitor::calVBattAdc(
    float _ValAdcCal1, 
    float _VTrueAdcCal1, 
    float _ValAdcCal2, 
    float _VTrueAdcCal2)
{
    ValAdcCal1 = _ValAdcCal1;
    VTrueAdcCal1 = _VTrueAdcCal1;
    ValAdcCal2 = _ValAdcCal2;
    VTrueAdcCal2 = _VTrueAdcCal2;
}

/**
 * Initialize battery monitor and respective filters with first battery voltage reading.
 * It is recommended to call this function when the current drawn from the battery is
 * close to the operating current in order to get plausible readings.
 */
void BatteryMonitor::init()
{
    VBattRaw = readBattVoltage();

    /* Only for ADC calibration */
    float ValAdcRaw, ValAdcFilt;
    readAdcValue(&ValAdcRaw, &ValAdcFilt);
    LpfAdc.setValue(ValAdcRaw);

    if(EnaVBattFilt)
    {
        LpfVBatt.setValue(VBattRaw);
        VBattFilt = LpfVBatt.calculate(VBattRaw);
    }
    else
    {
        VBattFilt = VBattRaw;
    }

}
/**
 * Takes a defined number of ADC measurements and averages them. Afterwards a lowpass filter
 * is applied to get a very stable ADC reading. Use a high quality multimeter and measure
 * the corresponding voltage (before voltage devider). The ADC value as well as the multimeter
 * voltage reading can the be used to calibrate the battery monitor.
 * 
 * @return ValAdcRaw [0..1024], average of 100 ADC readings every 1ms
 * @return ValAdcFilt [0..1024], low-pass filtered value (Ts = 1s, Tc = 5s)
 */
void BatteryMonitor::readAdcValue(float* ValAdcRaw, float* ValAdcFilt)
{
    float ValAdcMean = 0;

    /* Measure NrMeasAvg times and calculate average value */
    int NrAvg = 100;
    for(int i=0; i < NrAvg; i++)
    {   /* Battery voltage is read through 10k-10k voltage divider */

        ValAdcMean += (float)analogRead(PinVBatt);

        delay(1);
    }

    *ValAdcRaw = ValAdcMean/(float)NrAvg;
    *ValAdcFilt = LpfAdc.calculate(*ValAdcRaw);

}

/**
 * Take a defined number of battery voltage readings and average them to reduce
 * measurement noise. The value is mapped from ADC to a true voltage (2-point calibration).
 * 
 * @return VBattRaw [V], average of NrMeasAvg voltage readings (mapped using 2-point calibration)
 */
float BatteryMonitor::readBattVoltage()
{
    float VBattMean = 0;

    /* Measure NrMeasAvg times and calculate average value */
    for(int i=0; i < NrMeasAvg; i++)
    {   
        /* Battery voltage is read through 10k-10k voltage divider */
        //float VBattTemp = (float)analogRead(PinVBatt)/1024.0 * 2 * 3.218; /* 1024 = 3.3 V */

        /* Use 2-point calibration to map the ADC values on true voltages */
        float VBattTemp = PL::mapfloat((float)analogRead(PinVBatt), ValAdcCal1, ValAdcCal2, VTrueAdcCal1, VTrueAdcCal2);

        VBattMean += VBattTemp;

        delay(TiMeasDlyMs);
    }

    return(VBattMean/(float)NrMeasAvg);
}

/** 
 * Update all battery states: VBatt, SOC, TiOpRemain, etc.
 * This method needs to be called every TsFilterVBatt if low-pass filtering is used.
 * If filter is not active, the method can also be called asynchronously.
 */
void BatteryMonitor::update()
{
    VBattRaw = readBattVoltage();

    if(EnaVBattFilt)
    {
        VBattFilt = LpfVBatt.calculate(VBattRaw);
    }
    else
    {
        VBattFilt = VBattRaw;
    }

    /* Map battery voltage to state-of-charge */
    SocBatt = PL::LookupTable(VBat2Soc_A, VBat2Soc_M, VBat2Soc_N, VBattFilt);
    TiOpRemain = TiOpBattFull * SocBatt/100; /* [min], remaining battery operation time */
}

/** 
 * Get the newest, raw battery voltage (unfiltered)
 * 
 * @return VBattRaw [V], unfiltered battery voltage
 */
float BatteryMonitor::getVBattRaw()
{
    return(VBattRaw);
}

/** 
 * Get the newest, filtered battery voltage
 * 
 * @return VBattFilt [V], low-pass filtered battery voltage
 */
float BatteryMonitor::getVBattFilt()
{
    return(VBattFilt);
}

/** 
 * Get current state of charge
 * 
 * @return SocBatt [%], state-of-charge
 */
float BatteryMonitor::getSocBatt()
{
    return(SocBatt);
}

/** 
 * Get remaining battery operation time in seconds
 * 
 * @return TiOpRemain [s], remaining battery operation time
 */
float BatteryMonitor::getTiOpRemain()
{
    return(TiOpRemain);
}







