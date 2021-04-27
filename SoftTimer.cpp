#include "SoftTimer.h"

/** Construct software timer object
 * 
 * @param _ModeTime [-], time unit 0=ms, 1=us
 */
SoftTimer::SoftTimer(int _ModeTmr)
{
    ModeTmr = _ModeTmr;
    TiTmrStart = 0;
    TiTmrStop = 0;
    FlgTmrRunning = 0;
}

/** 
 * Get the current or last held time of the timer
 * 
 * @return TiTmr [ms] or [us], time between timer start and now (or timer stopped)
 */
long SoftTimer::getTime()
{
    if(FlgTmrRunning)
    {
        switch(ModeTmr)
        {   /* Timer is counting in [ms] */
            case 0:
                TiTmr = millis() - TiTmrStart; 
                break;
            /* Timer is counting in [us] */
            case 1: 
                TiTmr = micros() - TiTmrStart; 
                break;
            default: break;
        }
    }
    /* Timer is not running */
    else
    {
        TiTmr = TiTmrStop - TiTmrStart;
    }

    return(TiTmr);
}

/** 
 * Start the timer. Calling start while the timer is running does not restart it.
 */
void SoftTimer::start()
{
    if(!FlgTmrRunning)
    {
        switch(ModeTmr)
        {   /* Timer is counting in [ms] */
            case 0:
                TiTmrStart = millis(); 
                TiTmrStop = 0;
                TiTmr = 0;
                break;
            /* Timer is counting in [us] */
            case 1: 
                TiTmrStart = micros(); 
                TiTmrStop = 0;
                TiTmr = 0;
                break;
            default: break;
        }
    }

    FlgTmrRunning = 1;
}

/** 
 * Pause the timer if it is running.
 */
void SoftTimer::pause()
{
    if(FlgTmrRunning)
    {
        switch(ModeTmr)
        {   /* Timer is counting in [ms] */
            case 0:
                TiTmrStop = millis(); 
                break;
            /* Timer is counting in [us] */
            case 1: 
                TiTmrStop = micros(); 
                break;
            default: break;
        }
    }

    FlgTmrRunning = 0;
}

/** 
 * Resume the timer if it is paused.
 */
void SoftTimer::resume()
{
    if(!FlgTmrRunning)
    {   
        long TiNow;

        switch(ModeTmr)
        {   /* Timer is counting in [ms] */
            case 0:
                TiNow = millis(); 
                break;
            /* Timer is counting in [us] */
            case 1: 
                TiNow = micros(); 
                break;
            default: break;
        }
        TiTmrStart += TiNow - TiTmrStop;
    }
    FlgTmrRunning = 1;
}

/** 
 * Reset the timer
 * 
 */
void SoftTimer::reset()
{
    TiTmrStart = 0;
    TiTmrStop = 0;
    FlgTmrRunning = 0;
}
