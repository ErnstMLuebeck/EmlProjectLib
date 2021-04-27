#ifndef SOFTTIMER_H
#define SOFTTIMER_H

#include <Arduino.h>

/**
 * Software timer
 * 
 * @author E. M. Luebeck
 * @date 2021-04-23
 */ 
class SoftTimer 
{
    public:
        SoftTimer(int _ModeTmr);
        long getTime();
        void start();
        void pause();
        void resume();
        void reset();
        
    private:
        /** [-], timer running state */
        boolean FlgTmrRunning;
        /** [-], time unit 0=ms, 1=us */
        int ModeTmr;
        /** [ms] or [us], absolute time the timer was started */
        long TiTmrStart;
        /** [ms] or [us], absolute time the timer was stopped/paused */
        long TiTmrStop;
        /** [ms] or [us], last evaluated time */
        long TiTmr;
};

#endif /* SOFTTIMER_H */


