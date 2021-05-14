

#ifndef MOVGAVGFILTER_H
#define MOVGAVGFILTER_H

#include <Arduino.h>

/**
 * @brief Moving Average Filter
 * 
 * @author E. M. Luebeck
 * @date 2021-05-14
 */
class MovgAvgFilter
{
    public:
        MovgAvgFilter(uint8_t _Len);
        float calculate(float);

    private:
        /** filter length */
        uint8_t Len;
        /** filter buffer */
        float* buffer;
        /** filter coefficient */
        float alpha;
        /** buffer write index */
        uint8_t pointer;

};

#endif /* MOVGAVGFILTER_H */


