/* @file Collection of useful functions
 * 
 * @author E. M. Luebeck
 * @date 2021-04-23
 */

#ifndef PROJECTLIB_H
#define PROJECTLIB_H

//#include "WConstants.h"
#include <Arduino.h>
#include <math.h>

float LookupTable(float axis[], float data[], uint8_t size, float input);
float saturate(float in, float LimLwr, float LimUpr);
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max);
uint16_t read16(File &f);
uint32_t read32(File &f);
boolean getBit16(uint16_t BitWord, uint16_t PosnBit);
uint16_t setBit16(uint16_t BitWord, uint16_t PosnBit);
uint16_t clearBit16(uint16_t BitWord, uint16_t PosnBit);
uint16_t toggleBit16(uint16_t BitWord, uint16_t PosnBit);
uint16_t putBit16(uint16_t BitWord, uint16_t PosnBit, boolean BitNew);

float LookupTable(float axis[], float data[], uint8_t size, float input)
{   /* Axis must be strictly monotonic increasing */

    float output = 0;
    float factor = 0;

    /* saturate under-/overflow */
    if(input <= axis[0]) 
    {   //Serial.println("Underflow");
        return(data[0]);
    }
    else if(input >= axis[size-1]) 
    {   //Serial.println("Overflow");
        return(data[size-1]);
    }  
    else
    {   //Serial.println("Interpolation");
        for(int i=0; i < size; i++)
        {   
            if(input == axis[i]) 
            {   //Serial.println("Exact breakpoint");
                return(data[i]);
            }            
            else if(axis[i] > input)
            {   /* linear interpolation */

                // Serial.print("Between Breakpoints [");
                // Serial.print(axis[i-1],3);
                // Serial.print(", ");
                // Serial.print(axis[i],3);
                // Serial.println("]");

                factor = (input-axis[i-1]) / (axis[i]-axis[i-1]);
                output = factor * (data[i]-data[i-1]) + data[i-1];
                return(output);
            }
        }
    }

    /* should not be reached */
    return(input);
}

float saturate(float in, float LimLwr, float LimUpr)
{
    if(in >= LimUpr) return(LimUpr);
    if(in <= LimLwr) return(LimLwr);
    return(in);
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

boolean getBit16(uint16_t BitWord, uint16_t PosnBit)
{
    if(BitWord & (1 << PosnBit))
    {
        return(1);
    }
    else
    {
        return(0);
    }
}

uint16_t setBit16(uint16_t BitWord, uint16_t PosnBit)
{
    BitWord |=  1 << PosnBit;
    return(BitWord);
}
 
uint16_t clearBit16(uint16_t BitWord, uint16_t PosnBit)
{
    BitWord &= ~(1 << PosnBit);
    return(BitWord);
}

uint16_t putBit16(uint16_t BitWord, uint16_t PosnBit, boolean BitNew)
{
    if(BitNew)
    {
        BitWord |=  1 << PosnBit;
    }
    else
    {
        BitWord &= ~(1 << PosnBit);
    }

    return(BitWord);
}
 
uint16_t toggleBit16(uint16_t BitWord, uint16_t PosnBit)
{
    if(BitWord & (1 << PosnBit)) BitWord &= ~(1 << PosnBit);
    else BitWord |=  1 << PosnBit;
    return(BitWord);
}

#endif /* PROJECTLIB_H */
