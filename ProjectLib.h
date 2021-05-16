/**@file ProjectLib.h
 * 
 * Collection of useful functions.
 * 
 * @author E. M. Luebeck
 * @date 2021-04-23
 */
#pragma once

#include <Arduino.h>
#include <math.h>

namespace ProjectLib {

/* Signal conditioning */
float LookupTable(float axis[], float data[], uint8_t size, float input);
float saturate(float in, float LimLwr, float LimUpr);
boolean checkInterval(float in, float LimLwr, float LimUpr);
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max);

/* Bit operations */
boolean getBit16(uint16_t BitWord, uint16_t PosnBit);
uint16_t setBit16(uint16_t BitWord, uint16_t PosnBit);
uint16_t clearBit16(uint16_t BitWord, uint16_t PosnBit);
uint16_t toggleBit16(uint16_t BitWord, uint16_t PosnBit);
uint16_t putBit16(uint16_t BitWord, uint16_t PosnBit, boolean BitNew);
uint8_t genPrbs7(uint8_t seed);

/* Matrix operations */
void MatrixMultiply(float* A, float* B, int m, int p, int n, float* C);
void MatrixPrint(float* A, int m, int n);
void MatrixSubtract(float* A, float* B, int m, int n, float* C);
void MatrixAdd(float* A, float* B, int m, int n, float* C);
void MatrixScale(float* A, int m, int n, float k, float* C);
void MatrixCopy(float* A, int n, int m, float* B);
void MatrixTranspose(float* A, int m, int n, float* C);
int MatrixInvert(float* A, int n);

/* Field oriented control transformations */
void ClarkeTransform(float Ia, float Ib, float Ic, float *Ialpha, float *Ibeta);
void ParkTransform(float Ialpha, float Ibeta, float Theta, float *Id, float *Iq);
void ParkTransformInverse(float Vd, float Vq, float Theta, float *Valpha, float *Vbeta);
void ClarkeTransformInverse(float Valpha, float Vbeta, float *Va, float *Vb, float *Vc);

/* Sun model */
int date2day(int day, int month);
void calcSunAngle(float lati, float longi, int month, int day, int hour, int minute, float* azimut, float* elevation);
void calcSunriseTime(float lati, float longi, int month, int day, float AgEleMin, int* rise_h, int* rise_min);
void calcSunsetTime(float lati, float longi, int month, int day, float AgEleMin, int* set_h, int* set_min);

} /* namespace ProjectLib */