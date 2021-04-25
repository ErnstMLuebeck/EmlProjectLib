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
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max);

/* Bit operations */
boolean getBit16(uint16_t BitWord, uint16_t PosnBit);
uint16_t setBit16(uint16_t BitWord, uint16_t PosnBit);
uint16_t clearBit16(uint16_t BitWord, uint16_t PosnBit);
uint16_t toggleBit16(uint16_t BitWord, uint16_t PosnBit);
uint16_t putBit16(uint16_t BitWord, uint16_t PosnBit, boolean BitNew);

/* Matrix operations */
void MatrixMultiply(float* A, float* B, int m, int p, int n, float* C);
void MatrixPrint(float* A, int m, int n);
void MatrixSubtract(float* A, float* B, int m, int n, float* C);
void MatrixAdd(float* A, float* B, int m, int n, float* C);
void MatrixScale(float* A, int m, int n, float k, float* C);
void MatrixCopy(float* A, int n, int m, float* B);
void MatrixTranspose(float* A, int m, int n, float* C);
int MatrixInvert(float* A, int n);

} /* namespace ProjectLib */