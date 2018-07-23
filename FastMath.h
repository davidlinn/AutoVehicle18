/*
 * FastMath.h: NOT ALL FUNCS HAVE BEEN IMPLEMENTED
 *
 *  Created on: Jul 18, 2018
 *      Author: NetBurner
 */

#ifndef FASTMATH_H_
#define FASTMATH_H_

void setupFastMath(); //sets up lookup tables
void testFastMath();
void testFastMath2();

//Degree-LookupVal conversion: 4096 (2^12) lookup vals correspond to 360 degrees
int fixedToLookupVal(int degFixed);

//Multiply by 2^18 to get an int with lower 18 bits representing fractional bits
//Can only represent vals between -8191 and 8191
int floatToFixed(float flt);
int floatToFixed(int flt);
float fixedToFloat(int fixed);

float Zto360Wrap(float deg);
int FixZto360Wrap(int deg);

//Convert float to a fixed pt, then lookupVal if neccesary, look up, convert back to float
float fastsin(float deg);
float fastcos(float deg);
float fasttan(float deg);
float fastatan2(float y, float x);
float fastasin(float deg);
float fastacos(float deg);
float fastsqrt(float val);

//One step ahead of the functions above, input is already in fixed pt
int fixsin(int deg);
int fixcos(int deg);
int fixtan(int deg);
int fixatan2(int y, int x);
int fixasin(int deg);
int fixacos(int deg);
int fixsqrt(int val);

#endif /* FASTMATH_H_ */
