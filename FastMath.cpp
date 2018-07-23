/*
 * FastMath.cpp
 *
 *  Created on: Jul 18, 2018
 *      Author: NetBurner
 */

#include "FastMath.h"
#include <math.h>
#include <constants.h>
#include <HiResTimer.h>
#include "Profiler.h"

float sinTable[4096];
int fixSinTable[4096];

void setupFastMath() { //sets up lookup tables
	for (int i = 0; i<4096; ++i) {
		sinTable[i] = sin(i*2*M_PI/4096);
		fixSinTable[i] = floatToFixed(sinTable[i]);
	}
}

void testFastMath() {
	HiResTimer* t = HiResTimer::getHiResTimer(GLOBAL_TIMER);
	t->init();
	t->start();
	float a, b;
	Profiler::start();
	for (int i = 0; i < 10; ++i)
		a = sin(i*M_PI/360.);
	Profiler::tic(2);
	for (int i = 0; i < 10; ++i)
		b = fastsin(i);
	Profiler::toc(2);
	printf("%f,%f\n",a,b);
	Profiler::tic(3);
	for (int i = 0; i < 10; ++i)
		a = cos(i*M_PI/360.);
	Profiler::toc(3);
	Profiler::tic(4);
	for (int i = 0; i < 10; ++i)
		b = fastcos(i);
	Profiler::toc(4);
	printf("%f,%f\n",a,b);
	Profiler::tic(5);
	for (int i = 0; i < 10; ++i)
		a = tan(i*M_PI/360.);
	Profiler::toc(5);
	Profiler::tic(6);
	for (int i = 0; i < 10; ++i)
		b = fasttan(i);
	Profiler::toc(6);
	printf("%f,%f\n",a,b);
	Profiler::tic(7);
	a = 3290.745;
	b = fixedToFloat(floatToFixed(a));
	Profiler::toc(7);
	printf("%f,%f",a,b);
	Profiler::tic(8);
	a = tan(4*M_PI/360.);
	Profiler::toc(8);
	Profiler::tic(9);
	b = fixtan(floatToFixed(4));
	Profiler::toc(9);
	printf("%f,%f",a,b);
	Profiler::stop();
}

void testFastMath2() {
	HiResTimer* t = HiResTimer::getHiResTimer(GLOBAL_TIMER);
	t->init();
	t->start();
	volatile float a;
	volatile int b;
	int c[5] = { 0,1,2,3,7 };
	Profiler::start();
	Profiler::tic(0);
	for (float i = 0; i < 100; i+=.1) //1000 muls,1000 adds
		a = 2145.39109+i*4423.7434324;
	Profiler::toc(0);
	Profiler::tic(1);
	for (float i = 0; i < 10000; i+=1) //10000 muls,10000 adds
		b = 45920+i*32184;
	Profiler::toc(1);
	Profiler::tic(2);
	a = 2.539*3.103;
	Profiler::toc(2);
	Profiler::tic(3);
	b = 9489*4910;
	Profiler::toc(3);
	Profiler::tic(4);
	a = sqrt(4.57);
	Profiler::toc(4);
	Profiler::tic(5);
	b = pow(10,4);
	Profiler::toc(5);
	Profiler::tic(6);
	b = c[4];
	Profiler::toc(6);
	Profiler::stop();
}

//Degree-LookupVal conversion: 4096 (2^12) lookup vals correspond to 360 degrees
int fixedToLookupVal(int degFixed) {
	return ((degFixed/360)*4096)>>18;
}

//Multiply by 2^18 to get an int with lower 18 bits representing fractional bits
//Can only represent vals between -8191 and 8191
int floatToFixed(float flt) {
	return (int)(flt*262144);
}
int floatToFixed(int flt) {
	return flt*262144;
}
float fixedToFloat(int fixed) {
	return ((float)fixed)/262144;
}

float Zto360Wrap(float deg) {
	while (deg < 0) deg += 360;
	while (deg >= 360) deg -= 360;
	return deg;
}
int FixZto360Wrap(int deg) {
	while (deg < 0) deg += floatToFixed(360);
	while (deg > floatToFixed(360)) deg -= floatToFixed(360);
	return deg;
}

//Convert float to a fixed pt, then lookupVal if neccesary, look up, convert back to float
float fastsin(float deg) {
	return sinTable[(int)(Zto360Wrap(deg)*4096/360)];
}
float fastcos(float deg) {
	return sinTable[(int)(Zto360Wrap(deg+90)*4096/360)];
}
float fasttan(float deg) {
	return fastsin(deg)/fastcos(deg);
}
//float fastatan2(float y, float x);
//float fastasin(float deg);
//float fastacos(float deg);

//Input is in fixed pt, just shift
int fixsin(int deg) {
	return fixSinTable[FixZto360Wrap(deg<<12/360)];
}
int fixcos(int deg) {
	return fixSinTable[FixZto360Wrap((deg+floatToFixed(90))<<12/360)];
}
int fixtan(int deg) {
	return fixsin(deg)/fixcos(deg);
}
//int fixatan2(int y, int x);
//int fixasin(int deg);
//int fixacos(int deg);
