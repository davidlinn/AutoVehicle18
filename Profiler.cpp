/*
 * Profiler.cpp - This class assists in profiling an application by simple time measurement
 * between specified starting and ending points in code. It keeps track of min, max, and
 * average time spent between the points, as well as what percentage of the total profiling
 * time was spent in the code block. Users should consider all the paths the code could
 * take when setting starting and ending points and understand how multiple running blocks
 * could affect results.
 *
 *  Created on: Jul 17, 2018
 *      Author: David Linn
 */

#include "Profiler.h"

//Redeclaration of static members
HiResTimer* Profiler::t;
double (*Profiler::readTime)(); //function called when time is read
Profiler::Block Profiler::blockArray[NUM_BLOCKS];
double Profiler::profileStart; //time profiling was started

void Profiler::tic(int block) {
	if (profileStart!=0)
		blockArray[block].blockStart = readTime();
}

void Profiler::toc(int block) {
	//If profiling has begun and tic() was called for the block, capture time
	if (profileStart!=0 && blockArray[block].blockStart!=0) {
		++blockArray[block].num;
		double deltaTime = readTime() - blockArray[block].blockStart;
		blockArray[block].sum += deltaTime;
		if (deltaTime < blockArray[block].min) blockArray[block].min = deltaTime;
		if (deltaTime > blockArray[block].max) blockArray[block].max = deltaTime;
	}
}

void Profiler::start(HiResTimer* timer) {
	t = timer;
	readTime = readHiResTimer;
	profileStart = readTime();
}

void Profiler::start(double (*func)()) {
	readTime = func;
	profileStart = readTime();
}

void Profiler::stop() { //stops profiling, prints out results
	double totalTime = readTime() - profileStart;
	printf("\n         Avg    Min    Max    PctTotal\n");
	for (int i = 0; i < NUM_BLOCKS; ++i) {
		double avg = 1000*blockArray[i].sum/blockArray[i].num;
		double min = 1000*blockArray[i].min;
		double max = 1000*blockArray[i].max;
		double pctTotal = 100*(blockArray[i].sum/totalTime);
		printf("Block %i: %4.3f, %4.3f, %4.3f, %3.1f pct\n",i,avg,min,max,pctTotal);
	}
}

double Profiler::readHiResTimer() {
	return t->readTime();
}
