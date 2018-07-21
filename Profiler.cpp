/*
 * Profiler.cpp
 *
 *  Created on: Jul 17, 2018
 *      Author: NetBurner
 */

#include "Profiler.h"

HiResTimer* Profiler::t;
Profiler::Task Profiler::taskArray[NUM_TASKS];
double Profiler::profileStart;

void Profiler::tic(int task) { //call at start of task
	if (profileStart!=0)
		taskArray[task].taskStart = t->readTime();
}

void Profiler::toc(int task) { //call at end of task
	if (taskArray[task].taskStart!=0) {
		++taskArray[task].num;
		double deltaTime = t->readTime() - taskArray[task].taskStart;
		taskArray[task].sum += deltaTime;
		if (deltaTime < taskArray[task].min) taskArray[task].min = deltaTime;
		if (deltaTime > taskArray[task].max) taskArray[task].max = deltaTime;
	}
}

void Profiler::start() { //starts profiling, reads current time
	t = HiResTimer::getHiResTimer(GLOBAL_TIMER);
	profileStart = t->readTime();
}

void Profiler::stop() { //stops profiling, prints out results
	double totalTime = t->readTime() - profileStart;
	printf("\n         Avg    Min    Max    PctTotal\n");
	for (int i = 0; i < NUM_TASKS; ++i) {
		double avg = 1000*taskArray[i].sum/taskArray[i].num;
		double min = 1000*taskArray[i].min;
		double max = 1000*taskArray[i].max;
		double pctTotal = 100*(taskArray[i].sum/totalTime);
		printf("Task %i: %4.3f, %4.3f, %4.3f, %3.1f pct\n",i,avg,min,max,pctTotal);
	}

}
