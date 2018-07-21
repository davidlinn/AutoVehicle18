/*
 * Profiler.h
 *
 *  Created on: Jul 17, 2018
 *      Author: NetBurner
 */

#ifndef PROFILER_H_
#define PROFILER_H_

#include <HiResTimer.h>
#include "VehDefs.h"

#define NUM_TASKS 10 //Number of tasks that can be profiled

class Profiler {
public:
	static void tic(int task); //call at start of task or process to profile
	static void toc(int task); //call at end of task or process to profile
	static void start(); //starts profiling, reads current time
	static void stop(); //stops profiling, prints out results

	struct Task {
		double taskStart;
		double sum;
		int num;
		double min = 1;
		double max;
	};

private:
	static HiResTimer* t; //Relies on a global read-only timer already running
	static Task taskArray[NUM_TASKS];
	static double profileStart;
};

#endif /* PROFILER_H_ */
