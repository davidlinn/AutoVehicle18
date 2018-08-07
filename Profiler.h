/*
 * Profiler.h - This class assists in profiling an application by simple time measurement
 * between specified starting and ending points in code. It keeps track of min, max, and
 * average time spent between the points, as well as what percentage of the total profiling
 * time was spent in the code block. Users should consider all the paths the code could
 * take when setting starting and ending points and understand how multiple running tasks
 * could affect results.
 *
 *  Created on: Jul 17, 2018
 *      Author: David Linn
 */

#ifndef PROFILER_H_
#define PROFILER_H_

#include <HiResTimer.h>

#define NUM_BLOCKS 10 //Number of code blocks that can be profiled

class Profiler {
public:
	static void tic(int task); //call at start of code block
	static void toc(int task); //call at end of code block

	//Starts profiling, reads time in seconds by calling timer->readTime()
	//Global read-only timer must be already running before passed in:
		//HiResTimer* t = HiResTimer::getHiResTimer();
		//t->init();
		//t->start();
	static void start(HiResTimer* timer);

	//Starts profiling, reads time by calling a static or global namespace function
	static void start(double (*func)());

	//Stops profiling, prints results to serial out in milliseconds
	static void stop();

	struct Block {
		double blockStart = 0; //time tic() was called
		double sum = 0; //sum of all times between tic() and toc()
		int num = 0; //num times Block was measured
		double min = 1; //minimum time between tic() and toc()
		double max = 0; //max time between tic() and toc()
	};

private:
	static HiResTimer* t; //Global read-only timer already started
	static double readHiResTimer(); //Wrapper for t->readTime()
	static double (*readTime)(); //Function that returns time
	static Block blockArray[NUM_BLOCKS];
	static double profileStart; //As long as profileStart is 0, tic and toc will do nothing
};

#endif /* PROFILER_H_ */
