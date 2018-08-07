/*
 * Logger.h
 *
 *  Created on: Jun 11, 2018
 *      Author: David Linn
 */

#ifndef LOGGER_H_
#define LOGGER_H_

class Logger {
public:
	static void logBegin(); //begins continuous logging by creating logLoop task
	static void logWrite(char* buf); //allows logging

	//methods not designed to be called from outside class
	static void logLoop(void*); //main logging loop
};

#endif /* LOGGER_H_ */
