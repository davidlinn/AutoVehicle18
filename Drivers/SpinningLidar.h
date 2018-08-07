/*
 * SpinningLidar.h
 *
 *  Created on: Jun 22, 2018
 *      Author: NetBurner
 */

#ifndef SPINNINGLIDAR_H_
#define SPINNINGLIDAR_H_

#include <basictypes.h>

namespace SpinningLidar {

	//Creates RTOS task: Runs LidarTask()
	void SpinningLidarInit();

	//results of most recent scan, where the coordinate system is not as defined by Nav, but
	//is as defined by SLAMTEC, the manufacturers of RPLIDAR. 0 is straight ahead, 90 is to the right.
	extern volatile int dist[360];
	extern volatile int sampleQuality[360];

	//-----------------------------------------------------
	//Not designed to be accessed from outside namespace

	//opens serial port, sends a StartScan() request to the LIDAR, continuously
	//reads results, and processes them by calling LIDAR_ProcessChar
	void LidarTask(void *);

	int checkHealth(int fds);

	void hardReset(int fds);

	void startScan(int fds);

	int processChar(int fds, unsigned char c);

	//defines the 5 bytes of a data response packet
	//(see RPK-02-Communication-Protocol.pdf, pg14)
	struct OneResult {
		uint8_t qf; //quality[7:2], not start bit, start bit
		uint8_t a_lsb; //raw angle
		uint8_t a_msb;
		uint8_t d_lsb; //raw distance
		uint8_t d_msb;
	};
}

#endif /* SPINNINGLIDAR_H_ */
