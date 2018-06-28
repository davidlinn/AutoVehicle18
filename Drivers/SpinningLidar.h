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

	//-----------------------------------------------------
	//Not designed to be accessed from outside namespace

	//opens serial port, sends a StartScan() request to the LIDAR, continuously
	//reads results, and processes them by calling LIDAR_ProcessChar
	void LidarTaskOld(void *);

	void LidarTask(void *);

	void LIDAR_ProcessChar(uint8_t c);

	void LIDAR_ProcessChar2(uint8_t * c);

	int checkHealth(int fds);

	void startScan(int fds);

	int readPacket(int fds);

	//defines the 5 bytes of a data response packet
	//(see RPK-02-Communication-Protocol.pdf, pg14)
	struct OneResult {
		uint8_t qf; //quality[7:2], not start bit, start bit
		uint8_t a_lsb; //raw angle
		uint8_t a_msb;
		uint8_t d_lsb; //raw distance
		uint8_t d_msb;
	};

	extern volatile double dist[360];
	extern volatile int sampleQuality[360];
	//results of most recent scan, where the coordinate system is not as defined by Nav, but
	//is as defined by SLAMTEC, the manufacturers of RPLIDAR. 0 is straight ahead, 90 is to the right.
	extern volatile double prevDist[360];
	extern volatile int prevSampleQuality[360];
}

#endif /* SPINNINGLIDAR_H_ */
