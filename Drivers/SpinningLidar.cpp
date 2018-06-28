/*
 * SpinningLidar.cpp
 *
 *  Created on: Jun 22, 2018
 *      Author: NetBurner
 */

#include "SpinningLidar.h"
#include <basictypes.h>
#include <ucos.h>
#include <serial.h>
#include <stdio.h>
#include <pins.h>
#include <pin_irq.h>
#include "VehDefs.h"
#include <iosys.h>
#include <math.h>

namespace SpinningLidar {
	//Global members
	volatile double dist[360];
	volatile int sampleQuality[360];
	//Members not for use outside namespace
	char lidar_buffer[128];
	int rv; //num chars read
	bool LIDARReset = false;
	int charCount = 0;
	unsigned char packet[5];

	void SpinningLidarInit() {
		OSSimpleTaskCreatewName(LidarTask,SPINNING_LIDAR_PRIO,"LIDARTask");
	}

	int checkHealth(int fds) {
		lidar_buffer[0] = 0xA5;
		lidar_buffer[1] = 0x52;
		write(fds, lidar_buffer, 2);
		rv=0;
		while (rv < 7)
			rv += read(fds, lidar_buffer + rv, 7-rv); //response descriptor
		rv = 0;
		while (rv < 3)
			rv += read(fds, lidar_buffer + rv, 3-rv); //data response packet
		if (lidar_buffer[0] != 0) {
			printf("Check Spinning LIDAR health: status %i, error code %x %x\n",
					lidar_buffer[0],(unsigned char)lidar_buffer[2], (unsigned char)lidar_buffer[1]);
			return -1;
		}
		return 0;
	}

	void hardReset(int fds) {
		LIDARReset = true;
		printf("Performing hard reset of spinning LIDAR\n");
		lidar_buffer[0] = 0xA5;
		lidar_buffer[1] = 0x40; //StartScan() request code (RPK-02-Communication-Protocol.pdf, pg14)
		write(fds, lidar_buffer, 2);
	}

	void startScan(int fds) {
		lidar_buffer[0] = 0xA5;
		lidar_buffer[1] = 0x20; //StartScan() request code (RPK-02-Communication-Protocol.pdf, pg14)
		write(fds, lidar_buffer, 2); //Serial write: transmit from start of buffer (index 0) to end
		rv=0;
		while (rv < 7)
			rv += read(fds, lidar_buffer + rv, 7-rv); //response descriptor
		//printf("\n%i bytes read: 0x", rv);
		//for (int i = 0; i < rv; ++i)
			//printf("%x ", (unsigned char)lidar_buffer[i]);
	}

	int processChar(int fds, unsigned char c) { //5 bytes, returns start bit or -1 if error
			packet[charCount]=c;
			++charCount;
			if (charCount<5) return 0;
			charCount=0;
			//Uncomment for low-level serial debugging: Print out 5-character packets in HEX
			//printf("\nPacket read: 0x");
			//for (int i = 0; i < 5; ++i)
				//printf("%x ", packet[i]);
			bool startBit = packet[0] & 0x01;
			bool notStartBit = ((packet[0])>>1) & 0x01;
			if (startBit==notStartBit) { //Packet is bad :(
				printf("ERROR IN PACKET READ, MAY HAVE MISSED A BYTE");
				if (!LIDARReset) {
					hardReset(fds);
					OSTimeDly(2);
					startScan(fds);
				}
				return -1;
			}
			//Packet is good if down here
			uint16_t d;
			d = packet[4];
			d = (d << 8) | packet[3]; //concatenate distance bytes
			uint16_t a;
			a = packet[2];
			a = (a << 7) | ((packet[1]) >> 1); //concatenate angle bytes
			uint8_t quality = (packet[0]) >> 2; //mask start bits and get sample quality
			//Save result
			int index = (int)(round(a/64.))%360; //calculate angle in degrees
			sampleQuality[index]=quality; //Save quality of reading
			dist[index]=(d/40.); //Save result in cm
			return startBit;
		}

	void LidarTask(void *) {
		int fds=SimpleOpenSerial(LIDAR_PORT,115200);
		if (fds <= 0) {
			printf("LIDAR serial open failed\n");
			return;
		}
		else printf("LIDAR serial open successful");
		if (checkHealth(fds)!=0) {
			hardReset(fds);
			OSTimeDly(2);
			if (checkHealth(fds)!=0) return;
		}
		printf("Checked LIDAR health\n");
		startScan(fds);
		printf("Started LIDAR scan\n");

		//Uncomment for high-level debugging: check how many samples in one 360 degree scan
		/*readPacket(fds); //read first packet that contains a start bit
		for (int i = 0; i < 500; ++i) {
			int sampleNum = 0;
			int startBit = 0;
			while (!startBit) {
				rv = read(fds,lidar_buffer,128);
				for (int j = 0; j < rv; ++j)
					if (processchar(fds,lidar_buffer[j])) startBit=1;
				sampleNum+=rv;
			}
			printf("\n%i samples in scan", sampleNum);
		}
		OSTimeDly(150);*/

		while (1) {
			rv = read(fds,lidar_buffer,128);
			for (int j = 0; j < rv; ++j)
				processChar(fds,lidar_buffer[j]);
		}
	}
}//end namespace

