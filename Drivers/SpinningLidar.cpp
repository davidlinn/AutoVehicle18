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
	char lidar_buffer[128];
	int rv; //num chars read
	volatile double dist[360];
	volatile int sampleQuality[360];

	void SpinningLidarInit() {
		OSSimpleTaskCreatewName(LidarTask,SPINNING_LIDAR_PRIO,"LIDARTask");
	}

	void LidarTaskOld(void *) {
		int fds=SimpleOpenSerial(LIDAR_PORT,115200);
		if (fds <= 0) {
			printf("LIDAR serial open failed\n");
			return;
		}
		OSTimeDly(2);
		char lidar_buffer[128];
		lidar_buffer[0]=0xA5;
		lidar_buffer[1]=0x20; //StartScan() request code (RPK-02-Communication-Protocol.pdf, pg14)
		lidar_buffer[2]=0;
		write(fds,lidar_buffer,2); //Serial write: transmit from start of buffer (index 0) to end
		while(1)
		{
			int rv=read(fds,lidar_buffer,5); //Serial read populates lidar_buffer array
			if(rv>0) //if no error
			{
			  for(int i=0; i<rv; i++)
			  {
				LIDAR_ProcessChar(lidar_buffer[i]); //process each character
			  }
			}
		}
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

	bool LIDARReset = false;
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

	int readPacket(int fds) { //5 bytes, returns start bit or -1 if error
		rv=0;
		while (rv < 5)
			rv += read(fds, lidar_buffer + rv, 5-rv); //response descriptor
		//OneResult packet;
		//packet->qf=lidar_buffer[0];
		//packet->a_lsb =lidar_buffer[1];

		printf("\n%i bytes read: 0x", rv);
		for (int i = 0; i < rv; ++i)
			printf("%x ", (unsigned char)lidar_buffer[i]);

		uint16_t d;
		d = lidar_buffer[4];
		d = (d << 8) | lidar_buffer[3]; //concatenate distance bytes

		uint16_t a;
		a = lidar_buffer[2];
		a = (a << 7) | (((unsigned char)lidar_buffer[1]) >> 1); //concatenate angle bytes

		uint8_t quality = ((unsigned char)lidar_buffer[0]) >> 2; //mask start bits and get sample quality

		if (quality>0) {
			int index = (a/64)%360;
			sampleQuality[index]=quality; //Save quality of reading
			dist[index]=(d/40.); //Save result in cm
		}
		bool startBit = ((unsigned char)lidar_buffer[0]) & 0x01;
		bool notStartBit = (((unsigned char)lidar_buffer[0])>>1) & 0x01;
		if (startBit != notStartBit)
			return startBit; //return startBit if it corresponds with ~startBit
		else {
			printf("ERROR IN PACKET READ, MAY HAVE MISSED A BYTE");
			return -1;
		}
	}

	int charCount = 0;
	unsigned char packet[5];
	int processchar(int fds, unsigned char c) { //5 bytes, returns start bit or -1 if error
			packet[charCount]=c;
			++charCount;
			if (charCount<5) return 0;
			charCount=0;
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
				processchar(fds,lidar_buffer[j]);
		}

		rv = 0;
		bool sBit; //start bit
		while(!sBit) {
			rv = read(fds,lidar_buffer,5);
			sBit = (lidar_buffer[0] & 0x01);
		}
		while(1)
		{
			if(rv==5) //if no error
			{
				LIDAR_ProcessChar2((uint8_t*)lidar_buffer); //process packet of 5 chars
			}
			rv=read(fds,lidar_buffer,5); //Serial read populates lidar_buffer array
		}
	}

	//volatile unsigned char packet[5];
	int packetIndex=0;

	//results of most recent scan, where the coordinate system is not as defined by Nav, but
	//is as defined by SLAMTEC, the manufacturers of RPLIDAR. 0 is straight ahead, 90 is to the right.
	volatile double prevDist[360];
	volatile int prevSampleQuality[360];

	void LIDAR_ProcessChar(uint8_t c) {
		packet[packetIndex++] = c;   //put the character as the next byte in our data response packet
		if (packetIndex == 5) {        //we have a full data response packet (5 bytes)!
			OneResult * po;
			po = (OneResult *) packet; //save a pointer to a struct containing the bytes
			if (packet[0] & 0x01) { //if the Start flag bit is set (a full 360 deg scan is COMPLETE)
				//printf("Start flag\n");
				//for (int i = 0; i < 360; i++) { //for each degree
					//prevDist[i] = dist[i]; //copy everything to the previous result arrays
					//prevSampleQuality[i] = sampleQuality[i];
					//dist[i] = 0; //set all elems in current result arrays to 0
					//sampleQuality[i] = 0;
				//}
			} //end full scan block

			uint16_t d;
			d = po->d_msb;
			d = (d << 8) | po->d_lsb; //concatenate distance bytes

			uint16_t a;
			a = po->a_msb;
			a = (a << 7) | (po->a_lsb >> 1); //concatenate angle bytes

			uint8_t ss = po->qf >> 2; //mask start bits and get sample quality
			packetIndex = 0; //reset byte number
			if (ss>0) {
				int index = (a/64)%360;
				sampleQuality[index]=ss; //Save quality of reading
				dist[index]=(d/40.); //Save result in cm
			}
			//printf("Putting %i cm at angle %i w/ quality %i\n", dist[index],index,sampleQuality[index]);
		} //end full response packet action
	} //end ProcessLidarChar

	void LIDAR_ProcessChar2(uint8_t * c) {
				uint16_t d;
				d = c[4];
				d = (d << 8) | c[3]; //concatenate distance bytes

				uint16_t a;
				a = c[2];
				a = (a << 7) | (c[1] >> 1); //concatenate angle bytes

				uint8_t ss = c[0] >> 2; //mask start bits and get sample quality
				packetIndex = 0; //reset byte number
				if (ss>0) {
					int index = (a/64)%360;
					sampleQuality[index]=ss; //Save quality of reading
					dist[index]=(d/40.); //Save result in cm
				}
				//printf("Putting %i cm at angle %i w/ quality %i\n", dist[index],index,sampleQuality[index]);
		} //end ProcessLidarChar

}//end namespace

