/*
 * IMU.h
 *
 *  Created on: Jun 4, 2018
 *      Author: NetBurner
 */

#ifndef IMU_H_
#define IMU_H_

#include <basictypes.h>

class IMU {
public:
	//Constructs an IMU object, initializes I2C communication, takes raw register data at specified priority
	//Use Utility::i2cscan() to find i2c address if unknown
	IMU(uint8_t i2cAddress, int priority);

	//Methods designed to be used by main()
	void printRaw();

	//Raw interfacing
	static void ReadGAData(int16_t * dest);   //Puts raw accel,temp,gyro registers in destination[7]
	void readAccelData(int16_t * destination); //Puts raw accelerometer registers in destination[3]
	void readGyroData(int16_t * destination); //Puts raw gyro registers in destination[3]
	void readMagData(int16_t * destination); //Puts raw magnetometer registers in destination[3]

	//I2C Interfacing Helper Functions
	static uint8_t ReadReg(uint8_t addr, uint8_t reg);
	static void WriteReg(uint8_t addr,uint8_t reg, uint8_t val);
	static uint8_t I2CWriteReadBuffer(uint8_t address, uint8_t writeVal, uint8_t *buffer, int readLength);
	static void ReadManyReg(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest);

	//Sampling and initial processing
	static void IMUSampleTask(void*); //Real-Time Operating System activated task: puts measurement registers in IMUResults[8] and CompassResult[3]
	static void ProcessIMUResults();

	//Do-nothings
	IMU(); //Do-nothing constructor
	virtual ~IMU(); //Do-nothing destructor

private:
	uint8_t MPU_ADDRESS; //I2C address of IMU, inititialized in constructor as 0x68
	int prio; //Priority of sampling task
	int whoAmICheck(); //Returns 0 for successful initialization, -1 for fail
	void initMPU9250(); //IMU initialization routine- enable registers, setting clocks, etc.- implemented by Kris Winer
	void activeDataModeInit(); //starts IMUSampleTask() at prio
};

#endif /* IMU_H_ */
