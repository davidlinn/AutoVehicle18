/*
 * IMU.cpp
 *
 *  Created on: Jun 4, 2018
 *      Author: NetBurner
 */

#include "IMU.h"
#include <multichanneli2c.h>
#include <constants.h>
#include <stdio.h>
#include <basictypes.h>
#include "Utility.h"
#include <pins.h>
#include "IMUDefs.h"
#include <ucos.h>
#include <pin_irq.h>


IMU::IMU() { } //Default constructor does nothing

IMU::IMU(uint8_t i2cAddress, int priority) :
	MPU_ADDRESS(i2cAddress),  prio(priority) {
	//I2C should have already been initialized with a call to MultiChannel_I2CInit() in main
	//Start bit, address of slave device, R/W, ack

	//Uncomment the following two lines to check I2C communication
	//bool discovered[0x80];
	//Utility::I2CScan(discovered);

	if (whoAmICheck() == -1) return; //Fail initialization if whoAmICheck fails
	initMPU9250();
	activeDataModeInit();
}

IMU::~IMU() {
	// TODO Auto-generated destructor stub
}

extern volatile int16_t IMUResults[8];
extern volatile int16_t CompassResult[3];
extern volatile float MagHeading;
extern volatile float IntegratedHeading;
extern volatile float RawHeading;
void IMU::printRaw() {
	printf("Raw Mag- x-%i,y-%i,z-%i\n", CompassResult[0],CompassResult[1],CompassResult[2]);
	printf("Processed: RawHead- %f, MagHead- %f, IntegHead- %f\n", RawHeading, MagHeading, IntegratedHeading);
}

//===================================================================================================================
//====== Set of useful function to access acceleration. gyroscope, magnetometer data
//===================================================================================================================
void IMU::ReadGAData(int16_t * dest)
{
ReadManyReg(MPU9250_ADDRESS, ACCEL_XOUT_H, 14, (uint8_t *)dest);  // Read the seven raw data registers for MPU into data array
}


void IMU::readAccelData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z accel register data stored here
  ReadManyReg(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;
}


void IMU::readGyroData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  ReadManyReg(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;
}

void IMU::readMagData(int16_t * destination)
{
  uint8_t rawData[7];  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
  if(ReadReg(AK8963_ADDRESS, AK8963_ST1) & 0x01) { // wait for magnetometer data ready bit to be set
  ReadManyReg(AK8963_ADDRESS, AK8963_XOUT_L, 7, &rawData[0]);  // Read the six raw data and ST2 registers sequentially into data array
  uint8_t c = rawData[6]; // End data read by reading ST2 register
    if(!(c & 0x08)) { // Check if magnetic sensor overflow set, if not then report data
    destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
    destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  // Data stored as little Endian
    destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ;
   }
  }
}

//PRIVATE IMU FUNCTIONS

int IMU::whoAmICheck() { //Returns 0 for successful initialization, -1 for fail
	uint8_t val = ReadReg(MPU_ADDRESS,WHO_AM_I_REGISTER);
	if (val != 0x71) // WHO_AM_I should always be 0x71
	  {
		  printf("Whoami: 0x%x, should be 0x71\n", val);
		  iprintf("Attempting to unclod I2C bus\r\n");
		  Pins[27].function(PIN_27_GPIO);//I2C for IMU
		  Pins[29].function(PIN_29_GPIO);//I2C For IMU
		  Pins[29].hiz();
		  for(int i=0; i<256; i++)
		  {
			 Pins[27]=1;
			 for (volatile int d=0; d<100; d++) asm(" nop");
			 Pins[27]=0;
			 for (volatile int d=0; d<100; d++) asm(" nop");
		  }
		  Pins[27].function(PIN_27_I2C0_SCL    );//I2C for IMU
		  Pins[29].function(PIN_29_I2C0_SDA    );//I2C For IMU
		  I2CResetPeripheral();
		  OSTimeDly(2);
		  //val = readReg((unsigned char *)0x75);  // Read WHO_AM_I register for MPU-9250
		  val = ReadReg(MPU_ADDRESS,0x75);
		  iprintf("Retry ID=%02X\r\n",(int)val);
	 }
	if (val == 0x71) {
		printf("IMU I2C connection successful\n");
		return 0;
	}
	else {
		printf("IMU I2C connection failed\n");
		return -1;
	}
}

void IMU::initMPU9250() //Initialization routine for MPU-9250 implemented by Kris Winer
{
 // wake up device
  WriteReg(MPU9250_ADDRESS, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors
  OSTimeDly(5); // Wait for all registers to reset

 // get stable time source
  WriteReg(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);  // Auto select clock source to be PLL gyroscope reference if ready else
  OSTimeDly(10);

 // Configure Gyro and Thermometer
 // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively;
 // minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
 // be higher than 1 / 0.0059 = 170 Hz
 // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
 // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
  WriteReg(MPU9250_ADDRESS, CONFIG, 0x03);

 // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
  WriteReg(MPU9250_ADDRESS, SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; a rate consistent with the filter update rate
                                    // determined inset in CONFIG above

 // Set gyroscope full scale range
 // Range selects FS_SEL and GFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
  uint8_t c = ReadReg(MPU9250_ADDRESS, GYRO_CONFIG); // get current GYRO_CONFIG register value
 // c = c & ~0xE0; // Clear self-test bits [7:5]
  c = c & ~0x03; // Clear Fchoice bits [1:0]
  c = c & ~0x18; // Clear GFS bits [4:3]
  c = c | Gscale << 3; // Set full scale range for the gyro
 // c =| 0x00; // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
  WriteReg(MPU9250_ADDRESS, GYRO_CONFIG, c ); // Write new GYRO_CONFIG value to register

 // Set accelerometer full-scale range configuration
  c = ReadReg(MPU9250_ADDRESS, ACCEL_CONFIG); // get current ACCEL_CONFIG register value
 // c = c & ~0xE0; // Clear self-test bits [7:5]
  c = c & ~0x18;  // Clear AFS bits [4:3]
  c = c | Ascale << 3; // Set full scale range for the accelerometer
  WriteReg(MPU9250_ADDRESS, ACCEL_CONFIG, c); // Write new ACCEL_CONFIG register value

 // Set accelerometer sample rate configuration
 // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
 // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
  c = ReadReg(MPU9250_ADDRESS, ACCEL_CONFIG2); // get current ACCEL_CONFIG2 register value
  c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
  c = c | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
  WriteReg(MPU9250_ADDRESS, ACCEL_CONFIG2, c); // Write new ACCEL_CONFIG2 register value
 // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
 // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

  // Configure Interrupts and Bypass Enable
  // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
  // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips
  // can join the I2C bus and all can be controlled by the Arduino as master
   WriteReg(MPU9250_ADDRESS, INT_PIN_CFG, 0x22);
   WriteReg(MPU9250_ADDRESS, INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt

   WriteReg(AK8963_ADDRESS ,AK8963_CNTL ,0x16);

   OSTimeDly(5);
}

//-----Task Management----
OS_SEM PirqSem;
volatile uint32_t PirqCount;
void Pirq(void)
{
PirqSem.Post();
PirqCount++;
}
//-------------------------
//----Variables for IMU Processing------
volatile uint32_t LastIerror;
volatile uint32_t nCor;
volatile float lferror;

//volatile uint32_t IMUSample;
//volatile uint32_t CompassCount;

//IMUResults array: 0-AccelX,1-AccelY,2-AccelZ
//3-Temp_raw
//4-GyroX,5-GyroY,6-GyroZ,7-Ext Sensor
//Convert Temp_raw to degC w/ degC=(Temp_raw-roomTempOffset)/Temp_Sensitivity + 21 degC
//Gyro output is 131LSB*Ang rate
//Registers are ordered high byte first, then low
volatile int16_t IMUResults[8];

//0-X,1-Y,2-Z
//Registers are ordered low byte first, then high
volatile int16_t CompassResult[3];

volatile int16_t RotVel[3];
enum IMUModeEnum {eIdle,eCalibrating,eRunning};
volatile IMUModeEnum ImuMode;
volatile int32_t GAxisSum[3];
volatile int32_t AAxisSum[3];
volatile int32_t RawHeadSum;

volatile float MagHeading;
volatile float IntegratedHeading;
volatile float RawHeading;
volatile bool  bIMU_Id;

int32_t GZero[3];
int32_t AZero[3];

int32_t GZeroCalc[3];
int32_t AZeroCalc[3];

int32_t ZeroCount;
//----------------------------------------
void IMUSampleTask(void*);
void IMU::activeDataModeInit() {
    OSSimpleTaskCreatewName(IMUSampleTask,prio,"IMU");
	SetPinIrq(50, 1,Pirq);
    iprintf("MPU9250 initialized for active data mode....\r\n"); // Initialize device for active mode read of acclerometer, gyroscope, and temperature
    uint8_t d = ReadReg(AK8963_ADDRESS, AK8963_WHO_AM_I);  // Read WHO_AM_I register for AK8963
    iprintf("Whoami for magnetometer is 0x%02X, should be 0x48\r\n",d);
	ImuMode = eRunning;
    PirqSem.Post();
}
void IMU::IMUSampleTask(void*) {
	int16_t dest[8];
	while(1)
	 {
	 PirqSem.Pend();
	 if (ReadReg(MPU9250_ADDRESS, INT_STATUS) & 0x01)
	 {  // On interrupt, check if data ready interrupt
		 ReadGAData(&dest[0]);
		 //USER_ENTER_CRITICAL();
		 for(int i=0; i<8; i++) {
			 IMUResults[i]=dest[i]; //IMUResults is a class variable
		 }
		 //IMUSample++;
		 //uint32_t lm=LIDAR_MIN;
		 //LIDAR_MIN=0x7FFFFFFF;
	     //USER_EXIT_CRITICAL();
	     IMU::ProcessIMUResults();
		 //ImuR.ax=IMUResults[0];
	     //ImuR.ay=IMUResults[1];
		 //ImuR.az=IMUResults[2];
		 //ImuR.gz=IMUResults[6];
		 //ImuR.lidar=LIDAR_VALUE;
		 //ImuR.lidmin=lm;
		 //ImuR.odo=OdoCount;
		 //ImuR.dtodo=DtOdoCount;
	     //ImuR.head=IntegratedHeading;
		 //ImuR.rhead=RawHeading;
		 //ImuR.axsum=AAxisSum[0];
		 //ImuR.Log();

	     uint8_t cstatus=ReadReg(0xC,2);
		 if(cstatus& 1)
		 {
		 uint8_t buf[10];
		 I2CWriteReadBuffer(AK8963_ADDRESS ,AK8963_XOUT_L, buf,7); //Reading 7 to get ST2 register
		 uint8_t * p8=(uint8_t *)&CompassResult[0];
		 //USER_ENTER_CRITICAL();
		 p8[0]=buf[1];
		 p8[1]=buf[0];
		 p8[2]=buf[3];
		 p8[3]=buf[2];
		 p8[4]=buf[5];
		 p8[5]=buf[4];
		 //CompassCount++;
		 //USER_EXIT_CRITICAL();
		 //ProcessCompassResults();
		 //CompR.mx=CompassResult[0];
	     //CompR.my=CompassResult[1];
		 //CompR.mz=CompassResult[2];
		 //CompR.mh=MagHeading;
	//	 CompR.li=LastIerror;
	//	 CompR.lf=lferror;
		 //CompR.Log();
		 }
	 }
	 //ProcessNewImuData();
	}
}
float DegScale(int32_t v);
void IMU::ProcessIMUResults()
{
switch(ImuMode)
    {case eCalibrating:
        {
            GZero[0]+=IMUResults[4];
            GZero[1]+=IMUResults[5];
            GZero[2]+=IMUResults[6];
			AZero[0]+=IMUResults[0];
			AZero[1]+=IMUResults[1];
			AZero[2]+=IMUResults[2];
            ZeroCount++;
			GZeroCalc[0]=GZero[0]/ZeroCount;
			GZeroCalc[1]=GZero[1]/ZeroCount;
			GZeroCalc[2]=GZero[2]/ZeroCount;
		    AZeroCalc[0]=AZero[0]/ZeroCount;
			AZeroCalc[1]=AZero[1]/ZeroCount;
			AZeroCalc[2]=AZero[2]/ZeroCount;
        }
        break;
case eRunning :
     {
        for(int i=0; i<3; i++)
        {  int32_t a=IMUResults[0+i]-AZeroCalc[i];
           int32_t v=IMUResults[4+i]-GZeroCalc[i];
           AAxisSum[i]+=a;
		   RotVel[i]=v;
           GAxisSum[i]+=v;
           if (GAxisSum[i]>4718592)GAxisSum[i]-=2*4718592;
           if (GAxisSum[i]<-4718592)GAxisSum[i]+=2*4718592;
		   if(i==2)
		   {
			   RawHeadSum+=v;
			   if (RawHeadSum> 4718592)RawHeadSum-=2*4718592;
			   if (RawHeadSum<-4718592)RawHeadSum+=2*4718592;
		   }
        }
		IntegratedHeading=DegScale(GAxisSum[2]);
		RawHeading=DegScale(RawHeadSum);
     }
     break;
case eIdle:
	 break;
    }
}
float DegScale(int32_t v)
{
float dv=v;
dv/=200;//Samples per sec
dv*=(250.0/32768.0);//rate
return -dv;
}


//HELPER FUNCTIONS FOR I2C COMMUNICATION

uint8_t IMU::I2CWriteReadBuffer(uint8_t address, uint8_t writeVal, uint8_t *buffer, int readLength)
{
    int status;

    if(readLength <= 0)
    {
        // Non valid length. Return state 10, "Read Length Not Supported"
        return 10;
    }

    status = I2CSendBuf(address,&writeVal, 1, false);

    if(status > I2C_MASTER_OK)
    {
        // I2C Bus is in an undesired state. Return the state
        return status;
    }
    status = I2CRestart(address, I2C_START_READ);
    if ((status > I2C_MASTER_OK) || (status == I2C_NEXT_WRITE_OK))
    {
        // I2C Bus is in an undesired state. Return the state
        return status;
    }

   status = I2CReadBuf(address,buffer, readLength, true);

    if((status >= I2C_NEXT_WRITE_OK) && (status <= I2C_MASTER_OK))
    {
        I2CResetPeripheral();

    }

    return status;
}


void IMU::WriteReg(uint8_t addr,uint8_t reg, uint8_t val)
{
uint8_t buf[10];
buf[0]=reg;
buf[1]=val;
I2CSendBuf(addr,buf, 2, true);
}

uint8_t IMU::ReadReg(uint8_t addr, uint8_t reg)
{

    uint8_t buf[10];
    I2CWriteReadBuffer(addr,reg,buf,1);
    return buf[0];
}

void IMU::ReadManyReg(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{
    I2CWriteReadBuffer(address,subAddress,dest,count);
}
