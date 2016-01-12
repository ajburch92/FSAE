//
//  MAIN.h
//  
//
//  Created by Austin Burch on 1/2/16.
//
//

#ifndef ____MAIN__
#define ____MAIN__


//************************************************************
#ifndef INCLUDE_FILE
#define INCLUDE_FILE

#include <cstring>
#include <ctime>
#include <fcntl.h>
#include <fstream>
#include <iostream>
#include <linux/i2c.h>
#include <linux/i2c-dev.h> // destributed with i2c-tools
#include <linux/spi/spidev.h>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>
#include <math.h>
#include <bcm2835.h>
#include <vector.h>
// added in IMU script:
#include <errno.h>

//*****************************************************************
// THAR Class
//*****************************************************************


#define THAR_FL 1 // oriented to begin at front left tire,
#define THAR_FR 2 // and count up clockwise
#define THAR_BR 3
#define THAR_BL 4


class THAR {
public:
    THAR();
    int getArray(int channel);
    int getValue(int channel, int pixelRow, int pixelColumn);
    bool fail();
private:
    i2cBus i2cBus;
    bool error_flag;
};


//*****************************************************************
// IMU Class
//*****************************************************************

#define IMU_xAccel 0
#define IMU_yAccel 1
#define IMU_zAccel 2

#define IMU_xRot 3
#define IMU_yRot 4
#define IMU_zRot 5

#define IMU_temp 6

//I2C device address and registers' subaddress
#define LSM303_ACCEL_ADDR (0x32 >> 1)
#define LSM303_MAG_ADDR (0x3C >> 1)
#define LSM303_ACCEL_OUTPUT_RANGE 4
#define LSM303_CTRL_REG1_A 0x20 //XYZ enable
#define LSM303_CTRL_REG4_A 0x23 //resolution
#define LSM303_OUT_X_L_A 0X28
#define LSM303_OUT_X_H_A 0x29
#define LSM303_OUT_Y_L_A 0x2A
#define LSM303_OUT_Y_H_A 0x2B
#define LSM303_OUT_Z_L_A 0x2C
#define LSM303_OUT_Z_H_A 0x2D
#define LSM303_MR_REG_M 0x02 //
#define LSM303_OUT_X_H_M 0X03
#define LSM303_OUT_X_L_M 0x04
#define LSM303_OUT_Z_H_M 0x05
#define LSM303_OUT_Z_L_M 0x06
#define LSM303_OUT_Y_H_M 0x07
#define LSM303_OUT_Y_L_M 0x08
#define debug 0

class IMU {
public:
    int twos_compliment(int lower_bit, int higher_bit);
    int getAccelData();
    int getTimeStamp();
private:
    bool error_flag;
};



//*****************************************************************
// ADC Class
//*****************************************************************

#define ADC_ch1 1 //steering angle
#define ADC_ch2 2 // airSpeed
#define ADC_ch3 3// brakePressF
#define ADC_ch4 4// brakePressB
#define ADC_ch5 5// shockDispFL
#define ADC_ch6 6// shockDispFR
#define ADC_ch7 7// shockDispBR
#define ADC_ch8 8// shockDispBL


class ADC {
public:
    ADC();
    int getValue(int channel);
    bool fail();
private:
    i2cBus i2cBus;
    bool error_flag;
};


//*****************************************************************
// STORAGE Class
//*****************************************************************

class STORAGE {
public:
    STORAGE();
    int init();
    int saveData();
    bool fail();
private:
    // SD STORAGE DRIVER
    bool error_flag;
    
    
};


//*****************************************************************
// RTC Class
//*****************************************************************

class RTC {
public:
    RTC();
    int getTimeStamp(int channel);
    int getValue(int channel, int pixelRow, int pixelColumn);
    bool fail();
private:
    i2cBus i2cBus;
    bool error_flag;
};



//************************************************************
// ECU Class
//************************************************************

class ECU {
public:
private:
};



//************************************************************
// I2C Bus Class
//************************************************************
class i2cBus {
	bool error_flag;
	int slave_address;
	std::string devicefile;
	std::string ErrorMessage;
    int file;
    int errorMsg(std::string message);
    int open_file();
    void close_file();
	void init(std::string DeviceFile, int Address);
public:
	i2cBus();
	i2cBus(int Address);
	i2cBus(std::string DeviceFile, int Address);
	~i2cBus();
	bool fail();
	int setAddress(int Address);
	int getAddress();
	const char *getErrorMessage();
	int setDevicefile(std::string filename);
	int receive(unsigned char *RxBuf, int length);
	int receive(unsigned char RegisterAddress, unsigned char *RxBuf, int length);
	int send(unsigned char *TxBuf, int length);
	int send(unsigned char RegisterAddress, unsigned char *TxBuf, int length);
	int send(unsigned char value);
};



#endif /* defined(____MAIN_1_1_16__) */
