//
//  MAIN.h
//  
//
//  Created by Austin Burch on 1/2/16.
//
//

//#ifndef ____MAIN__
//#define ____MAIN__


//************************************************************
//#ifndef INCLUDE_FILE
//#define INCLUDE_FILE

#include <cstring>
#include <ctime>
#include <fcntl.h>
#include <fstream>
#include <iostream>
#include <linux/i2c.h>
#include <linux/i2c-dev.h> // destributed with i2c-tools
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>
#include <math.h>
#include <inttypes.h>  // uint8_t, etc..
//#include <bcm2835.h>
//#include <vector.h>
// added in IMU script:
#include <errno.h>



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
	int receive(unsigned char *readBuf, int length);
	int receive(unsigned char RegisterAddress, unsigned char *readBuf, int length);
	int send(unsigned char *writeBuf, int length);
	int send(unsigned char RegisterAddress, unsigned char *writeBuf, int length);
	int send(unsigned char value);
};

//*****************************************************************
// THAR Class
//*****************************************************************


#define THAR_FL 1 // oriented to begin at front left tire,
#define THAR_FR 2 // and count up clockwise
#define THAR_BR 3
#define THAR_BL 4

//------------------Thermal Array Sensor Variables Init------------------
/* Thermal Array variable notation is referenced as follows:
 * THAR -- first tag for Thermal Array
 
 * F -- Front
 * B -- Back
 * R -- Right
 * L -- Left
 * 4x16 array
 */

class THAR {
public:

    int FL[4][16];
    int FR[4][16];
    int BR[4][16];
    int BL[4][16];
    
    int getArray(int channel);
    int getValue(int channel, int pixelRow, int pixelColumn);
    bool fail();
private:
    i2cBus i2c;
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



//------------------IMU Variables Init------------------

/* Inertial Measurement Unit variable notation is referenced as follows:
 * IMU -- first tag for Inertial Measurement Unit variables
 
 * xAccel -- forward acceleration
 * yAccel -- lateral acceleration
 * zAccel -- verticle acceleration
 * xRot   -- Roll about x-axis
 * yRot   -- Pitch about y-axis
 * zRot   -- Yaw about z-axis
 * temp   -- Internal DAQ temperature
 */
class IMU {
public:

    int xAccel;
    int yAccel;
    int zAccel;
    int xRot;
    int yRot;
    int zRot;
    int temp;
    
    
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


//------------------Analog Sensor Variables Init------------------

/* Analog to Digital Convertor variable notation is referenced as follows:
 * ADC -- first tag for all analog variables
 
 * steerAngle  -- steeing angle of the steering wheel rotatry potentiometer
 * airSpeed    -- air speed from pitot tube
 
 * F -- Front
 * B -- Back
 * R -- Right
 * L -- Left
 * brakePressF -- Hydraulic pressure in front brake line
 * brakePressB -- Hydraulic pressure in back brake line
 
 * shockDispFR   -- Displacement in FRONT RIGHT linear potentiometer
 * shockDispFL   -- Displacement in FRONT LEFT linear potentiometer
 * shockDispBR   -- Displacement in BACK RIGHT linear potentiometer
 * shockDispBL   -- Displacement in BACK LEFT linear potentiometer
 */
class ADC {
public:
//    ADC();

    int steerAngle; // channel 1
    
    int airSpeed; // channel 2
    
    int brakePressF; // channel 3
    int brakePressB; // channel 4
    
    int shockDispFL; // channel 5
    int shockDispFR; // channel 6
    int shockDispBR; // channel 7
    int shockDispBL; // channel 8
    
    int getValueTest();
    int getValue(int channel);
    bool fail();
private:
    i2cBus i2c;
    bool error_flag;
};


//*****************************************************************
// STORAGE Class
//*****************************************************************

class STORAGE {
public:

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

//------------------RTC Variables Init------------------

#define DS3231_SLAVE_ADDR 0X68
#define DS3231_SECOND 0X00
#define DS3231_MINUTE 0X01
#define DS3231_HOUR 0X02
#define DS3231_WEEK 0X03
#define DS3231_DAY 0X04
#define DS3231_MONTH 0X05
#define DS3231_YEAR 0X06
#define DS3231_CONTROL 0x0E

#define DS3231_1Hz 0X00 // 1 Hz
#define DS3231_1kHz 0x08 // 1 kHz



/* Real Time Clock variable notation is referenced as follows:
 * RTC -- first tag for all real time clock variables
 
 * startTime   -- initial time stamp given by real time clock
 * elapsedTime -- elapsed time since initial time stamp
 * stampTime   -- time stamp of each reading
 * interval    -- interval between each sample
 */

class RTC {
public:
    
    int initCount;
    int interval;
    int count;
    int startTime;
    int elapsedTime;
    int timeStamp;
    
    int init(unsigned char frequency); // initialize RTC
    int getTimeStamp(); // initialize RTC
    int isr();  // interrupt service routine and count interrupts
private:
    i2cBus i2c;
};



//************************************************************
// ECU Class
//************************************************************

//------------------ECU Variables Init------------------

/* Engine Control Unit variable notation is referenced as follows:
 * ECU -- first tag for all engine control unit variables
 
 * engineTemp  -- engine temperature
 * waterTemp   -- water temperature
 * RPM         -- revolultions per minute
 * throttlePos -- throttle position
 * oilTemp     -- oil temperature
 * oilPress    -- oil pressure
 * lambda      -- air fuel ratio
 
 */
class ECU {
public:

    int engineTemp;
    int waterTemp;
    int rpm;
    int throttlePos;
    int oilTemp;
    int oilPress;
    int lambda;
    
    
private:
};






//#endif /* defined(____MAIN_1_1_16__) */
