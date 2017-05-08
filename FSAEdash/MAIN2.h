#ifndef MAIN2_H
#define MAIN2_H



//  MAIN.h
//
//
//  Created by Austin Burch on 1/2/16.
//
//

//#ifndef ____MAIN__
//#define ____MAIN__

//#include "MAIN2.cpp"
//************************************************************
//#ifndef INCLUDE_FILE
//#define INCLUDE_FILE

#include <cstring>
#include <ctime>
#include <fcntl.h>
#include <fstream>
#include <iostream>

//#include <linux/i2c.h>
#include <linux/i2c-dev.h> // destributed with i2c-tools
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <time.h>
#include <termios.h>
#include <unistd.h>
#include <math.h>
#include <inttypes.h>  // uint8_t, etc..
//#include <bcm2835.h>
#include <vector>
// added in IMU script:
#include <errno.h>
#include <wiringPi.h>
#include <sr595.h>
using namespace std;


//************************************************************
// I2C Bus Class
//************************************************************

#define DEFAULTDEVICE "/dev/i2c-1"  //


struct Tuple {
    Tuple() {}

    Tuple(int num1, int num2){
        low = num1;
        high = num2;
    }
    int low;
    int high;
};


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
    int receive(unsigned char registerAddress, unsigned char *readBuf, int length);
    int receive(unsigned char *registerAddress, unsigned char *readBuf, int addLength, int bufLength);

    int send(unsigned char *writeBuf, int length);
    int send(unsigned char registerAddress, unsigned char *writeBuf, int length);
    int send(unsigned char value);
};

//************************************************************
// GPIO Bus Class
//************************************************************



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

// Accelerometer register addresses
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

// Gyro register addresses
#define L3GD20_GYRO_ADDRESS         0x6B        // Gyro Address 1101011
#define GYRO_REGISTER_WHO_AM_I      0x0F   // 11010100   r
#define GYRO_REGISTER_CTRL_REG1     0x20   // 00000111   rw
#define GYRO_REGISTER_CTRL_REG2     0x21   // 00000000   rw
#define GYRO_REGISTER_CTRL_REG3     0x22   // 00000000   rw
#define GYRO_REGISTER_CTRL_REG4     0x23   // 00000000   rw
#define GYRO_REGISTER_CTRL_REG5     0x24   // 00000000   rw
#define GYRO_REGISTER_REFERENCE     0x25   // 00000000   rw
#define GYRO_REGISTER_OUT_TEMP      0x26   //            r
#define GYRO_REGISTER_STATUS_REG    0x27   //            r
#define GYRO_REGISTER_OUT_X_L       0x28   //            r
#define GYRO_REGISTER_OUT_X_H       0x29   //            r
#define GYRO_REGISTER_OUT_Y_L       0x2A   //            r
#define GYRO_REGISTER_OUT_Y_H       0x2B   //            r
#define GYRO_REGISTER_OUT_Z_L       0x2C   //            r
#define GYRO_REGISTER_OUT_Z_H       0x2D   //            r

#define GYRO_SENSITIVITY_250DPS  (0.00875F)    // Roughly 22/256 for fixed point match
#define GYRO_SENSITIVITY_500DPS  (0.0175F)     // Roughly 45/256
#define GYRO_SENSITIVITY_2000DPS (0.070F)      // Roughly 18/256


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

//    int xAccel;
//    int yAccel;
//    int zAccel;
//    int xRot;
//    int yRot;
//    int zRot;
//    int temp;


    void setupAccel();
    void setupGyro();
    float getAccelValue(int channel);
    float getGyroValue(int channel);
    //int getTimeStamp();
private:
    bool error_flag;
    i2cBus i2c;
};



//*****************************************************************
// ADC Class
//*****************************************************************

#define ADC_steerAngle  1 //ADC 16 ch1
#define ADC_airSpeed    2 // ADC 16 ch2
#define ADC_brakePressF 3 // ADC 16 ch3
#define ADC_fuelLevel   4 // ADC 16 ch4
#define ADC_shockDispFL 5 // First ADC 12 ch1
#define ADC_shockDispFR 6 // First ADC 12 ch2
#define ADC_shockDispBR 7 // First ADC 12 ch3
#define ADC_shockDispBL 8 // First ADC 12 ch4
#define ADC_temp        9 // Second ADC 12 ch1
#define ADC_channel10   10// Second ADC 12 ch2
#define ADC_channel11   11// Second ADC 12 ch3
#define ADC_channel12   12// Second ADC 12 ch4

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
    unsigned char ADC_address;
    int getValueTest();
    float getValue(int channel);
    int getCurrentSensorValue();
    bool fail();
private:
    i2cBus i2c;
    bool error_flag;
};
//*****************************************************************
// Adafruit_INA219 Class
//*****************************************************************


/*===================================================================
 I2C ADDRESS/BITS
 ------------------------------------------------------------------*/
#define INA219_ADDRESS                         (0x40)    // 1000000 (A0+A1=GND)
#define INA219_READ                            (0x01)
/*=================================================================*/

/*==================================================================
 CONFIG REGISTER (R/W)
 ------------------------------------------------------------------*/
#define INA219_REG_CONFIG                      (0x00)
/*-----------------------------------------------------------------*/
#define INA219_CONFIG_RESET                    (0x8000)  // Reset Bit

#define INA219_CONFIG_BVOLTAGERANGE_MASK       (0x2000)  // Bus Voltage Range Mask
#define INA219_CONFIG_BVOLTAGERANGE_16V        (0x0000)  // 0-16V Range
#define INA219_CONFIG_BVOLTAGERANGE_32V        (0x2000)  // 0-32V Range

#define INA219_CONFIG_GAIN_MASK                (0x1800)  // Gain Mask
#define INA219_CONFIG_GAIN_1_40MV              (0x0000)  // Gain 1, 40mV Range
#define INA219_CONFIG_GAIN_2_80MV              (0x0800)  // Gain 2, 80mV Range
#define INA219_CONFIG_GAIN_4_160MV             (0x1000)  // Gain 4, 160mV Range
#define INA219_CONFIG_GAIN_8_320MV             (0x1800)  // Gain 8, 320mV Range

#define INA219_CONFIG_BADCRES_MASK             (0x0780)  // Bus ADC Resolution Mask
#define INA219_CONFIG_BADCRES_9BIT             (0x0080)  // 9-bit bus res = 0..511
#define INA219_CONFIG_BADCRES_10BIT            (0x0100)  // 10-bit bus res = 0..1023
#define INA219_CONFIG_BADCRES_11BIT            (0x0200)  // 11-bit bus res = 0..2047
#define INA219_CONFIG_BADCRES_12BIT            (0x0400)  // 12-bit bus res = 0..4097

#define INA219_CONFIG_SADCRES_MASK             (0x0078)  // Shunt ADC Resolution and Averaging Mask
#define INA219_CONFIG_SADCRES_9BIT_1S_84US     (0x0000)  // 1 x 9-bit shunt sample
#define INA219_CONFIG_SADCRES_10BIT_1S_148US   (0x0008)  // 1 x 10-bit shunt sample
#define INA219_CONFIG_SADCRES_11BIT_1S_276US   (0x0010)  // 1 x 11-bit shunt sample
#define INA219_CONFIG_SADCRES_12BIT_1S_532US   (0x0018)  // 1 x 12-bit shunt sample
#define INA219_CONFIG_SADCRES_12BIT_2S_1060US  (0x0048)	 // 2 x 12-bit shunt samples averaged together
#define INA219_CONFIG_SADCRES_12BIT_4S_2130US  (0x0050)  // 4 x 12-bit shunt samples averaged together
#define INA219_CONFIG_SADCRES_12BIT_8S_4260US  (0x0058)  // 8 x 12-bit shunt samples averaged together
#define INA219_CONFIG_SADCRES_12BIT_16S_8510US (0x0060)  // 16 x 12-bit shunt samples averaged together
#define INA219_CONFIG_SADCRES_12BIT_32S_17MS   (0x0068)  // 32 x 12-bit shunt samples averaged together
#define INA219_CONFIG_SADCRES_12BIT_64S_34MS   (0x0070)  // 64 x 12-bit shunt samples averaged together
#define INA219_CONFIG_SADCRES_12BIT_128S_69MS  (0x0078)  // 128 x 12-bit shunt samples averaged together

#define INA219_CONFIG_MODE_MASK                (0x0007)  // Operating Mode Mask
#define INA219_CONFIG_MODE_POWERDOWN           (0x0000)
#define INA219_CONFIG_MODE_SVOLT_TRIGGERED     (0x0001)
#define INA219_CONFIG_MODE_BVOLT_TRIGGERED     (0x0002)
#define INA219_CONFIG_MODE_SANDBVOLT_TRIGGERED (0x0003)
#define INA219_CONFIG_MODE_ADCOFF              (0x0004)
#define INA219_CONFIG_MODE_SVOLT_CONTINUOUS    (0x0005)
#define INA219_CONFIG_MODE_BVOLT_CONTINUOUS    (0x0006)
#define INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS (0x0007)
/*====================================================================*/

/*====================================================================
 SHUNT VOLTAGE REGISTER (R)
 -------------------------------------------------------------------*/
#define INA219_REG_SHUNTVOLTAGE                (0x01)
/*===================================================================*/

/*====================================================================
 BUS VOLTAGE REGISTER (R)
 --------------------------------------------------------------------*/
#define INA219_REG_BUSVOLTAGE                  (0x02)
/*===================================================================*/

/*====================================================================
 POWER REGISTER (R)
 --------------------------------------------------------------------*/
#define INA219_REG_POWER                       (0x03)
/*====================================================================*/

/*====================================================================
 CURRENT REGISTER (R)
 --------------------------------------------------------------------*/
#define INA219_REG_CURRENT                     (0x04)
/*===================================================================*/

/*===================================================================
 CALIBRATION REGISTER (R/W)
 ---------------------------------------------------------------------*/
#define INA219_REG_CALIBRATION                 (0x05)
/*====================================================================*/

class Adafruit_INA219{
public:
    Adafruit_INA219(uint8_t addr = INA219_ADDRESS);
    void begin(void);
    void begin(uint8_t addr);
    void setCalibration_32V_2A(void);
    void setCalibration_32V_1A(void);
    void setCalibration_16V_400mA(void);
    float getBusVoltage_V(void);
    float getShuntVoltage_mV(void);
    float getCurrent_mA(void);


    float getValue();
    void setup();

private:
    i2cBus i2c;
    uint8_t ina219_i2caddr;
    uint32_t ina219_calValue;
    // The following multipliers are used to convert raw current and power
    // values to mA and mW, taking into account the current config settings
    uint32_t ina219_currentDivider_mA;
    uint32_t ina219_powerDivider_mW;

    void wireWriteRegister(uint8_t reg, uint16_t value);
    void wireReadRegister(uint8_t reg, uint16_t *value);
    int16_t getBusVoltage_raw(void);
    int16_t getShuntVoltage_raw(void);
    int16_t getCurrent_raw(void);
};





//*****************************************************************
// STORAGE Class
//*****************************************************************
//
//class STORAGE {
//public:
//
//    int init();
//    int saveData();
//    bool fail();
//private:
//    // SD STORAGE DRIVER
//    bool error_flag;
//
//
//};


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

#define BUTTON_PIN      6 //define GPIO6 ... header 22



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
    float interval;
    int count3;
    int startTime[7];
    int elapsedTime;
    int timeStamp[7];

    void initSKW(unsigned char frequency); // initialize RTC
    int getTimeStamp(); // initialize RTC
    int isr();  // interrupt service routine and count interrupts
private:
    i2cBus i2c;
};
//*****************************************************************
// GPS Class
//*****************************************************************

//------------------GPS Variables Init------------------
#define GPS_BAUDRATE B9600 ///Baud rate : 115200
#define GPS_DEVICE "/dev/ttyAMA0"
#define GPS_BUFFER_SIZE 8
//=======================================

#define PMTK_SET_NMEA_UPDATE_100_MILLIHERTZ  "$PMTK220,10000*2F\r\n" // Once every 10 seconds, 100 millihertz.
#define PMTK_SET_NMEA_UPDATE_200_MILLIHERTZ  "$PMTK220,5000*1B"  // Once every 5 seconds, 200 millihertz.
#define PMTK_SET_NMEA_UPDATE_1HZ  "$PMTK220,1000*1F"
#define PMTK_SET_NMEA_UPDATE_5HZ  "$PMTK220,200*2C"
#define PMTK_SET_NMEA_UPDATE_10HZ "$PMTK220,100*2F\r\n"
// Position fix update rate commands.
#define PMTK_API_SET_FIX_CTL_100_MILLIHERTZ  "$PMTK300,10000,0,0,0,0*2C" // Once every 10 seconds, 100 millihertz.
#define PMTK_API_SET_FIX_CTL_200_MILLIHERTZ  "$PMTK300,5000,0,0,0,0*18"  // Once every 5 seconds, 200 millihertz.
#define PMTK_API_SET_FIX_CTL_1HZ  "$PMTK300,1000,0,0,0,0*1C\r\n"
#define PMTK_API_SET_FIX_CTL_5HZ  "$PMTK300,200,0,0,0,0*2F\r\n"
// Can't fix position faster than 5 times a second!

#define PMTK_API_SET_SBAS_ENABLED  "$PMTK313,1*0A\r\n"


#define PMTK_SET_BAUD_115200 "$$PMTK251,115200*3B\r\n"
#define PMTK_SET_BAUD_57600 "$PMTK251,57600*2C\r\n"
#define PMTK_SET_BAUD_9600 "$PMTK251,9600*17\r\n"

// turn on only the second sentence (GPRMC)
#define PMTK_SET_NMEA_OUTPUT_RMCONLY "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n"
// turn on GPRMC and GGA
#define PMTK_SET_NMEA_OUTPUT_RMCGGA "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
// turn on ALL THE DATA
#define PMTK_SET_NMEA_OUTPUT_ALLDATA "$PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
// turn off output
#define PMTK_SET_NMEA_OUTPUT_OFF "$PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"

// to generate your own sentences, check out the MTK command datasheet and use a checksum calculator
// such as the awesome http://www.hhhh.org/wiml/proj/nmeaxor.html

#define PMTK_LOCUS_STARTLOG  "$PMTK185,0*22"
#define PMTK_LOCUS_STOPLOG "$PMTK185,1*23"
#define PMTK_LOCUS_STARTSTOPACK "$PMTK001,185,3*3C"
#define PMTK_LOCUS_QUERY_STATUS "$PMTK183*38"
#define PMTK_LOCUS_ERASE_FLASH "$PMTK184,1*22"
#define LOCUS_OVERLAP 0
#define LOCUS_FULLSTOP 1

#define PMTK_ENABLE_SBAS "$PMTK313,1*2E"
#define PMTK_ENABLE_WAAS "$PMTK301,2*2E"

// standby command & boot successful message
#define PMTK_STANDBY "$PMTK161,0*28"
#define PMTK_STANDBY_SUCCESS "$PMTK001,161,3*36"  // Not needed currently
#define PMTK_AWAKE "$PMTK010,002*2D"

// ask for the release and version
#define PMTK_Q_RELEASE "$PMTK605*31"

// request for updates on antenna status
#define PGCMD_ANTENNA "$PGCMD,33,1*6C"
#define PGCMD_NOANTENNA "$PGCMD,33,0*6D"


/* Global Positioning System variable notation is referenced as follows:
 * GPS -- first tag for all real time clock variables

 * startTime   -- initial time stamp given by real time clock
 * elapsedTime -- elapsed time since initial time stamp
 * stampTime   -- time stamp of each reading
 * interval    -- interval between each sample
 */

class GPS {
    FILE *stream;
    int nFd;
    std::string storage;
    struct termios stNew;
    struct termios stOld;
    int nRet;
    int nRetw;
    char buf[GPS_BUFFER_SIZE];
    int recnum;
    i2cBus i2c;

public:
    unsigned char teensyAddress;
    int init(void);
    int storespeed(void);//store speed into vector
    int storedirection(void);//store direction into vector
    std::string getGPSRMC(void);//return GPSRMC sentence
    int getSpeed(std::string GPSRMC_sentence);//return speed for dash display
    std::string printlocation(std::string GPSRMC_sentence);//return longitude and latitude
    std::string printdate(std::string GPSRMC_sentence);//return date
    std::string printtimestamp(std::string GPSRMC_sentence);//return timestamp
    int getValue( unsigned char channel);
    //int getTimeStamp();

};

//************************************************************
// GPIO Class
//************************************************************

//------------------GPIO Pin List------------------
#define ISR_PIN      6 //define GPIO6
#define SHIFTREG_DATA_PIN      3 //define GPIO3, wiring pi number 3, header number 15
#define SHIFTREG_LATCH_PIN      1 //define GPIO1,wiring pi number 1,  header number 12 jumped on pcb
#define SHIFTREG_CLK_PIN      4 //define GPIO4,wiring pi number 4,  header number 16 jumped on pcb


//--------Wiring Pi documentation and function calls-------

/*
 for list of pinout see :

 https://projects.drogon.net/raspberry-pi/wiringpi/pins/

 for list of pin special use see:

 http://wiringpi.com/pins/special-pin-functions/


//----------------------------------------------------------
                            SETUP:
 //----------------------------------------------------------
     wiringPiSetup(void) ;

        This initialises the wiringPi system and assumes that the calling program is going to be using the wiringPi pin numbering scheme. This is a simplified numbering scheme which provides a mapping from virtual pin numbers 0 through 16 to the real underlying Broadcom GPIO pin numbers. See the pins page for a table which maps the wiringPi pin number to the Broadcom GPIO pin number to the physical location on the edge connector.

         This function needs to be called with root privileges.

 //----------------------------------------------------------
                        GENERAL FUNCTIONS:
 //----------------------------------------------------------
 void pinMode (int pin, int mode) ;

    This sets the mode of a pin to either INPUT, OUTPUT, or PWM_OUTPUT. Note that only wiringPi pin 1 (BCM_GPIO 18) supports PWM output. The pin number is the number obtained from the pins table.

    This function has no effect when in Sys mode.
//----------------------------------------------------------
 void digitalWrite (int pin, int value) ;

    Writes the value HIGH or LOW (1 or 0) to the given pin which must have been previously set as an output.

 //----------------------------------------------------------
 int digitalRead (int pin) ;

    This function returns the value read at the given pin. It will be HIGH or LOW (1 or 0) depending on the logic level at the pin.
 //----------------------------------------------------------
 void pullUpDnControl (int pin, int pud) ;

    This sets the pull-up or pull-down resistor mode on the given pin, which should be set as an input. Unlike the Arduino, the BCM2835 has both pull-up an down internal resistors. The parameter pud should be; PUD_OFF, (no pull up/down), PUD_DOWN (pull to ground) or PUD_UP (pull to 3.3v)

    This function has no effect when in Sys mode. If you need to activate a pull-up/pull-down, then you can do it with the gpio program in a script before you start your program.
 //----------------------------------------------------------






//#endif /* defined(____MAIN_1_1_16__) */


#endif // MAIN2_H
