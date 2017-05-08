// DATA AQUISITION CODE : Developed for Texas A&M Formula SAE Team 2016
// BY : Austin Burch Tong Lu Manuel Rodriguez
//

///////////////////////////////////////////////////////////////////////////////////
//      MAIN      //
///////////////////////////////////////////////////////////////////////////////////


//                              7
//                          +......7
//  ,............           7.. 7   I..
//   ...        7         I.7      :.777..,7
//  ...     77...: 7   7,7          7?....7    77...        7  7
//   7,.  7=77  777., 7               77....777  77 7 ?.:7  77+.......777
//    7. 7 7.777I.,7..7           .....................77   7.= 77 7...,..., 7
//     77 ?77     I.7.+7  77,.~7                          77.7.. 7......~   ....77
//       7        ..7. 777                               7~7.7     .....7      ...
//                 =.?.                       7  ?.......+ ~7        ....        7.~
//                7. .     7:.77777       7I.....  77   7.7         7....7      7..I
//      77     77.7.I           +.....~7777:.............. 77     7....7..........
//       7777  77..      77  7  77777II??++==~::,,......... .7,==.....7..........+
//             77                                             7....?7
/////////////////////////////////////////////////////////////////////////////////////
//
////////////////////////////// ///////////////////////////////////////////////////////



// tests
//  MAIN.cpp
//
//
//  Created by Austin Burch on 1/2/16.
//
//
//
//
//
//

#include "MAIN2.h"
//*******************************************************************
// INITIALIZATIONS
//*******************************************************************
//int count2 = 0;
//int countOverload = 0;
//
//struct ADCdata {
//    float steerAngle[1024]; // channel 1
//    float airSpeed[1024]; // channel 2
//    float brakePressB[1024]; // channel 3
//    float fuelTankLevel[1024]; // channel 4
//    float shockDispFL[1024]; // channel 5
//    float shockDispFR[1024]; // channel 6
//    float shockDispBR[1024]; // channel 7
//    float shockDispBL[1024]; // channel 8
//    float channel9[1024];
//    float channel10[1024];
//    float channel11[1024];
//    float channel12[1024];
//};
//
//struct INAdata {
//    float brakePressF[1024]; // current sensor
//};
//
//struct ECUdata {
//
//    short engineTemp[1024];
//    short waterTemp[1024];
//    short rpm[1024];
//    short throttlePos[1024];
//    short oilTemp[1024];
//    short oilPress[1024];
//    short lambda[1024];
//
//};
//
//struct RTCdata {
//
//    short interval[1024];
//    short RTCcount[1024];
//    short startTime[1024];// [7]?
//    short elapsedTime[1024];
//    short timeStamp[1024];//[7]
//
//};
//
//struct IMUdata {
//    float xAccel[1024];
//    float yAccel[1024];
//    float zAccel[1024];
//    float xRot[1024];
//    float yRot[1024];
//    float zRot[1024];
//    float temp[1024];
//
//};
//
//struct THARdata {
//
//};
//
//struct GPSdata {
//    //std::vector <std::string> GPSRMC_sentence; // 1024 strings with 56 char and a null terminator. ... use stirng copy ... strcpy(destination, sentance);
//    vector<string> GPS_sentenceVector;
//
//    //char GPSRMC[1024][56];
//    int speed[1024];
//
//};
//
//
//i2cBus i2c;
//RTC rtc;
//ADC adc;
//IMU imu;
//GPS gps;
//GPIO gpio;
//ADCdata ADCdata;
//ECUdata ECUdata;
//RTCdata RTCdata;
//IMUdata IMUdata;
//THARdata THARdata;
//GPSdata GPSdata;
//INAdata INAdata;
//STORAGE storage; //duplicate in the main.cpp file
//Adafruit_INA219 ina219;
//Dash_LED led;
////
//void DAQinterrupt ();

//Converting string repesenting a hexnumber to number
int hexstringToNumber(std::string str){
    std::stringstream  strin;
    int var;
    
    strin << std::hex << str;
    strin >> var;
    
    return var;
}
//Converting string to number
int stringToNumber(std::string str){
    std::stringstream  strin;
    int var;
    
    strin << str;
    strin >> var;
    
    return var;
}

//Converting a number to string
std::string numberToString(int num){
    std::ostringstream strout;
    std::string str;
    
    strout << num;
    str = strout.str();
    
    return str;
}


//*******************************************************************
//Functions for i2c Bus
//*******************************************************************

//------------------Constructor------------------
/*
 * initalizes the i2c bus. Sets the devicefile to "/dev/i2c-1"
 */

i2cBus::i2cBus()
{
    init(DEFAULTDEVICE, -1);
}

/*
 * initalizes the i2c bus. Sets the devicefile to "/dev/i2c-1"
 *
 * @param Address of new I2C slave adress
 */

i2cBus::i2cBus(int Address)
{
    init(DEFAULTDEVICE, Address);
}

/*
 * initalizes the i2c bus.
 *
 * @param Devicefile new I2C device file, e.g. "/dev/i2c-2"
 * @param Address of new I2C slave adress
 */
i2cBus::i2cBus(std::string Devicefile, int Address)
{
    init(Devicefile, Address);
}

//------------------Destructor------------------
/*
 * Closes the file handle
 */

i2cBus::~i2cBus()
{
    close_file();
}

//------------------Init------------------
/*
 * Called by the constructors to initialize class variables.
 *
 * @param Devicefile new I2C device file, e.g. "/dev/i2c-2"
 * @param Address new I2C slave adress
 */
void i2cBus::init(std::string Devicefile, int Address)
{
    devicefile=Devicefile;
    slave_address = Address;
    error_flag=false;
    file = 0;
}

//------------------error messaging------------------
/*
 * Called by the send and receive Methods when an Error occures
 *
 * @param message String contents that describe the error.
 * @return -1
 */
int i2cBus::errorMsg(std::string message)
{
    ErrorMessage=message;
    error_flag=true;
    close_file();
    return -1;
}

//------------------close file descriptor------------------
/*
 * Closes the file if open and resets the variable.
 */
void i2cBus::close_file()
{
    if (file) {
        close(file);
        file = 0;
    }
}

//------------------open file descriptor------------------
/*
 * Opens the devicefile. If a file is already open it is closed first.  A new file is opened
 * and io operations defined based on the class values for devicefile
 * and slave_address.
 *
 * @return success: 0, failure: -1
 */
int i2cBus::open_file()
{
    error_flag = false;
    
    if (file) {
        close_file();
        file = 0;
    }
    
    if (slave_address == -1) {
        return errorMsg("ERROR slave address is not set\n");
    }
    if ((file = open(devicefile.c_str(), O_RDWR)) < 0) {
        return errorMsg("ERROR opening: " + devicefile + "\n");
    }
    if (ioctl(file, I2C_SLAVE, slave_address) < 0) {
        return errorMsg("ERROR address: " + numberToString(slave_address) + "\n");
    }
    return 0;
}

//-------------------------------Fail-------------------------------
/*
 * returns the error flag to check if the last operation went wrong
 *
 * @return error_flag as boolean
 */
bool i2cBus::fail(){
    return error_flag;
}

//-------------set Address-------------
/*
 * Set the i2c slave address
 *
 * With this function you can set the individual I2C Slave-Address.
 * @param Address new I2C slave Address
 * @return failure: -1
 * I2C slave Address
 * @return failure: -1
 */
int i2cBus::setAddress(int Address){
    slave_address = Address;
    return open_file();
}

//-------------get Address-------------
/*
 * Get the i2c slave address
 *
 * With this function you can get the set Slave-Address.
 * @return Address I2C slave Address
 */
int i2cBus::getAddress(){
    return slave_address;
}

//-------------get Error Message-------------
/*
 * Get the last Error Message.
 *
 * This function returns the last Error Message, which occurred in that Class.
 * @return ErrorMessage as c-string
 */
const char *i2cBus::getErrorMessage(){
    return ErrorMessage.c_str();
}

//-------------------set devicefile----------------
/*
 * set i2c the device file.                 DEFAULT is "/dev/i2c-1"
 *
 * This function sets the devicefile you want to access. by default "/dev/i2c-1" is set.
 * @param filename path to the devicefile e.g. "/dev/i2c-0"
 * @return failure: -1
 */
int i2cBus::setDevicefile(std::string filename){
    devicefile = filename;
    return open_file();
}


//----------------------------------receive----------------------------------
/*
 * receive bytes from the I2C bus.
 *
 * This function reads "length" number of bytes from the i2c bus and stores them into the "readBuf". At success the function returns 1, on failure -1.<br>
 * @param readBuf Receive buffer. The read bytes will be stored in it.
 * @param length Amount of bytes that will be read.
 * @return success: 1, failure: -1
 */
int i2cBus::receive(unsigned char *readBuf, int length){
    
    if (readBuf == 0) {
        return errorMsg("Receive method received a null writeBuf pointer.\n");
    }
    if (length < 1) {
        return errorMsg("Receive method received an invalid buffer length.\n");
    }
    
    if (!file) {
        if (open_file() == -1) {
            return -1;
        }
    }
    
    error_flag=false;
    
    if (read(file, readBuf, length) != length) {
        return errorMsg("i2c read error! Address: " + numberToString(slave_address) + " dev file: " + devicefile + "\n");
    }
    
    return 1;
}

//----------------------------------receive----------------------------------
/*
 * receive bytes from the I2C bus.
 *
 * This function reads "length" number of bytes from the register "RegisterAddress" and stores them into the "readBuf". At success the function returns 1, on failure -1.
 * @param RegisterAddress Address of the register you want to read from
 * @param readBuf Receive buffer. The read bytes will be stored in it.
 * @param length Amount of bytes that will be read.
 * @return success: 1, failure: -1
 */
int i2cBus::receive(unsigned char registerAddress, unsigned char *readBuf, int length){
    
    if (readBuf == 0) {
        return errorMsg("Receive method received a null writeBuf pointer.\n");
    }
    if (length < 1) {
        return errorMsg("Receive method received an invalid buffer length.\n");
    }
    if (!file) {
        if (open_file() == -1) {
            return -1;
        }
    }
    
    error_flag=false;
    
    
    if (write(file, &registerAddress, 1) != 1) {
        return errorMsg("i2c write error!\n");
    }
    if (read(file, readBuf, length) != length) {
        return errorMsg("i2c read error! Address: " + numberToString(slave_address) + " dev file: " + devicefile + "\n");
    }
    
    return 1;
}

//----------------------------------receive----------------------------------
/*
 * receive bytes from the I2C bus.
 *
 * This function reads "length" number of bytes from the register "RegisterAddress" and stores them into the "readBuf". At success the function returns 1, on failure -1.
 * @param RegisterAddress Address of the register you want to read from
 * @param readBuf Receive buffer. The read bytes will be stored in it.
 * @param addLength Number of Addresses that will be called.
 * @param bufLength Amount of bytes that will be read.
 * @return success: 1, failure: -1
 */
int i2cBus::receive(unsigned char *registerAddress, unsigned char *readBuf, int addLength, int bufLength){
    int val[addLength];
    if (readBuf == 0) {
        return errorMsg("Receive method received a null writeBuf pointer.\n");
    }
    if (bufLength < 1) {
        return errorMsg("Receive method received an invalid buffer length.\n");
    }
    if (!file) {
        if (open_file() == -1) {
            return -1;
        }
    }
    
    error_flag=false;
    
    int i=0;
    while (i < addLength) {
        if (write(file, &registerAddress[i], 1) != 1) {
            return errorMsg("i2c write error!\n");
        }
        if (bufLength == 1) {
            val[i] = read(file, readBuf, bufLength);
        }
        else if (bufLength > 1 ) {
            val[i] = readBuf[0] << 8 | readBuf[1];
            
        }
        i++;
    }
    
    return 1;
}


//----------------------------------send----------------------------------
/*
 * send bytes to the I2C bus.
 *
 * This function sends "length" number of bytes from the "writeBuf" to the i2c bus. At success the function returns 1, on failure -1.
 * @param writeBuf Transmit buffer. The bytes you want to send are stored in it.
 * @param length Amount of bytes that will be send.
 * @return success: 1, failure: -1
 */
int i2cBus::send(unsigned char *writeBuf, int length){
    
    if (writeBuf == 0) {
        return errorMsg("Send method received a null writeBuf pointer.\n");
    }
    if (length < 1) {
        return errorMsg("Send method received an invalid buffer length.\n");
    }
    if (!file) {
        if (open_file() == -1) {
            return -1;
        }
    }
    
    error_flag=false;
    
    if(write(file, writeBuf, length) != length) {
        return errorMsg("i2c write error!\n");
    }
    return 1;
}

//----------------------------------send----------------------------------
/*
 * send bytes to the I2C bus.
 *
 * This function sends "length" number of bytes from the "writeBuf" to the register "RegisterAddress". At success the function returns 1, on failure -1.
 * @param RegisterAddress Address of the register you want to send the bytes to
 * @param writeBuf Transmit buffer. The bytes you want to send are stored in it.
 * @param length Amount of bytes that will be send.
 * @return success: 1, failure: -1
 */
int i2cBus::send(unsigned char registerAddress, unsigned char *writeBuf, int length){
    int i;
    unsigned char data[length+1];
    data[0]=registerAddress;
    
    for ( i = 0; i < length ; i++ ) {
        data[i+1] = writeBuf[i];
    }
    
    if (writeBuf == 0) {
        return errorMsg("Send method received a null writeBuf pointer.\n");
    }
    if (length < 1) {
        return errorMsg("Send method received an invalid buffer length.\n");
    }
    if (!file) {
        if (open_file() == -1) {
            return -1;
        }
    }
    
    error_flag=false;
    
    /*	if (send(RegisterAddress) == -1)
     return -1;
     */
    if(write(file, data, length+1) != length+1) {
        return errorMsg("i2c write error!\n");
    }
    return 1;
}

//----------------------------------send----------------------------------
/*
 * This function sends a byte to the I2C bus.
 *
 * @param value byte that will be send.
 * @return success: 1, failure: -1
 */
int i2cBus::send(unsigned char value){
    
    if (!file) {
        if (open_file() == -1) {
            return -1;
        }
    }
    error_flag=false;
    
    if(write(file, &value, 1) != 1) {
        return errorMsg("i2c write error!\n");
    }
    return 1;
}



//*******************************************************************
//Functions for ADC
//*******************************************************************

//---------------------- getValue() -----------------------

/**
 * Get a value of an ADC channel in reference to GND
 
 * @param channel Number of the ADC-channel (1-8), see below for references.
 
 * @return value
 */

float ADC::getValue(int channel) {
    
    int16_t val=0;
    float value;
    unsigned char readBuf[2];
    readBuf[0]= 0;
    readBuf[1]= 0;
    unsigned char writeBuf[3];
    
    
    if (channel < 9) {
        if (channel < 5) {
            ADC_address = 0x49;   // VDD Address of ADC 16-bit device on I2C bus
            
            
        }
        else {
            ADC_address = 0x48;   // GROUND Address of first ADC 12-bit device on I2C bus
        }
    }
    else {
        ADC_address = 0x4A;   // SDA Address of second ADC 12-bit device on I2C bus
        
        
    }
    i2c.setAddress(ADC_address);
    
    error_flag=false;
    
    switch (channel) {  // ADC Channel Selection
    
    // ---------------------- ADC 16-bit---------------------//
    
    case 1: // steerAngle
        writeBuf[0] = 1;          // This sets the pointer register so that the following two bytes write to the config register
        writeBuf[1] = 0xC1;       // This sets the 8 MSBs of the config register (bits 15-8) 11000011
        writeBuf[2] = 0xE3;       // This sets the 8 LSBs of the config register (bits 7-0) 11100011
        break; // channel 1
    case 2: // airSpeed
        writeBuf[0] = 1;          // This sets the pointer register so that the following two bytes write to the config register
        writeBuf[1] = 0xD1;       // This sets the 8 MSBs of the config register (bits 15-8) 11010011
        writeBuf[2] = 0xE3;       // This sets the 8 LSBs of the config register (bits 7-0) 11100011
        break; // channel 2
    case 3: // brakePressB    *** brakePressF will come from current sensor
        writeBuf[0] = 1;          // This sets the pointer register so that the following two bytes write to the config register
        writeBuf[1] = 0xE1;       // This sets the 8 MSBs of the config register (bits 15-8) 11100011
        writeBuf[2] = 0xE3;       // This sets the 8 LSBs of the config register (bits 7-0) 11100011
        break; // channel 3
    case 4: // fuel tank level
        writeBuf[0] = 1;          // This sets the pointer register so that the following two bytes write to the config register
        writeBuf[1] = 0xF1;       // This sets the 8 MSBs of the config register (bits 15-8) 11110011
        writeBuf[2] = 0xE3;       // This sets the 8 LSBs of the config register (bits 7-0) 11100011
        break; // channel 4
        
        // ---------------------- First ADC 12-bit---------------------//
    case 5: // shockDispFL
        writeBuf[0] = 1;          // This sets the pointer register so that the following two bytes write to the config register
        writeBuf[1] = 0xC1;       // This sets the 8 MSBs of the config register (bits 15-8) 11000011
        writeBuf[2] = 0xE3;       // This sets the 8 LSBs of the config register (bits 7-0) 11100011
        break;
        
    case 6: // shockDispFR
        writeBuf[0] = 1;          // This sets the pointer register so that the following two bytes write to the config register
        writeBuf[1] = 0xD1;       // This sets the 8 MSBs of the config register (bits 15-8) 11010011
        writeBuf[2] = 0xE3;       // This sets the 8 LSBs of the config register (bits 7-0) 11100011
        break;
        
    case 7: // shockDispBR
        writeBuf[0] = 1;          // This sets the pointer register so that the following two bytes write to the config register
        writeBuf[1] = 0xE1;       // This sets the 8 MSBs of the config register (bits 15-8) 11100011
        writeBuf[2] = 0xE3;       // This sets the 8 LSBs of the config register (bits 7-0) 11100011
        break;
        
    case 8: // shockDispBL
        writeBuf[0] = 1;          // This sets the pointer register so that the following two bytes write to the config register
        
        writeBuf[1] = 0xF1;       // This sets the 8 MSBs of the config register (bits 15-8) 11110011
        writeBuf[2] = 0xE3;       // This sets the 8 LSBs of the config register (bits 7-0) 11100011
        break;
        // ---------------------- Second ADC 12-bit---------------------//
        
    case 9: // channel 9 ADC_3_A0  DAQ temp
        writeBuf[0] = 1;          // This sets the pointer register so that the following two bytes write to the config register
        writeBuf[1] = 0xC1;       // This sets the 8 MSBs of the config register (bits 15-8) 11000011
        writeBuf[2] = 0xE3;       // This sets the 8 LSBs of the config register (bits 7-0) 11100011
        break;
        
    case 10: // channel 10 ADC_3_A1  EXTRA
        writeBuf[0] = 1;          // This sets the pointer register so that the following two bytes write to the config register
        writeBuf[1] = 0xD1;       // This sets the 8 MSBs of the config register (bits 15-8) 11010011
        writeBuf[2] = 0xE3;       // This sets the 8 LSBs of the config register (bits 7-0) 11100011
        
        break;
        
    case 11: // channel 11 ADC_3_A2  EXTRA
        writeBuf[0] = 1;          // This sets the pointer register so that the following two bytes write to the config register
        writeBuf[1] = 0xE1;       // This sets the 8 MSBs of the config register (bits 15-8) 11100011
        writeBuf[2] = 0xE3;       // This sets the 8 LSBs of the config register (bits 7-0) 11100011
        break;
        
    case 12: // channel 12 ADC_3_A3  EXTRA
        writeBuf[0] = 1;          // This sets the pointer register so that the following two bytes write to the config register
        
        writeBuf[1] = 0xF1;       // This sets the 8 MSBs of the config register (bits 15-8) 11110011
        writeBuf[2] = 0xE3;       // This sets the 8 LSBs of the config register (bits 7-0) 11100011
        break;
        
    default: error_flag = true; return -1; break; // channel selection out of ADC range
    }
    
    i2c.send(writeBuf, 3);
    //write(I2CFile, writeBuf, 3);
    
    for (int j=0; j>= 100; j++) {
        i2c.receive(readBuf, 2); //read the config register into readbuf
        if(readBuf[0] & 0x80 == 1) { // readBuf[0] contains 8 MSBs of config register, and with 10000000 to select bit 15
            j=100;
        }

    }
    
    writeBuf[0] = 0;                  // Set pointer register to 0 to read from the conversion register
    i2c.send(writeBuf, 1);
    //write(I2CFile, writeBuf, 1);
    
    i2c.receive(readBuf, 2);        // Read the contents of the conversion register into readBuf
    //read(I2CFile, readBuf, 2);
    
    val = readBuf[0] << 8 | readBuf[1];   // Combine the two bytes of readBuf into a single 16 bit result
    //printf("%i %f (V) \n",  val, (float)val*6.144/32767.0);
    //printf("%i %f (V) \n",  val, (float)val*4.096/32767.0);
    value= (float)val*6.144/32767.0;
    printf("%f (V) ",  value);
    
    
    if (i2c.fail()) {
        error_flag = true;
        return -1;
    }
    
    error_flag = false;
    return value;
}

//----------------- ADS1115 : (16-bit ADC) Setup -------------------
/*
 * Sets up programmable operating modes and writes to config register for the 16-bit ADC
 *
 * @param Devicefile new I2C device file, e.g. "/dev/i2c-2"
 * @param Address new I2C slave adress
 */

int ADC::getValueTest() {
    
    int ADS_address = 0x48;   // Address of our device on the I2C bus
    int I2CFile;
    
    uint8_t writeBuf[3];      // Buffer to store the 3 bytes that we write to the I2C device
    uint8_t readBuf[2];       // 2 byte buffer to store the data read from the I2C device
    
    int16_t val;              // Stores the 16 bit value of our ADC conversion
    
    I2CFile = open("/dev/i2c-1", O_RDWR);     // Open the I2C device
    
    ioctl(I2CFile, I2C_SLAVE, ADS_address);   // Specify the address of the I2C Slave to communicate with
    
    // These three bytes are written to the ADS1115 to set the config register and start a conversion
    writeBuf[0] = 1;          // This sets the pointer register so that the following two bytes write to the config register
    writeBuf[1] = 0xC3;       // This sets the 8 MSBs of the config register (bits 15-8) to 11000011
    writeBuf[2] = 0x03;       // This sets the 8 LSBs of the config register (bits 7-0) to 00000011
    
    // Initialize the buffer used to read data from the ADS1115 to 0
    readBuf[0]= 0;
    readBuf[1]= 0;
    
    // Write writeBuf to the ADS1115, the 3 specifies the number of bytes we are writing,
    // this begins a single conversion
    write(I2CFile, writeBuf, 3);
    
    // Wait for the conversion to complete, this requires bit 15 to change from 0->1
    while ((readBuf[0] & 0x80) == 0)  // readBuf[0] contains 8 MSBs of config register, AND with 10000000 to select bit 15
    {
        read(I2CFile, readBuf, 2);    // Read the config register into readBuf
    }
    
    writeBuf[0] = 0;                  // Set pointer register to 0 to read from the conversion register
    write(I2CFile, writeBuf, 1);
    
    read(I2CFile, readBuf, 2);        // Read the contents of the conversion register into readBuf
    
    val = readBuf[0] << 8 | readBuf[1];   // Combine the two bytes of readBuf into a single 16 bit result
    
    printf("%i %f (V) \n",  val, (float)val*4.096/32767.0);
    close(I2CFile);
    
    return val;
    
}




//-------------------------------Fail-------------------------------
/* returns the error flag to check if the last operation went wrong
 *
 * @return error_flag as boolean
 */
bool ADC::fail(){
    return error_flag;
}


//*******************************************************************
//Functions for current sensor
//*******************************************************************

//---------------------- getValue() -----------------------

/**
 * Get a value of an ADC channel in reference to GND
 * Uses 5V Power Source
 
 * @param channel Number of the ADC-channel (1-8), see below for references.
 
 * @return value
 
 
 
 void Adafruit_INA219::setCalibration_16V_400mA(void) {
 
 // Calibration which uses the highest precision for
 // current measurement (0.1mA), at the expense of
 // only supporting 16V at 400mA max. ///////////////////// i think this is reduce to .01 mA and 40mA max because of the 1 Ohm resister added to the breakout board.
 
 // VBUS_MAX = 16V
 // VSHUNT_MAX = 0.04          (Assumes Gain 1, 40mV)
 // RSHUNT = 1               (Resistor value in ohms)
 
 // 1. Determine max possible current
 // MaxPossible_I = VSHUNT_MAX / RSHUNT
 // MaxPossible_I = 0.04A
 
 // 2. Determine max expected current
 // MaxExpected_I = 0.04A
 
 // 3. Calculate possible range of LSBs (Min = 15-bit, Max = 12-bit)
 // MinimumLSB = MaxExpected_I/32767
 // MinimumLSB = 0.00000122              (1.2uA per bit)
 // MaximumLSB = MaxExpected_I/4096
 // MaximumLSB = 0.00000977              (9.8uA per bit)
 
 // 4. Choose an LSB between the min and max values
 //    (Preferrably a roundish number close to MinLSB)
 // CurrentLSB = 0.000005 (5.0uA per bit)
 
 // 5. Compute the calibration register
 // Cal = trunc (0.04096 / (Current_LSB * RSHUNT))
 // Cal = 8192 (0x2000)
 
 ina219_calValue = 8192;
 
 // 6. Calculate the power LSB
 // PowerLSB = 20 * CurrentLSB
 // PowerLSB = 0.001 (1mW per bit)
 
 // 7. Compute the maximum current and shunt voltage values before overflow
 //
 // Max_Current = Current_LSB * 32767
 // Max_Current = 1.63835A before overflow
 //
 // If Max_Current > Max_Possible_I then
 //    Max_Current_Before_Overflow = MaxPossible_I
 // Else
 //    Max_Current_Before_Overflow = Max_Current
 // End If
 //
 // Max_Current_Before_Overflow = MaxPossible_I
 // Max_Current_Before_Overflow = 0.4
 //
 // Max_ShuntVoltage = Max_Current_Before_Overflow * RSHUNT
 // Max_ShuntVoltage = 0.04V
 //
 // If Max_ShuntVoltage >= VSHUNT_MAX
 //    Max_ShuntVoltage_Before_Overflow = VSHUNT_MAX
 // Else
 //    Max_ShuntVoltage_Before_Overflow = Max_ShuntVoltage
 // End If
 //
 // Max_ShuntVoltage_Before_Overflow = VSHUNT_MAX
 // Max_ShuntVoltage_Before_Overflow = 0.04V
 
 // 8. Compute the Maximum Power
 // MaximumPower = Max_Current_Before_Overflow * VBUS_MAX
 // MaximumPower = 0.4 * 16V
 // MaximumPower = 6.4W
 
 // Set multipliers to convert raw current/power values
 ina219_currentDivider_mA = 20;  // Current LSB = 50uA per bit (1000/50 = 20)
 ina219_powerDivider_mW = 1;     // Power LSB = 1mW per bit
 
 // Set Calibration register to 'Cal' calculated above
 wireWriteRegister(INA219_REG_CALIBRATION, ina219_calValue);
 
 // Set Config register to take into ac the settings above
 uint16_t config = INA219_CONFIG_BVOLTAGERANGE_16V |
 INA219_CONFIG_GAIN_1_40MV |
 INA219_CONFIG_BADCRES_12BIT |
 INA219_CONFIG_SADCRES_12BIT_1S_532US |
 INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;
 wireWriteRegister(INA219_REG_CONFIG, config);
 }
 */

/**************************************************************************/
/*!
 @brief  Sends a single command byte over I2C
 */
/**************************************************************************/
void Adafruit_INA219::wireWriteRegister (uint8_t reg, uint16_t value)
{
    unsigned char writeBuf[3];
    i2c.setAddress(ina219_i2caddr);
    writeBuf[0] = reg;
    writeBuf[1] = value >> 8;
    writeBuf[2] = value & 0xFF;
    i2c.send(writeBuf, 3);
    
}

/**************************************************************************/
/*!
 @brief  Reads a 16 bit values over I2C
 */
/**************************************************************************/
void Adafruit_INA219::wireReadRegister(uint8_t reg, uint16_t *value)
{
    unsigned char writeBuf[1];
    unsigned char readBuf[2];
    writeBuf[0] = 0;
    i2c.send(writeBuf, 1);
    i2c.receive(readBuf, 2);
    
    
    readBuf[0]= 0;
    readBuf[1]= 0;
    
    i2c.setAddress(ina219_i2caddr);
    writeBuf[0] = reg;
    i2c.send(writeBuf, 1);
    //while ((readBuf[0] & 0x80) == 0)
    //{
    //   i2c.receive(readBuf, 2);
    //}
    delay(1); // Max 12-bit conversion time is 586us per sample
    i2c.receive(readBuf,2);
    *value = ((readBuf[0] << 8) | readBuf[1]);
}

/**************************************************************************/
/*!
 @brief  Configures to INA219 to be able to measure up to 32V and 2A
 of current.  Each unit of current corresponds to 100uA, and
 each unit of power corresponds to 2mW. Counter overflow
 occurs at 3.2A.
 
 @note   These calculations assume a 0.1 ohm resistor is present
 */
/**************************************************************************/
void Adafruit_INA219::setCalibration_32V_2A(void)
{
    // By default we use a pretty huge range for the input voltage,
    // which probably isn't the most appropriate choice for system
    // that don't use a lot of power.  But all of the calculations
    // are shown below if you want to change the settings.  You will
    // also need to change any relevant register settings, such as
    // setting the VBUS_MAX to 16V instead of 32V, etc.
    
    // VBUS_MAX = 32V             (Assumes 32V, can also be set to 16V)
    // VSHUNT_MAX = 0.32          (Assumes Gain 8, 320mV, can also be 0.16, 0.08, 0.04)
    // RSHUNT = 0.1               (Resistor value in ohms)
    
    // 1. Determine max possible current
    // MaxPossible_I = VSHUNT_MAX / RSHUNT
    // MaxPossible_I = 3.2A
    
    // 2. Determine max expected current
    // MaxExpected_I = 2.0A
    
    // 3. Calculate possible range of LSBs (Min = 15-bit, Max = 12-bit)
    // MinimumLSB = MaxExpected_I/32767
    // MinimumLSB = 0.000061              (61uA per bit)
    // MaximumLSB = MaxExpected_I/4096
    // MaximumLSB = 0,000488              (488uA per bit)
    
    // 4. Choose an LSB between the min and max values
    //    (Preferrably a roundish number close to MinLSB)
    // CurrentLSB = 0.0001 (100uA per bit)
    
    // 5. Compute the calibration register
    // Cal = trunc (0.04096 / (Current_LSB * RSHUNT))
    // Cal = 4096 (0x1000)
    
    ina219_calValue = 4096;
    
    // 6. Calculate the power LSB
    // PowerLSB = 20 * CurrentLSB
    // PowerLSB = 0.002 (2mW per bit)
    
    // 7. Compute the maximum current and shunt voltage values before overflow
    //
    // Max_Current = Current_LSB * 32767
    // Max_Current = 3.2767A before overflow
    //
    // If Max_Current > Max_Possible_I then
    //    Max_Current_Before_Overflow = MaxPossible_I
    // Else
    //    Max_Current_Before_Overflow = Max_Current
    // End If
    //
    // Max_ShuntVoltage = Max_Current_Before_Overflow * RSHUNT
    // Max_ShuntVoltage = 0.32V
    //
    // If Max_ShuntVoltage >= VSHUNT_MAX
    //    Max_ShuntVoltage_Before_Overflow = VSHUNT_MAX
    // Else
    //    Max_ShuntVoltage_Before_Overflow = Max_ShuntVoltage
    // End If
    
    // 8. Compute the Maximum Power
    // MaximumPower = Max_Current_Before_Overflow * VBUS_MAX
    // MaximumPower = 3.2 * 32V
    // MaximumPower = 102.4W
    
    // Set multipliers to convert raw current/power values
    ina219_currentDivider_mA = 10;  // Current LSB = 100uA per bit (1000/100 = 10)
    ina219_powerDivider_mW = 2;     // Power LSB = 1mW per bit (2/1)
    
    // Set Calibration register to 'Cal' calculated above
    wireWriteRegister(INA219_REG_CALIBRATION, ina219_calValue);
    
    // Set Config register to take into account the settings above
    uint16_t config = INA219_CONFIG_BVOLTAGERANGE_32V |
            INA219_CONFIG_GAIN_8_320MV |
            INA219_CONFIG_BADCRES_12BIT |
            INA219_CONFIG_SADCRES_12BIT_1S_532US |
            INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;
    wireWriteRegister(INA219_REG_CONFIG, config);
}

/**************************************************************************/
/*!
 @brief  Configures to INA219 to be able to measure up to 32V and 1A
 of current.  Each unit of current corresponds to 40uA, and each
 unit of power corresponds to 800�W. Counter overflow occurs at
 1.3A.
 
 @note   These calculations assume a 0.1 ohm resistor is present
 */
/**************************************************************************/
void Adafruit_INA219::setCalibration_32V_1A(void)
{
    // By default we use a pretty huge range for the input voltage,
    // which probably isn't the most appropriate choice for system
    // that don't use a lot of power.  But all of the calculations
    // are shown below if you want to change the settings.  You will
    // also need to change any relevant register settings, such as
    // setting the VBUS_MAX to 16V instead of 32V, etc.
    
    // VBUS_MAX = 32V		(Assumes 32V, can also be set to 16V)
    // VSHUNT_MAX = 0.32	(Assumes Gain 8, 320mV, can also be 0.16, 0.08, 0.04)
    // RSHUNT = 0.1			(Resistor value in ohms)
    
    // 1. Determine max possible current
    // MaxPossible_I = VSHUNT_MAX / RSHUNT
    // MaxPossible_I = 3.2A
    
    // 2. Determine max expected current
    // MaxExpected_I = 1.0A
    
    // 3. Calculate possible range of LSBs (Min = 15-bit, Max = 12-bit)
    // MinimumLSB = MaxExpected_I/32767
    // MinimumLSB = 0.0000305             (30.5�A per bit)
    // MaximumLSB = MaxExpected_I/4096
    // MaximumLSB = 0.000244              (244�A per bit)
    
    // 4. Choose an LSB between the min and max values
    //    (Preferrably a roundish number close to MinLSB)
    // CurrentLSB = 0.0000400 (40�A per bit)
    
    // 5. Compute the calibration register
    // Cal = trunc (0.04096 / (Current_LSB * RSHUNT))
    // Cal = 10240 (0x2800)
    
    ina219_calValue = 10240;
    
    // 6. Calculate the power LSB
    // PowerLSB = 20 * CurrentLSB
    // PowerLSB = 0.0008 (800�W per bit)
    
    // 7. Compute the maximum current and shunt voltage values before overflow
    //
    // Max_Current = Current_LSB * 32767
    // Max_Current = 1.31068A before overflow
    //
    // If Max_Current > Max_Possible_I then
    //    Max_Current_Before_Overflow = MaxPossible_I
    // Else
    //    Max_Current_Before_Overflow = Max_Current
    // End If
    //
    // ... In this case, we're good though since Max_Current is less than MaxPossible_I
    //
    // Max_ShuntVoltage = Max_Current_Before_Overflow * RSHUNT
    // Max_ShuntVoltage = 0.131068V
    //
    // If Max_ShuntVoltage >= VSHUNT_MAX
    //    Max_ShuntVoltage_Before_Overflow = VSHUNT_MAX
    // Else
    //    Max_ShuntVoltage_Before_Overflow = Max_ShuntVoltage
    // End If
    
    // 8. Compute the Maximum Power
    // MaximumPower = Max_Current_Before_Overflow * VBUS_MAX
    // MaximumPower = 1.31068 * 32V
    // MaximumPower = 41.94176W
    
    // Set multipliers to convert raw current/power values
    ina219_currentDivider_mA = 25;      // Current LSB = 40uA per bit (1000/40 = 25)
    ina219_powerDivider_mW = 1;         // Power LSB = 800�W per bit
    
    // Set Calibration register to 'Cal' calculated above
    wireWriteRegister(INA219_REG_CALIBRATION, ina219_calValue);
    
    // Set Config register to take into account the settings above
    uint16_t config = INA219_CONFIG_BVOLTAGERANGE_32V |
            INA219_CONFIG_GAIN_8_320MV |
            INA219_CONFIG_BADCRES_12BIT |
            INA219_CONFIG_SADCRES_12BIT_1S_532US |
            INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;
    wireWriteRegister(INA219_REG_CONFIG, config);
}

void Adafruit_INA219::setCalibration_16V_400mA(void) {
    
    // Calibration which uses the highest precision for
    // current measurement (0.1mA), at the expense of
    // only supporting 16V at 400mA max.
    
    // VBUS_MAX = 16V
    // VSHUNT_MAX = 0.04          (Assumes Gain 1, 40mV)
    // RSHUNT = 0.1               (Resistor value in ohms)
    
    // 1. Determine max possible current
    // MaxPossible_I = VSHUNT_MAX / RSHUNT
    // MaxPossible_I = 0.4A
    
    // 2. Determine max expected current
    // MaxExpected_I = 0.4A
    
    // 3. Calculate possible range of LSBs (Min = 15-bit, Max = 12-bit)
    // MinimumLSB = MaxExpected_I/32767
    // MinimumLSB = 0.0000122              (12uA per bit)
    // MaximumLSB = MaxExpected_I/4096
    // MaximumLSB = 0.0000977              (98uA per bit)
    
    // 4. Choose an LSB between the min and max values
    //    (Preferrably a roundish number close to MinLSB)
    // CurrentLSB = 0.00005 (50uA per bit)
    
    // 5. Compute the calibration register
    // Cal = trunc (0.04096 / (Current_LSB * RSHUNT))
    // Cal = 8192 (0x2000)
    
    ina219_calValue = 8192;
    
    // 6. Calculate the power LSB
    // PowerLSB = 20 * CurrentLSB
    // PowerLSB = 0.001 (1mW per bit)
    
    // 7. Compute the maximum current and shunt voltage values before overflow
    //
    // Max_Current = Current_LSB * 32767
    // Max_Current = 1.63835A before overflow
    //
    // If Max_Current > Max_Possible_I then
    //    Max_Current_Before_Overflow = MaxPossible_I
    // Else
    //    Max_Current_Before_Overflow = Max_Current
    // End If
    //
    // Max_Current_Before_Overflow = MaxPossible_I
    // Max_Current_Before_Overflow = 0.4
    //
    // Max_ShuntVoltage = Max_Current_Before_Overflow * RSHUNT
    // Max_ShuntVoltage = 0.04V
    //
    // If Max_ShuntVoltage >= VSHUNT_MAX
    //    Max_ShuntVoltage_Before_Overflow = VSHUNT_MAX
    // Else
    //    Max_ShuntVoltage_Before_Overflow = Max_ShuntVoltage
    // End If
    //
    // Max_ShuntVoltage_Before_Overflow = VSHUNT_MAX
    // Max_ShuntVoltage_Before_Overflow = 0.04V
    
    // 8. Compute the Maximum Power
    // MaximumPower = Max_Current_Before_Overflow * VBUS_MAX
    // MaximumPower = 0.4 * 16V
    // MaximumPower = 6.4W
    
    // Set multipliers to convert raw current/power values
    ina219_currentDivider_mA = 20;  // Current LSB = 50uA per bit (1000/50 = 20)
    ina219_powerDivider_mW = 1;     // Power LSB = 1mW per bit
    
    // Set Calibration register to 'Cal' calculated above
    wireWriteRegister(INA219_REG_CALIBRATION, ina219_calValue);
    
    // Set Config register to take into account the settings above
    uint16_t config = INA219_CONFIG_BVOLTAGERANGE_16V |
            INA219_CONFIG_GAIN_1_40MV |
            INA219_CONFIG_BADCRES_12BIT |
            INA219_CONFIG_SADCRES_12BIT_1S_532US |
            INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;
    wireWriteRegister(INA219_REG_CONFIG, config);
}

/**************************************************************************/
/*!
 @brief  Instantiates a new INA219 class
 */
/**************************************************************************/
Adafruit_INA219::Adafruit_INA219(uint8_t addr) {
    ina219_i2caddr = addr;
    ina219_currentDivider_mA = 0;
    ina219_powerDivider_mW = 0;
}

/**************************************************************************/
/*!
 @brief  Setups the HW (defaults to 32V and 2A for calibration values)
 */
/**************************************************************************/
void Adafruit_INA219::begin(uint8_t addr) {
    ina219_i2caddr = addr;
    begin();
}

void Adafruit_INA219::begin(void) {
    i2c.setAddress(ina219_i2caddr);
    //Wire.begin();
    // Set chip to large range config values to start
    setCalibration_32V_2A();
}

/**************************************************************************/
/*!
 @brief  Gets the raw bus voltage (16-bit signed integer, so +-32767)
 */
/**************************************************************************/
int16_t Adafruit_INA219::getBusVoltage_raw() {
    uint16_t value;
    wireReadRegister(INA219_REG_BUSVOLTAGE, &value);
    
    // Shift to the right 3 to drop CNVR and OVF and multiply by LSB
    return (int16_t)((value >> 3) * 4);
}

/**************************************************************************/
/*!
 @brief  Gets the raw shunt voltage (16-bit signed integer, so +-32767)
 */
/**************************************************************************/
int16_t Adafruit_INA219::getShuntVoltage_raw() {
    uint16_t value;
    wireReadRegister(INA219_REG_SHUNTVOLTAGE, &value);
    return (int16_t)value;
}

/**************************************************************************/
/*!
 @brief  Gets the raw current value (16-bit signed integer, so +-32767)
 */
/**************************************************************************/
int16_t Adafruit_INA219::getCurrent_raw() {
    uint16_t value;
    
    // Sometimes a sharp load will reset the INA219, which will
    // reset the cal register, meaning CURRENT and POWER will
    // not be available ... avoid this by always setting a cal
    // value even if it's an unfortunate extra step
    wireWriteRegister(INA219_REG_CALIBRATION, ina219_calValue);
    
    // Now we can safely read the CURRENT register!
    wireReadRegister(INA219_REG_CURRENT, &value);
    
    return (int16_t)value;
}

/**************************************************************************/
/*!
 @brief  Gets the shunt voltage in mV (so +-327mV)
 */
/**************************************************************************/
float Adafruit_INA219::getShuntVoltage_mV() {
    int16_t value;
    value = getShuntVoltage_raw();
    return value * 0.01;
}

/**************************************************************************/
/*!
 @brief  Gets the shunt voltage in volts
 */
/**************************************************************************/
float Adafruit_INA219::getBusVoltage_V() {
    int16_t value = getBusVoltage_raw();
    return value * 0.001;
}

/**************************************************************************/
/*!
 @brief  Gets the current value in mA, taking into account the
 config settings and current LSB
 */
/**************************************************************************/
float Adafruit_INA219::getCurrent_mA() {
    float valueDec = getCurrent_raw();
    valueDec /= ina219_currentDivider_mA;
    return valueDec;
    
}

//----------------- INA219 : setup() -------------------
/*
 */
void Adafruit_INA219::setup() {
    
    begin();
    // To use a slightly lower 32V, 1A range (higher precision on amps):
    //ina219.setCalibration_32V_1A();
    // Or to use a lower 16V, 400mA range (higher precision on volts and amps):
    setCalibration_16V_400mA();  // make calibration adjustments for the change of resistor
    
    
}
//----------------- INA219 : getValue() -------------------
/*
 */

float Adafruit_INA219::getValue() {
    
    
    
    
    //float shuntvoltage = 0;
    //float busvoltage = 0;
    //float loadvoltage = 0;
    
    float current_mA = 0;
    
    //shuntvoltage = ina219.getShuntVoltage_mV();
    //busvoltage = ina219.getBusVoltage_V();
    //loadvoltage = busvoltage + (shuntvoltage / 1000);
    
    current_mA = getCurrent_mA();
    printf("getValue : %f   ",  current_mA);
    return current_mA;
}



//*******************************************************************
//Functions for THARs
//*******************************************************************


//---------------------- getArray() -----------------------

/* Get an Array of one Thermal Array Sensor
  
 * @param channel Number of the Thermal Array Sensor
 * @return value
 */

int THAR::getArray(int channel) {
    
    int command;
    int value=0;
    unsigned char rx_buf[64];
    int pixel [4][16];
    int pixelValue [4][16];
    error_flag=false;
    
    switch (channel) {  // THAR Channel Selection
    case 1: command = 0x01; break; // channel 1
        
    case 2: command = 0x02; break; // channel 2
        
    case 3: command = 0x03; break; // channel 3
        
    case 4: command = 0x04; break; // channel 4
        
    default: error_flag = true; return -1; break; // channel selection out of ADC range
    }
    
    
    // do i need to send device address first???????????
    i2c.send(command);
    if (i2c.fail()) {
        error_flag = true;
        return -1;
    }
    for (int pixelRow=1; pixelRow<5; pixelRow ++) {
        for (int pixelColumn=1; pixelColumn<17; pixelColumn ++) {
            
            int pixelAddress = pixel[pixelRow][pixelColumn]; // some sort of conversion/reference to address
            //pixelAddress = [0x01];                       // hex address
            
            i2c.send(pixelAddress);
            if (i2c.fail()) {
                error_flag = true;
                return -1;
            }
            
            if(i2c.receive(rx_buf, 1)>0){
                value = rx_buf[0];
                pixelValue[pixelRow][pixelColumn] = value;
                
                
            }
            else{
                error_flag=true;
                return -1;
            }
            
            if (i2c.fail()) {
                error_flag = true;
                return -1;
            }
            error_flag = false;
            return value;
            
        }
    }
}


//---------------------- getValue() -----------------------

/* Get a value of one element from a selected Thermal Array Sensor
  
 * @param channel Number of the ADC-channel (1-8)
 * @return value
 */

int THAR::getValue(int channel, int pixelRow, int pixelColumn) {
    int command;
    int value=0;
    unsigned char rx_buf[64];
    int pixel [4][16];
    int pixelValue [4][16];
    error_flag=false;
    
    switch (channel) {  // ADC Channel Selection
    case 1: command = 0x01; break; // channel 1
        
    case 2: command = 0x02; break; // channel 2
        
    case 3: command = 0x03; break; // channel 3
        
    case 4: command = 0x04; break; // channel 4
        
    default: error_flag = true; return -1; break; // channel selection out of ADC range
    }
    
    int pixelAddress = pixel[pixelRow][pixelColumn]; // some sort of conversion/reference to address
    //pixelAddress = [0x01];                       // hex address
    
    i2c.send(pixelAddress);
    if (i2c.fail()) {
        error_flag = true;
        return -1;
    }
    
    if(i2c.receive(rx_buf, 1)>0){
        value = rx_buf[0];
        pixelValue[pixelRow][pixelColumn] = value;
        
        
    }
    
    else{
        error_flag=true;
        return -1;
    }
    
    if (i2c.fail()) {
        error_flag = true;
        return -1;
    }
    error_flag = false;
    return value;
}




//-------------------------------Fail-------------------------------
/* returns the error flag to check if the last operation went wrong
 *
 * @return error_flag as boolean
 */
bool THAR::fail(){
    return error_flag;
}


//*******************************************************************
//Functions for RTC
//*******************************************************************

//---------------------- init() -----------------------

/* returns the error flag to check if the last operation went wrong
 *
 * @return error_flag as boolean
 */

void RTC::initSKW(unsigned char frequency) {
    
    unsigned char writeBuf[2];
    //interval = 1/frequency;
    initCount = 0;
    count3 = 0;
    writeBuf[0] = DS3231_CONTROL;
    writeBuf[1] = frequency;
    i2c.setAddress(DS3231_SLAVE_ADDR);
    i2c.send(writeBuf, 2);
    /*
     int fd;
     fd = open("/dev/i2c-1", O_RDWR);
     
     if (fd < 0) {
     printf("Error opening file: %s\n", strerror(errno));
     return 1;
     }
     
     // RTC Code:
     if (ioctl(fd, I2C_SLAVE, DS3231_SLAVE_ADDR) < 0) {
     printf("ioctl error: %s\n", strerror(errno));
     return 1;
     }
     */
    printf("swk init\n");
    
}


//---------------------- getTimeStamp() -----------------------

/* returns the error flag to check if the last operation went wrong
 *
 * @return error_flag as boolean
 */

int RTC::getTimeStamp() {
    
    unsigned char registerAddress[7] = {DS3231_YEAR, DS3231_MONTH, DS3231_DAY, DS3231_WEEK, DS3231_HOUR, DS3231_MINUTE, DS3231_SECOND};
    unsigned char readBuf[1] = {0};
    
    i2c.setAddress(DS3231_SLAVE_ADDR);
    
    //readBuf[6] = 0x00;
    //readBuf[5] = DS3231_MINUTE;
    //readBuf[4] = DS3231_HOUR;
    //readBuf[3] = DS3231_WEEK;
    //readBuf[2] = DS3231_DAY;
    //readBuf[1] = DS3231_SECOND;
    //readBuf[0] = DS3231_SECOND;
    
    /*
     int fd;
     fd = open("/dev/i2c-1", O_RDWR);
     
     if (fd < 0) {
     printf("Error opening file: %s\n", strerror(errno));
     return 1;
     }
     
     // RTC Code:
     if (ioctl(fd, I2C_SLAVE, DS3231_SLAVE_ADDR) < 0) {
     printf("ioctl error: %s\n", strerror(errno));
     return 1;
     }
     
     */
    
    if (count3 = 0) {
        startTime[6] = i2c.receive(registerAddress,readBuf,7,1) & 0x7F;       // year
        printf("%x\t%x\t%x\t%x\t%x\t%x\t%x\n",startTime[0], startTime[1], startTime[2], startTime[3],startTime[4], startTime[5], startTime[6]);
        return 0;
    }
    
    else {
        timeStamp[6] = i2c.receive(registerAddress[6],readBuf,1) & 0x7F; // second
        timeStamp[5] = i2c.receive(registerAddress[5],readBuf,1) & 0x7F; // minute
        timeStamp[4] = i2c.receive(registerAddress[4],readBuf,1) & 0x3F; // hour
        timeStamp[3] = i2c.receive(registerAddress[3],readBuf,1) & 0x07; // week
        timeStamp[2] = i2c.receive(registerAddress[2],readBuf,1) & 0x3F; // date
        timeStamp[1] = i2c.receive(registerAddress[1],readBuf,1) & 0x1F; // month
        timeStamp[0] = i2c.receive(registerAddress[0],readBuf,1);        // year
        printf("%x\t%x\t%x\t%x\t%x\t%x\t%x\n",timeStamp[0], timeStamp[1], timeStamp[2], timeStamp[3],timeStamp[4], timeStamp[5], timeStamp[6]);
        return 0;
    }
    
}

//---------------------- isr() -----------------------

/* isr function is the gpio interrupt service routine that increments count
 *
 * @return error_flag as boolean
 */

int RTC::isr() {
    if (initCount < 2) {
        initCount++;
        count3 = 0;
    }
    else if (initCount = 2) {
        count3 = 0;
        getTimeStamp();
        elapsedTime = 0;
        initCount++;
    }
    else {
        count3++;
        elapsedTime = count3 * interval;
    }
    return count3;
}


//*******************************************************************
//Functions for IMU
//*******************************************************************



//---------------------- setupAccel() -----------------------

/*
 This function sets up the config registers for the accelerometer
 */
void IMU::setupAccel(){
    
    i2c.setAddress(LSM303_ACCEL_ADDR);
    
    unsigned char writeBuf[] = {LSM303_CTRL_REG1_A, 0x57};
    i2c.send(writeBuf, 2);
    
    writeBuf[0] = LSM303_CTRL_REG4_A;
    writeBuf[1] = 0x58;
    i2c.send(writeBuf, 2);
}


/*---------------------- getAccelValue() -----------------------
  
  
 This function returns the accelerometer data
 */

float IMU::getAccelValue(int channel){
    float value;
    int16_t combinedBit = 0;
    Tuple imuTuple;
    //Tuple imuerror = Tuple (-1,-1);
    float imuerror = -1;
    i2c.setAddress(LSM303_ACCEL_ADDR);
    unsigned char readBuf[] = {0,0};
    unsigned char reg_addr [2];
    unsigned char writeBuf[] = {LSM303_CTRL_REG1_A, 0x57};
    
    /*
     // Enable the accelerometer (100Hz)
     write8(LSM303_ADDRESS_ACCEL, LSM303_REGISTER_ACCEL_CTRL_REG1_A, 0x57);
     
     // LSM303DLHC has no WHOAMI register so read CTRL_REG1_A back to check
     // if we are connected or not
     uint8_t reg1_a = read8(LSM303_ADDRESS_ACCEL, LSM303_REGISTER_ACCEL_CTRL_REG1_A);
     if (reg1_a != 0x57)
     {
     return false;
     }
     
     return true;
     */
    
    error_flag = false;
    i2c.send(writeBuf, 2);
    writeBuf[0] = LSM303_CTRL_REG4_A;
    writeBuf[1] = 0x58;
    i2c.send(writeBuf, 2);
    
    
    switch (channel) {  // IMU Channel Selection
    
    // ---------------------- Accelerometeer---------------------//
    case 1: // channel 1: x-acceleration
        writeBuf[0] = LSM303_OUT_X_L_A;
        writeBuf[1] = LSM303_OUT_X_H_A;
        break;
    case 2: // channel 2: y-acceleration
        writeBuf[0] = LSM303_OUT_Y_L_A;
        writeBuf[1] = LSM303_OUT_Y_H_A;
        break;
    case 3: // channel 3: z-acceleration
        writeBuf[0] = LSM303_OUT_Z_L_A;
        writeBuf[1] = LSM303_OUT_Z_H_A;
        break;
        
    default:
        error_flag = true;
        return imuerror;
        break; // channel selection out of IMU range
    }
    
    i2c.send(writeBuf,1);
    
    delay(2);
    
    i2c.receive(readBuf, 2);        // Read the contents of the conversion register into readBuf
    //read(I2CFile, readBuf, 2);
    
    imuTuple.low = readBuf[0];  // Combine the two bytes of readBuf into a single 16 bit result
    imuTuple.high = readBuf[1];
    
    if (i2c.fail()) {
        error_flag = true;
        return imuerror;
    }
    
    //error_flag = false;
    //printf("%d\t%d\n",newtuple.low,newtuple.high);
    //    int combinedBit = 0;
    //    combinedBit = imuTuple.low | ( imuTuple.high << 8 );
    combinedBit = imuTuple.low | ( imuTuple.high << 8 );
    if (combinedBit > 32767) {
        
        combinedBit -= 65536;
        
    }
    value = (combinedBit >> 4 ) * 4/2048.0;
    printf("%f g",  value);
    return value;
}


/*---------------------- getGyroValue() -----------------------
  
  
 This function returns the gyro data
 */



/* Set CTRL_REG1 (0x20)
 ====================================================================
 BIT  Symbol    Description                                   Default
 ---  ------    --------------------------------------------- -------
 7-6  DR1/0     Output data rate                                   00
 5-4  BW1/0     Bandwidth selection                                00
 3  PD        0 = Power-down mode, 1 = normal/sleep mode          0
 2  ZEN       Z-axis enable (0 = disabled, 1 = enabled)           1
 1  YEN       Y-axis enable (0 = disabled, 1 = enabled)           1
 0  XEN       X-axis enable (0 = disabled, 1 = enabled)           1 */

/* Reset then switch to normal mode and enable all three channels */

//write8(GYRO_REGISTER_CTRL_REG1, 0x00);
//write8(GYRO_REGISTER_CTRL_REG1, 0x0F);

/* ------------------------------------------------------------------ */

/* Set CTRL_REG2 (0x21)
 ====================================================================
 BIT  Symbol    Description                                   Default
 ---  ------    --------------------------------------------- -------
 5-4  HPM1/0    High-pass filter mode selection                    00
 3-0  HPCF3..0  High-pass filter cutoff frequency selection      0000 */

/* Nothing to do ... keep default values */
/* ------------------------------------------------------------------ */

/* Set CTRL_REG3 (0x22)
 ====================================================================
 BIT  Symbol    Description                                   Default
 ---  ------    --------------------------------------------- -------
 7  I1_Int1   Interrupt enable on INT1 (0=disable,1=enable)       0
 6  I1_Boot   Boot status on INT1 (0=disable,1=enable)            0
 5  H-Lactive Interrupt active config on INT1 (0=high,1=low)      0
 4  PP_OD     Push-Pull/Open-Drain (0=PP, 1=OD)                   0
 3  I2_DRDY   Data ready on DRDY/INT2 (0=disable,1=enable)        0
 2  I2_WTM    FIFO wtrmrk int on DRDY/INT2 (0=dsbl,1=enbl)        0
 1  I2_ORun   FIFO overrun int on DRDY/INT2 (0=dsbl,1=enbl)       0
 0  I2_Empty  FIFI empty int on DRDY/INT2 (0=dsbl,1=enbl)         0 */

/* Nothing to do ... keep default values */
/* ------------------------------------------------------------------ */

/* Set CTRL_REG4 (0x23)
 ====================================================================
 BIT  Symbol    Description                                   Default
 ---  ------    --------------------------------------------- -------
 7  BDU       Block Data Update (0=continuous, 1=LSB/MSB)         0
 6  BLE       Big/Little-Endian (0=Data LSB, 1=Data MSB)          0
 5-4  FS1/0     Full scale selection                               00
 00 = 250 dps
 01 = 500 dps
 10 = 2000 dps
 11 = 2000 dps
 0  SIM       SPI Mode (0=4-wire, 1=3-wire)                       0 */

/* Adjust resolution if requested */

//    case GYRO_RANGE_250DPS:
//        write8(GYRO_REGISTER_CTRL_REG4, 0x00);

//    case GYRO_RANGE_500DPS:
//        write8(GYRO_REGISTER_CTRL_REG4, 0x10);

//    case GYRO_RANGE_2000DPS:
//        write8(GYRO_REGISTER_CTRL_REG4, 0x20);

/* ------------------------------------------------------------------ */

/* Set CTRL_REG5 (0x24)
 ====================================================================
 BIT  Symbol    Description                                   Default
 ---  ------    --------------------------------------------- -------
 7  BOOT      Reboot memory content (0=normal, 1=reboot)          0
 6  FIFO_EN   FIFO enable (0=FIFO disable, 1=enable)              0
 4  HPen      High-pass filter enable (0=disable,1=enable)        0
 3-2  INT1_SEL  INT1 Selection config                              00
 1-0  OUT_SEL   Out selection config                               00 */

/* Nothing to do ... keep default values */
/* ------------------------------------------------------------------ */

//---------------------- setupGyro() -----------------------

/*
 This function sets up the config registers for the gyro
 */
void IMU::setupGyro(){
    
    i2c.setAddress(L3GD20_GYRO_ADDRESS);
    
    unsigned char readBuf[] = {0,0};
    unsigned char writeBuf[] = {0,0};
    
    
    writeBuf[0] = GYRO_REGISTER_CTRL_REG1; // reset
    writeBuf[1] = 0x00;
    i2c.send(writeBuf, 2);
    
    delay (1);
    
    writeBuf[0] = GYRO_REGISTER_CTRL_REG1; // enable
    writeBuf[1] = 0x0F;
    i2c.send(writeBuf, 2);
    
    delay(1);
    
    writeBuf[0] = GYRO_REGISTER_CTRL_REG4; // 2000 dps
    writeBuf[1] = 0x60; //0x20
    i2c.send(writeBuf, 2);
}

//---------------------- getGyroValue() -----------------------

float IMU::getGyroValue(int channel){
    float value;
    int16_t combinedBit = 0;
    Tuple imuTuple;
    float imuerror = -1;
    
    i2c.setAddress(L3GD20_GYRO_ADDRESS);
    
    unsigned char readBuf[] = {0,0};
    unsigned char writeBuf[] = {0,0};
    
    
    writeBuf[0] = GYRO_REGISTER_CTRL_REG1; // reset
    writeBuf[1] = 0x00;
    i2c.send(writeBuf, 2);
    
    delay (1);
    
    writeBuf[0] = GYRO_REGISTER_CTRL_REG1; // enable
    writeBuf[1] = 0x0F;
    i2c.send(writeBuf, 2);
    
    delay(1);
    
    writeBuf[0] = GYRO_REGISTER_CTRL_REG4; // 2000 dps
    writeBuf[1] = 0x60; //0x20
    i2c.send(writeBuf, 2);
    
    
    switch (channel) {  // IMU Channel Selection
    
    // ---------------------- GYRO---------------------//
    case 1: // channel 1: x-roll
        writeBuf[0] = GYRO_REGISTER_OUT_X_L;
        writeBuf[1] = GYRO_REGISTER_OUT_X_H;
        break;
    case 2: // channel 2: y-pitch
        writeBuf[0] = GYRO_REGISTER_OUT_Y_L;
        writeBuf[1] = GYRO_REGISTER_OUT_Y_H;
        break;
    case 3: // channel 3: z-yaw
        writeBuf[0] = GYRO_REGISTER_OUT_Z_L;
        writeBuf[1] = GYRO_REGISTER_OUT_Z_H;
        break;
        
    default:
        return imuerror;
        break; // channel selection out of IMU range
    }
    
    i2c.send(writeBuf,1);
    
    i2c.receive(readBuf, 2);        // Read the contents of the conversion register into readBuf
    //read(I2CFile, readBuf, 2);
    
    imuTuple.low = readBuf[0];  // Combine the two bytes of readBuf into a single 16 bit result
    imuTuple.high = readBuf[1];
    
    if (i2c.fail()) {
        return imuerror;
    }
    
    combinedBit = imuTuple.low | ( imuTuple.high << 8 );
    //if (combinedBit > 32767) {
    
    //    combinedBit -= 65536;
    
    //}
    
    value = combinedBit * GYRO_SENSITIVITY_2000DPS;
    
    printf("%f deg",  value);
    
    return value;
}



//---------------------- twosCompliment() -----------------------
/*
 This function recieves two bytes, combines them into a 12-bit number, and takes the twos compliment.
 
 */
/*
int IMU::twosComplement(int lowerBit, int higherBit) {

    int combinedBit = 0;
    combinedBit = lowerBit | ( higherBit << 8 );
    
    if (combinedBit > 32767) {
    
        combinedBit -= 65536;
        
    }
    return (combinedBit >> 4);
}

//---------------------- getValue() -----------------------

 This function returns the accelerometer data

 
---------------------- getValue() -----------------------


 This function returns the accelerometer data

/*
Tuple IMU::getValue(int channel){

    Tuple newtuple;
    Tuple imuerror = Tuple (-1,-1);
    i2c.setAddress(LSM303_ACCEL_ADDR);
    unsigned char readBuf[] = {0,0};
    unsigned char reg_addr [2];
    unsigned char writeBuf[] = {LSM303_CTRL_REG1_A,0x27}; /// read whoami register after send
    

     // Enable the accelerometer (100Hz)
     write8(LSM303_ADDRESS_ACCEL, LSM303_REGISTER_ACCEL_CTRL_REG1_A, 0x57);
     
     // LSM303DLHC has no WHOAMI register so read CTRL_REG1_A back to check
     // if we are connected or not
     uint8_t reg1_a = read8(LSM303_ADDRESS_ACCEL, LSM303_REGISTER_ACCEL_CTRL_REG1_A);
     if (reg1_a != 0x57)
     {
     return false;
     }
     
     return true;
     */
/*
    error_flag = false;
    i2c.send(writeBuf, 2);
    writeBuf[0] = LSM303_CTRL_REG4_A;
    writeBuf[1] = 0x18;
    i2c.send(writeBuf, 2);
    
    
    switch (channel) {  // IMU Channel Selection
    
            // ---------------------- Accelerometeer---------------------//
        case 1: // channel 1: x-acceleration
            writeBuf[0] = LSM303_OUT_X_L_A;
            writeBuf[1] = LSM303_OUT_X_H_A;
            break;
        case 2: // channel 2: y-acceleration
            writeBuf[0] = LSM303_OUT_Y_L_A;
            writeBuf[1] = LSM303_OUT_Y_H_A;
            break;
        case 3: // channel 3: z-acceleration
            writeBuf[0] = LSM303_OUT_Z_L_A;
            writeBuf[1] = LSM303_OUT_Z_H_A;
            break;
            
        default:
            error_flag = true;
            return imuerror;
            break; // channel selection out of IMU range
    }
    
    i2c.send(writeBuf,1);
    
    i2c.receive(readBuf, 2);        // Read the contents of the conversion register into readBuf
    //read(I2CFile, readBuf, 2);
    
    newtuple.low = readBuf[0];  // Combine the two bytes of readBuf into a single 16 bit result
    newtuple.high = readBuf[1];
    
    if (i2c.fail()) {
        error_flag = true;
        return imuerror;
    }
    
    //error_flag = false;
    //printf("%d\t%d\n",newtuple.low,newtuple.high);
    return newtuple;
}



// *******************************************************************
// Functions for STORAGE Class
// *******************************************************************

////------------------initialization------------------
// *
// * sets up storage file and initializes datasets
// *
// * @return error_flag as boolean
// */
//
//int STORAGE::init(){
//
//    //ofstream myfile;
//   // myfile.open ("Data.txt");
//    //myfile << "\n\n\n";
//    //myfile.close();
//    //return 0;
//
//    FILE *stream;
//    stream = fopen("DAQdata.txt","w+"); // adjust to format title as DAQdata_*date*
//    fprintf(stream, "%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s,\n", "ADC.steerAngle", "ADC.airSpeed", "ADC.brakePressB", "ADC.fuelTankLevel", "ADC.shockDispFL", "ADC.shockDispFR", "ADC.shockDispBR", "ADC.shockDispBL", "ADC.channel9", "ADC.channel10", "ADC.channel11", "ADC.channel12", "ECU.engineTemp", "ECU.waterTemp", "ECU.rpm", "ECU.throttlePos", "ECU.oilTemp", "ECU.oilPress", "ECU.lambda", "IMU.xAccel", "IMU.yAccel", "IMU.zAccel", "IMU.xRot", "IMU.yRot", "IMU.zRot", "IMU.temp", "GPS.GPSRMCsentance", "GPS.speed", "THAR.data", "RTC.interval", "RTC.startTime", "RTC.elapsedTime","RTC.timeStamp");
//    fclose (stream);
//    printf("txt file initialized\n");
//
//
//}
//
//
////------------------Save Data Set------------------
///*
// * saves the current data set to sd card's text file
// *
// * @return error_flag as boolean
// */
//
//int STORAGE::saveData(){
//    FILE *stream;
//    int i=0;
//    stream = fopen("DAQdata.txt","a"); // adjust to format title as DAQdata_*date*
//
//    for (i=0; i>=1023; i++ ) {
//    printf("countinloop = %i", i);
//    fprintf(stream, "%f", ADCdata.shockDispFL[i]);
//        fprintf(stream, "%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n", ADCdata.steerAngle[i], ADCdata.airSpeed[i], ADCdata.brakePressB[i], ADCdata.fuelTankLevel[i], ADCdata.shockDispFL[i], ADCdata.shockDispFR[i], ADCdata.shockDispBR[i], ADCdata.shockDispBL[i], ADCdata.channel9[i], ADCdata.channel10[i], ADCdata.channel11[i], ADCdata.channel12[i], INAdata.brakePressF[i], ECUdata.engineTemp[i], ECUdata.waterTemp[i], ECUdata.rpm[i], ECUdata.throttlePos[i], ECUdata.oilTemp[i], ECUdata.oilPress[i], ECUdata.lambda[i], IMUdata.xAccel[i], IMUdata.yAccel[i], IMUdata.zAccel[i], IMUdata.xRot[i], IMUdata.yRot[i], IMUdata.zRot[i], IMUdata.temp[i]);//, GPS.GPSRMC_sentence[i], GPS.speed[i]);//, THAR.data, RTC.interval, RTC.startTime, RTC.elapsedTime, RTC.timeStamp);
//        // accesss char using pointer within structured array
//    fclose(stream);
//    printf("count = %i",i);
//    }
//    printf("count = %i",i);
//    GPSdata.GPS_sentenceVector.clear(); //clear gps vector
//    printf("data save complete\n");
//
//}
////
////
////-------------------------------Fail-------------------------------
///* returns the error flag to check if the last operation went wrong
// *
// * @return error_flag as boolean
// */
//
// bool STORAGE::fail(){
// return error_flag;
// }
//


//*******************************************************************
//Functions for GPS
//*******************************************************************


//---------------------- init() -----------------------

/* Initialize UART for the GPS
  
 */

int GPS::init(void){
    nFd = open(GPS_DEVICE, O_RDWR|O_NOCTTY|O_NDELAY);
    if(-1 == nFd){
        perror("Open Serial Port Error!\n");
        return -1;
    }
    if( (fcntl(nFd, F_SETFL, 0)) < 0 ){
        perror("Fcntl F_SETFL Error!\n");
        return -1;
    }
    if(tcgetattr(nFd, &stOld) != 0){
        perror("tcgetattr error!\n");
        return -1;
    }
    stNew = stOld;
    cfmakeraw(&stNew);//raw mode
    //set speed
    cfsetispeed(&stNew, GPS_BAUDRATE);//from header file, baudrate  = 9600
    cfsetospeed(&stNew, GPS_BAUDRATE);
    //set databits
    stNew.c_cflag |= (CLOCAL|CREAD);
    stNew.c_cflag &= ~CSIZE;
    stNew.c_cflag |= CS8;
    //set parity
    stNew.c_cflag &= ~PARENB;
    stNew.c_iflag &= ~INPCK;
    //set stopbits
    stNew.c_cflag &= ~CSTOPB;
    stNew.c_cc[VTIME]=0;    //?????????????
    stNew.c_cc[VMIN]=1; //??????????????,??????n*100ms
    //????VTIME=0,???????read()????????
    tcflush(nFd,TCIFLUSH);  //??????????/????????
    if( tcsetattr(nFd,TCSANOW,&stNew) != 0 ){
        perror("tcsetattr Error!\n");
        return -1;
    }
    
    nRetw = write(nFd, PMTK_SET_BAUD_9600, std::char_traits<char>::length(PMTK_SET_BAUD_9600));
    
    nRetw = write(nFd, PMTK_API_SET_SBAS_ENABLED, std::char_traits<char>::length(PMTK_API_SET_SBAS_ENABLED));
    
    nRetw = write(nFd, PMTK_SET_NMEA_OUTPUT_RMCONLY, std::char_traits<char>::length(PMTK_SET_NMEA_OUTPUT_RMCONLY));
    
    nRetw = write(nFd, PMTK_SET_NMEA_UPDATE_10HZ, std::char_traits<char>::length(PMTK_SET_NMEA_UPDATE_10HZ));
    
    if(-1 == nRetw){
        perror("Write Data Error!\n");
        return -1;
    }
    bzero(buf, GPS_BUFFER_SIZE);
    return nFd;
}
//---------------------- getGPSRMC() -----------------------

/* Get sentance of data from the GPS
  
 */
std::string GPS::getGPSRMC(void){
    bzero(buf, GPS_BUFFER_SIZE);
    storage = "";
    nRet = read(nFd, buf, GPS_BUFFER_SIZE);
    if(-1 == nRet){
        perror("Read Data Error!\n");
    }
    if(0 < nRet){
        if (storage == "" && buf[0] == '$' && buf[1] == 'G'){
            storage.append(buf);
            for (int j = 0; j < 7; j++){
                nRet = read(nFd, buf, GPS_BUFFER_SIZE);
                if (buf[0] != '$'){
                    storage.append(buf);
                }
                else j = 7;
            }
        }
        else if(storage ==""){
            GPS::getGPSRMC();
        }
    }
    
    return storage;
    //close(nFd);
}

//---------------------- getSpeed() -----------------------

/* get Speed from GPS and convert to MPH for the Dash
  
 */
int GPS::getSpeed (std::string GPSRMC_sentence){
    //inspired by http://stackoverflow.com/questions/1894886/parsing-a-comma-delimited-stdstring
    std::istringstream ss(GPSRMC_sentence);
    std::string token;
    std::string::size_type sz;     // alias of size_t
    int speed_in_mph;
    int i;
    for (int i = 0; i <10; i++){
        std::getline(ss, token, ',');
        if (token == "W"){
            std::getline(ss, token, ',');
            break;
        }
    }
    double speed_in_knot = std::stod (token,&sz);
    //double speed_in_knot = 10;
    speed_in_mph = ceil(speed_in_knot * 0.868976242);
    //printf("%i",speed_in_mph);
    return speed_in_mph;
}

int GPS::getValue (unsigned char channel){
    int value=0;
    unsigned char readBuf[2];
    readBuf[0]= 0;
    readBuf[1]= 0;
    unsigned char writeBuf[3];
    teensyAddress = 0x42;
    i2c.setAddress(teensyAddress);

    writeBuf[0] = channel;

    i2c.send(writeBuf, 1);
    delay(2);
    i2c.receive(readBuf, 2); //read the config register into readbuf

    value = readBuf[0];

    printf("%i (mph) ",  value);
    value = readBuf[0];
    value = (value<<8) | readBuf[1];
    printf("%i (2 byte) ",  value);
    return value;
}


//*******************************************************************
//TEST Routines
//******************************************************************

//************************************************************
// Interrupt
//************************************************************
//void DAQinterrupt () {};
////
////
//    countOverload++;
//    count2++;
//    printf("interrupt");
//    printf(" %i \n",  countOverload);
////
////
//    //ADCdata.shockDispFL[countOverload] = adc.getValue(5);
//    //ADCdata.shockDispFR[countOverload] = adc.getValue(6);
//    //ADCdata.shockDispBR[countOverload] = adc.getValue(7);
//    //ADCdata.shockDispBL[countOverload] = adc.getValue(8);
//    printf(" get adc\n ");
//    //IMUdata.xAccel[countOverload] = imu.getAccelValue(1);
//    //IMUdata.yAccel[countOverload] = imu.getAccelValue(2);
//    //IMUdata.zAccel[countOverload] = imu.getAccelValue(3);
//    printf("\n ");
//    //IMUdata.xRot[countOverload] = imu.getGyroValue(1);
//    //IMUdata.yRot[countOverload] = imu.getGyroValue(2);
//    //IMUdata.zRot[countOverload] = imu.getGyroValue(3);
//    //INAdata.brakePressF[countOverload] = ina219.getValue();
////
////    printf("getting GPS");
////
//    //GPSdata.GPS_sentenceVector.push_back(gps.getGPSRMC());
//    //GPS_sentence.copy(GPSdata.GPSRMC_sentence[countOverload], sizeof GPSdata.GPSRMC_sentence[countOverload]);
//    printf("get Speed");
//    //GPSdata.speed[countOverload] = gps.getSpeed(GPSdata.GPS_sentenceVector[countOverload]);
////
////   // printf("stored value: %f, %f ",  GPSdata.speed[countOverload], ADCdata.shockDispFL[countOverload]);
////
//
//}
//
//
////************************************************************
//// Main Test Functions
////****/*********************************************************
//
//int main(int argc, char **argv) {
//
//    printf("starting\n");
//    //ina219.setup();
//    storage.init();
//    //imu.setupAccel();
//    //imu.setupGyro();
//    gps.init();
//    //rtc.getTimeStamp();
//
//
//
////    //GPIO test_instance(BUTTON_PIN, INT_EDGE_FALLING) ;
////    //test_instance.isrInit();
//
//
////    printf ("GPS Initialized\n ") ;
////    rtc.initSKW(DS3231_1Hz);
//    printf ("Waiting ...\n ") ;
//
////    delay(5000);
//
//    gpio.isrInit();
//    printf ("Begin Interrupts ...\n ") ;
//
//    while(1)	{
//
//        if (countOverload >= 1024) {
//            printf("dumping data\n");
//            storage.saveData();
//            GPSdata.GPS_sentenceVector.clear();
//	    countOverload = 0;
//        }
//    }
//
////    // detach interrupt
////    //system ("/usr/local/bin/gpio edge 6 none") ; // what is this pin? this just disables GPIO, wiring pi has no built in function for this.
//
//
////    /*
////     So to disable interrupts - well, there's no easy built-in way, but this will work:
//
////     system ("/usr/local/bin/gpio edge 17 none") ;
//
////     and to allow them again:
//
////     system ("/usr/local/bin/gpio edge 17 rising") ;
//
//
////    for (k=0; k<1023; ++k) {
////        printf("%x  %d  %f\n", secondBuffer[k], countBuffer[k], adcBuffer[k]);
//
////    }
//
////    printf("%d\n" , k);
////    printf("%d" , count2);
//
////    */
//    return 0 ;
//}
