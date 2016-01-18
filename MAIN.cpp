
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
/////////////////////////////////////////////////////////////////////////////////////



//
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

#include "MAIN.h"

#define DEFAULTDEVICE "/dev/i2c-1"  //


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
 * This function reads "length" number of bytes from the i2c bus and stores them into the "RxBuf". At success the function returns 1, on failure -1.<br>
 * @param RxBuf Receive buffer. The read bytes will be stored in it.
 * @param length Amount of bytes that will be read.
 * @return success: 1, failure: -1
 */
int i2cBus::receive(unsigned char *RxBuf, int length){
    
	if (RxBuf == 0) {
		return errorMsg("Receive method received a null TxBuf pointer.\n");
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
    
	if (read(file, RxBuf, length) != length) {
		return errorMsg("i2c read error! Address: " + numberToString(slave_address) + " dev file: " + devicefile + "\n");
    }
	return 1;
}

//----------------------------------receive----------------------------------
/*
 * receive bytes from the I2C bus.
 *
 * This function reads "length" number of bytes from the register "RegisterAddress" and stores them into the "RxBuf". At success the function returns 1, on failure -1.
 * @param RegisterAddress Address of the register you want to read from
 * @param RxBuf Receive buffer. The read bytes will be stored in it.
 * @param length Amount of bytes that will be read.
 * @return success: 1, failure: -1
 */
int i2cBus::receive(unsigned char RegisterAddress, unsigned char *RxBuf, int length){
    
	if (RxBuf == 0) {
		return errorMsg("Receive method received a null TxBuf pointer.\n");
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
    
	if (write(file, &RegisterAddress, 1) != 1) {
  		return errorMsg("i2c write error!\n");
    }
	if (read(file, RxBuf, length) != length) {
		return errorMsg("i2c read error! Address: " + numberToString(slave_address) + " dev file: " + devicefile + "\n");
    }
    
	return 1;
}

//----------------------------------send----------------------------------
/*
 * send bytes to the I2C bus.
 *
 * This function sends "length" number of bytes from the "TxBuf" to the i2c bus. At success the function returns 1, on failure -1.
 * @param TxBuf Transmit buffer. The bytes you want to send are stored in it.
 * @param length Amount of bytes that will be send.
 * @return success: 1, failure: -1
 */
int i2cBus::send(unsigned char *TxBuf, int length){
    
	if (TxBuf == 0) {
		return errorMsg("Send method received a null TxBuf pointer.\n");
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
    
	if(write(file, TxBuf, length) != length) {
		return errorMsg("i2c write error!\n");
    }
	return 1;
}

//----------------------------------send----------------------------------
/*
 * send bytes to the I2C bus.
 *
 * This function sends "length" number of bytes from the "TxBuf" to the register "RegisterAddress". At success the function returns 1, on failure -1.
 * @param RegisterAddress Address of the register you want to send the bytes to
 * @param TxBuf Transmit buffer. The bytes you want to send are stored in it.
 * @param length Amount of bytes that will be send.
 * @return success: 1, failure: -1
 */
int i2cBus::send(unsigned char RegisterAddress, unsigned char *TxBuf, int length){
	int i;
	unsigned char data[length+1];
	data[0]=RegisterAddress;
	
	for ( i = 0; i < length ; i++ ) {
		data[i+1] = TxBuf[i];
	}
    
	if (TxBuf == 0) {
		return errorMsg("Send method received a null TxBuf pointer.\n");
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
 
     #define ADC_ch1 1
     #define ADC_ch2 2
     #define ADC_ch3 3
     #define ADC_ch4 4
     #define ADC_ch5 5
     #define ADC_ch6 6
     #define ADC_ch7 7
     #define ADC_ch8 8
 
 * @return value
 */

int ADC::getValue(int channel) {
    
    int value=0;
    unsigned char readBuf[2];
    readBuf[0]= 0;
    readBuf[1]= 0;
    unsigned char writeBuf[3];
    
    int ADS_address = 0x48;   // Address of our device on the I2C bus
    int I2CFile;
    I2CFile = open("/dev/i2c-1", O_RDWR);     // Open the I2C device
    ioctl(I2CFile, I2C_SLAVE, ADS_address);   // Specify the address of the I2C Slave to communicate with

    error_flag=false;
    
		switch (channel) {  // ADC Channel Selection
                
                // ---------------------- ADC 12-bit---------------------//

			case 1: // steerAngle
                
                break; // channel 1
			case 2: // airSpeed
                
                break; // channel 2
			case 3: // brakePressF
                
                break; // channel 3
			case 4: // brakePressB
                
                break; // channel 4
                
               // ---------------------- ADC 16-bit---------------------//
			case 5: // shockDispFL
                writeBuf[0] = 1;          // This sets the pointer register so that the following two bytes write to the config register
                writeBuf[1] = 0xC3;       // This sets the 8 MSBs of the config register (bits 15-8) 11000011
                writeBuf[2] = 0xE3;       // This sets the 8 LSBs of the config register (bits 7-0) 11100011
                break; // channel 5
                
			case 6: // shockDispFR
                writeBuf[0] = 1;          // This sets the pointer register so that the following two bytes write to the config register
                writeBuf[1] = 0xD3;       // This sets the 8 MSBs of the config register (bits 15-8) 11010011
                writeBuf[2] = 0xE3;       // This sets the 8 LSBs of the config register (bits 7-0) 11100011
                break; // channel 6
                
			case 7: // shockDispBR
                writeBuf[0] = 1;          // This sets the pointer register so that the following two bytes write to the config register
                writeBuf[1] = 0xE3;       // This sets the 8 MSBs of the config register (bits 15-8) 11100011
                writeBuf[2] = 0xE3;       // This sets the 8 LSBs of the config register (bits 7-0) 11100011
                break; // channel 7
                
			case 8: // shockDispBL
                writeBuf[0] = 1;          // This sets the pointer register so that the following two bytes write to the config register
                
                writeBuf[1] = 0xF3;       // This sets the 8 MSBs of the config register (bits 15-8) 11110011
                writeBuf[2] = 0xE3;       // This sets the 8 LSBs of the config register (bits 7-0) 11100011
                break; // channel 8
                
			default: error_flag = true; return -1; break; // channel selection out of ADC range
		}
    
    //i2c.send(writeBuf, 3);
    write(I2CFile, writeBuf, 3);
    
    while ((readBuf[0] & 0x80) == 0)  // readBuf[0] contains 8 MSBs of config register, AND with 10000000 to select bit 15
    {
        //i2c.receive(readBuf, 2);    // Read the config register into readBuf
        read(I2CFile, readBuf, 2);
        
    }
    
    writeBuf[0] = 0;                  // Set pointer register to 0 to read from the conversion register
    //i2c.send(writeBuf, 1);
    write(I2CFile, writeBuf, 1);
    
    read(I2CFile, readBuf, 2);
    //i2c.receive(readBuf, 2);        // Read the contents of the conversion register into readBuf
    
    value = readBuf[0] << 8 | readBuf[1];   // Combine the two bytes of readBuf into a single 16 bit result
    
    //printf("Voltage Reading %f (V) \n", (float)value*4.096/32767.0);    // Print the result to terminal, first
    
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


//---------------------- getStartTime() -----------------------

//regaddr,seconds,minutes,hours,weekdays,days,months,yeas
//char  buf[]={0x00,0x00,0x00,0x18,0x04,0x12,0x08,0x15};

/*
void zs042SetTime()
{
    bcm2835_i2c_write(buf,8);
}
*/
void getStartTime() {
    const char  *str[]  ={"Sun","Mon","Tues","Wed","Thur","Fri","Sat"};
    int buf[5];
   // bcm2835_i2c_write_read_rs(buf ,1, buf,7);
    
    buf[0] = buf[0]&0x7F; //sec
    buf[1] = buf[1]&0x7F; //min
    buf[2] = buf[2]&0x3F; //hour
    buf[3] = buf[3]&0x07; //week
    buf[4] = buf[4]&0x3F; //day
    buf[5] = buf[5]&0x1F; //mouth
    

}


//---------------------- getTimeStamp() -----------------------

//regaddr,seconds,minutes,hours,weekdays,days,months,yeas
//char  buf[]={0x00,0x00,0x00,0x18,0x04,0x12,0x08,0x15};

/*
 void zs042SetTime()
 {
 bcm2835_i2c_write(buf,8);
 }
 */
void getTimeStamp() {
    const char  *str[]  ={"Sun","Mon","Tues","Wed","Thur","Fri","Sat"};
    int buf[3];
   // bcm2835_i2c_write_read_rs(buf ,1, buf,7);

    buf[0] = buf[0]&0x7F; //sec
    buf[1] = buf[1]&0x7F; //min
    buf[2] = buf[2]&0x3F; //hour


}

//*******************************************************************
//Functions for IMU
//*******************************************************************


//---------------------- twosCompliment() -----------------------
/* 
 This function recieves two bytes, combines them into a 12-bit number, and takes the twos compliment.
 */
int twos_complement(int lower_bit, int higher_bit){
    int combined_bit = 0;
    combined_bit = lower_bit | ( higher_bit << 8 );
    if (combined_bit > 32767){
        combined_bit -= 65536;
    }
    return (combined_bit >> 4);
}

//---------------------- getAccelData() -----------------------

/* 
 This function returns the accelerometer data
 */

int IMU::getAccelData(){
   /*
    // does there need to be a send byte to initiate transfer???
    
    int xlower = i2c_smbus_read_byte_data(fd,LSM303_OUT_X_L_A);
    //i2c_smbus_write_byte_data(fd, LSM303_OUT_X_H_A, 0x00);//reposition the pointer to register 0x29
    int xhigher = i2c_smbus_read_byte_data(fd,LSM303_OUT_X_H_A);
    int ylower = i2c_smbus_read_byte_data(fd,LSM303_OUT_Y_L_A);
    int yhigher = i2c_smbus_read_byte_data(fd,LSM303_OUT_Y_H_A);
    int zlower = i2c_smbus_read_byte_data(fd,LSM303_OUT_Z_L_A);
    int zhigher = i2c_smbus_read_byte_data(fd,LSM303_OUT_Z_H_A);
    */
}



//*******************************************************************
//Functions for STORAGE Class
//*******************************************************************

//------------------initialization------------------
/*
 * sets up storage file and initializes datasets
 *
 * @return error_flag as boolean
 */

int STORAGE::init(){
    
    
}
 

//------------------Save Data Set------------------
/*
 * saves the current data set
 *
 * @return error_flag as boolean
 */

int STORAGE::saveData(){
    
    
    
}
 
 
//-------------------------------Fail-------------------------------
/* returns the error flag to check if the last operation went wrong
 *
 * @return error_flag as boolean
 */

 bool STORAGE::fail(){
 return error_flag;
 }


//*******************************************************************
//Main Routine
//*******************************************************************

int main(int argc, char **argv) {

    i2cBus i2c;
    ADC adc16;
    int val;
    
    for (int i=0; i<10; i++) {
        //val = adc16.getValueTest();
        //printf("Voltage Test: %f (V)", (float)val*4.096/32767.0);
        
        val = adc16.getValue(5);
        printf("Voltage A0: %f  ", (float)val*4.096/32767.0);
        
        val = adc16.getValue(6);
        printf("Voltage A1: %f  ", (float)val*4.096/32767.0);
        
        val = adc16.getValue(7);
        printf("Voltage A2: %f  ", (float)val*4.096/32767.0);
        
        val = adc16.getValue(8);
        printf("Voltage A3: %f  \n", (float)val*4.096/32767.0);
    }
    
    printf ("complete \n");
}
