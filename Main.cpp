
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

#define DEFAULTDEVICE "/dev/i2c-0"  // this changes depending on Rapsberry Pi Model
                                    // i2c-1 is for V2 pi's. For V1 Model B you need i2c-0


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
    
	if (slave_address == -1)
		return errorMsg("ERROR slave address is not set\n");
    
	if ((file = open(devicefile.c_str(), O_RDWR)) < 0)
		return errorMsg("ERROR opening: " + devicefile + "\n");
    
	if (ioctl(file, I2C_SLAVE, slave_address) < 0)
		return errorMsg("ERROR address: " + numberToString(slave_address) + "\n");
    
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
const char *i2CBus::getErrorMessage(){
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
    
	if (RxBuf == 0)
		return errorMsg("Receive method received a null TxBuf pointer.\n");
	if (length < 1)
		return errorMsg("Receive method received an invalid buffer length.\n");
    
	if (!file)
        if (open_file() == -1)
            return -1;
    
	error_flag=false;
    
	if (read(file, RxBuf, length) != length)
		return errorMsg("i2c read error! Address: " + numberToString(slave_address) + " dev file: " + devicefile + "\n");
    
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
    
	if (RxBuf == 0)
		return errorMsg("Receive method received a null TxBuf pointer.\n");
	if (length < 1)
		return errorMsg("Receive method received an invalid buffer length.\n");
    
	if (!file)
		if (open_file() == -1)
            return -1;
    
	error_flag=false;
    
	if (write(file, &RegisterAddress, 1) != 1)
  		return errorMsg("i2c write error!\n");
    
	if (read(file, RxBuf, length) != length)
		return errorMsg("i2c read error! Address: " + numberToString(slave_address) + " dev file: " + devicefile + "\n");
    
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
    
	if (TxBuf == 0)
		return errorMsg("Send method received a null TxBuf pointer.\n");
	if (length < 1)
		return errorMsg("Send method received an invalid buffer length.\n");
    
	if (!file)
		if (open_file() == -1)
            return -1;
    
	error_flag=false;
    
	if(write(file, TxBuf, length) != length)
		return errorMsg("i2c write error!\n");
    
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
    
	if (TxBuf == 0)
		return errorMsg("Send method received a null TxBuf pointer.\n");
	if (length < 1)
		return errorMsg("Send method received an invalid buffer length.\n");
    
	if (!file)
		if (open_file() == -1)
            return -1;
    
	error_flag=false;
    
    /*	if (send(RegisterAddress) == -1)
     return -1;
     */
	if(write(file, data, length+1) != length+1)
		return errorMsg("i2c write error!\n");
    
	return 1;
}

//----------------------------------send----------------------------------
/*
 * send a byte to the I2C bus.
 *
 * This function sends a byte to the i2c bus. At success the function returns 1, on failure -1.
 * @param value byte that will be sent.
 * @return success: 1, failure: -1
 */

int i2cBus::send(unsigned char value){
    
	if (!file)
		if (open_file() == -1)
            return -1;
    
	error_flag=false;
    
	if(write(file, &value, 1) != 1)
		return errorMsg("i2c write error!\n");
    
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
    
	int command;
    int value=0;
    unsigned char rx_buf[2];
    error_flag=false;
    
		switch (channel) {  // ADC Channel Selection
			case 1: command = 0x01; break; // channel 1

			case 2: command = 0x02; break; // channel 2
                
			case 3: command = 0x03; break; // channel 3
                
			case 4: command = 0x04; break; // channel 4
                
			case 5: command = 0x05; break; // channel 5
            
			case 6: command = 0x06; break; // channel 6
                
			case 7: command = 0x07; break; // channel 7
        
			case 8: command = 0x08; break; // channel 8
                
			default: error_flag = true; return -1; break; // channel selection out of ADC range
		}

	}
	// do i need to send device address first???????????
	i2cBus.send(command);
	if (i2cBus.fail()) {
		error_flag = true;
		return -1;
	}


    if(i2cBus.receive(rx_buf, 2)>0){
        // rx_buf[0] = MSByte
        // rx_buf[1] = LSByte
        // save the MSB
        value = rx_buf[0];
        // make space for the LSB
        value<<=8;
        // save the LSB
        value |= rx_buf[1];
        // 12 - Bit, so shift right 5 times
        value>>=5;
    
    }
    else{
        error_flag=true;
        return -1;
    }

	if (i2cBus.fail()) {
		error_flag = true;
		return -1;
	}
	error_flag = false;
	return value[0];
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
    unsigned char rx_buf[64]
    int pixel [4][16]
    int pixelValue [4][16]
    error_flag=false;
    
    switch (channel) {  // THAR Channel Selection
        case 1: command = 0x01; break; // channel 1
            
        case 2: command = 0x02; break; // channel 2
            
        case 3: command = 0x03; break; // channel 3
            
        case 4: command = 0x04; break; // channel 4
            
        default: error_flag = true; return -1; break; // channel selection out of ADC range
    }
    

    // do i need to send device address first???????????
    i2cBus.send(command);
    if (i2cBus.fail()) {
        error_flag = true;
        return -1;
    }
    for (pixelRow=1; pixelRow<5; pixelRow ++) {
        for (pixelColumn=1; pixelColumn<17; pixelColumn ++) {
            
            pixelAddress = pixel[pixelRow][pixelColumn] // some sort of conversion/reference to address
            pixelAddress = [0x01]                       // hex address
        
            i2cBus.send(pixelAddress);
            if (i2cBus.fail()) {
                error_flag = true;
                return -1;
            }

            if(i2c.receive(rx_buf, 1)>0){
                value = rx_buf[0];
                pixelValue[pixelRow][pixelColumn] = value

                
            }
            else{
                error_flag=true;
                return -1;
            }

            if (i2cBus.fail()) {
                error_flag = true;
                return -1;
            }
            error_flag = false;
            return value[0];

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
    unsigned char rx_buf[64]
    int pixel [4][16]
    int pixelValue [4][16]
    error_flag=false;
    
    switch (channel) {  // ADC Channel Selection
        case 1: command = 0x01; break; // channel 1
            
        case 2: command = 0x02; break; // channel 2
            
        case 3: command = 0x03; break; // channel 3
            
        case 4: command = 0x04; break; // channel 4
            
        default: error_flag = true; return -1; break; // channel selection out of ADC range
    }
    
    pixelAddress = pixel[pixelRow][pixelColumn] // some sort of conversion/reference to address
    pixelAddress = [0x01]                       // hex address
    
    i2cBus.send(pixelAddress);
    if (i2cBus.fail()) {
        error_flag = true;
        return -1;
    }
    
    if(i2c.receive(rx_buf, 1)>0){
        value = rx_buf[0];
        pixelValue[pixelRow][pixelColumn] = value
        
        
    }

    else{
        error_flag=true;
        return -1;
    }

    if (i2cBus.fail()) {
        error_flag = true;
        return -1;
    }
    error_flag = false;
    return value[0];
    }
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
char  *str[]  ={"Sun","Mon","Tues","Wed","Thur","Fri","Sat"};
/*
void zs042SetTime()
{
    bcm2835_i2c_write(buf,8);
}
*/
void getStartTime() {
    
    buf[0] = 0x00;
    bcm2835_i2c_write_read_rs(buf ,1, buf,7);
    
    buf[0] = buf[0]&0x7F; //sec
    buf[1] = buf[1]&0x7F; //min
    buf[2] = buf[2]&0x3F; //hour
    buf[3] = buf[3]&0x07; //week
    buf[4] = buf[4]&0x3F; //day
    buf[5] = buf[5]&0x1F; //mouth
    
    return 0;
}


//---------------------- getTimeStamp() -----------------------

//regaddr,seconds,minutes,hours,weekdays,days,months,yeas
//char  buf[]={0x00,0x00,0x00,0x18,0x04,0x12,0x08,0x15};
char  *str[]  ={"Sun","Mon","Tues","Wed","Thur","Fri","Sat"};
/*
 void zs042SetTime()
 {
 bcm2835_i2c_write(buf,8);
 }
 */
void getTimeStamp()
{
    buf[0] = 0x00;
    bcm2835_i2c_write_read_rs(buf ,1, buf,7);

    buf[0] = buf[0]&0x7F; //sec
    buf[1] = buf[1]&0x7F; //min
    buf[2] = buf[2]&0x3F; //hour

    
    return 0;
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
    
    // does there need to be a send byte to initiate transfer???
    
    int xlower = i2c_smbus_read_byte_data(fd,LSM303_OUT_X_L_A);
    //i2c_smbus_write_byte_data(fd, LSM303_OUT_X_H_A, 0x00);//reposition the pointer to register 0x29
    int xhigher = i2c_smbus_read_byte_data(fd,LSM303_OUT_X_H_A);
    int ylower = i2c_smbus_read_byte_data(fd,LSM303_OUT_Y_L_A);
    int yhigher = i2c_smbus_read_byte_data(fd,LSM303_OUT_Y_H_A);
    int zlower = i2c_smbus_read_byte_data(fd,LSM303_OUT_Z_L_A);
    int zhigher = i2c_smbus_read_byte_data(fd,LSM303_OUT_Z_H_A);
    
}

//---------------------- getTimeStamp() -----------------------

//regaddr,seconds,minutes,hours,weekdays,days,months,yeas
//char  buf[]={0x00,0x00,0x00,0x18,0x04,0x12,0x08,0x15};
char  *str[]  ={"Sun","Mon","Tues","Wed","Thur","Fri","Sat"};
/*
 void zs042SetTime()
 {
 bcm2835_i2c_write(buf,8);
 }
 */
void getTimeStamp()
{
    buf[0] = 0x00;
    bcm2835_i2c_write_read_rs(buf ,1, buf,7);
    
    buf[0] = buf[0]&0x7F; //sec
    buf[1] = buf[1]&0x7F; //min
    buf[2] = buf[2]&0x3F; //hour
    
    
    return 0;
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
// Data Structures Initialization & Variable Declartation
//*******************************************************************

// FORMAT FOR VARIABLES:

//------------------Thermal Array Sensor Variables Init------------------
/* Thermal Array variable notation is referenced as follows:
 * THAR -- first tag for Thermal Array
 
 * F -- Front
 * B -- Back
 * R -- Right
 * L -- Left
 * 4x16 array
 */

struct THAR {
    int FL[4][16];
    int FR[4][16];
    int BR[4][16];
    int BL[4][16];
};



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

struct IMU {
    int xAccel;
    int yAccel;
    int zAccel;
    int xRot;
    int yRot;
    int zRot;
    int temp;
};

vector<int> IMU = { IMU_xAccel, IMU_yAccel, IMu_zAccel, IMU_xRot, IMU_yRot, IMU_zRot, IMU_temp };
// equivalent to: vector<int> IMU = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };


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

struct ADC {
    int steerAngle; // channel 1
    
    int airSpeed; // channel 2
    
    int brakePressF; // channel 3
    int brakePressB; // channel 4
    
    int shockDispFL; // channel 5
    int shockDispFR; // channel 6
    int shockDispBR; // channel 7
    int shockDispBL; // channel 8
    
};

vector<int> ADC = { ADC.steerAngle, ACD.airSpeed, ADC.brakePressF, ADC.brakePressB, ADC.shockDispFL, ADC.shockDispFR, ADC.shockDispBL };
// equivalent to: vector<int> IMU = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };


//------------------RTC Variables Init------------------


/* Real Time Clock variable notation is referenced as follows:
 * RTC -- first tag for all real time clock variables
 
 * startTime   -- initial time stamp given by real time clock
 * elapsedTime -- elapsed time since initial time stamp
 * stampTime   -- time stamp of each reading
 * interval    -- interval between each sample
 */

struct RTC {
    int startTime;
    int elapsedTime;
    int timeStamp;
    int interval;
    
};

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

struct ECU {
    int engineTemp;
    int waterTemp;
    int rpm;
    int throttlePos;
    int oilTemp;
    int oilPress;
    int lambda;
};



//*******************************************************************
//Main Routine
//*******************************************************************

int main(int argc, char **argv) {
    // setup bus connections
    
    i2cBus i2cBus;
    
    i2cBus.setAddress(0x42);
    
    unsigned char TxBuf[8]; //transmittion buffer creater for outgoing data
    unsigned char RxBuf[8]; //recieving buffer created for incomming data
    
    TxBuf[0]=0x22;
    
    i2c.send(TxBuf,5);          // sends 5 bytes from the TxBuf
    i2c.send(0x12, TxBuf, 2);   //sends two bytes from the buffer to the address 0x12
    
    i2c.receive(RxBuf, 3);       // receives 3 bytes and stores them in the RxBuf
    i2c.receive(0x23, RxBuf, 3);  // receives 3 bytes from the address 0x23 and stores them in the RxBux
    
    
        // i2c
    
        // serial
    
        // uart
    
    
    // if no errors continue, else try again
    int i2cBus::fail()
    
    if (!= 1)
    // create logging file constructor and formatting
    
    // zero and initialize variables.
    
    // begin collection
        
        
    
    // interupt download
    
    
    
    
    
    
    
    
}
