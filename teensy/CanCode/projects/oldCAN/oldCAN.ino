
//#include <Arduino.h>
#include <i2c_t3.h> //Updated Wire Library for Teensy3
#include <SPI.h>

//1.CAN
#include <CAN.h>
//#if defined(ARDUINO_ARCH_AVR) // Arduino with SPI interface to MCP2515 chip
#include <CAN_MCP2515.h>
//#elif defined(ARDUINO_ARCH_SAM) // Arduino Due
#include <CAN_SAM3X.h>
//#elif defined(__MK20DX256__) // Teensy 3.1
#include <CAN_K2X.h>
//#else
//#error "Your CAN controller is currently unsupported."
//#endif
// Define CAN speed (bitrate)
#define canBitrate CAN_BPS_500K //PE3 pdf


/*****************************************
Global Variables
*****************************************/
//1.CAN
unsigned int RPM; //rpm 0-30000
signed int TPS; //percentage
signed int FuelOpenTime;  //ms
signed int IgnitionAngle;  //degrees

signed int Barometer;  //psi or kpa (refer to 7th byte of frame)
signed int MAP;  //psi or kpa (refer to 7th byte of frame)
signed int Lambda;
unsigned char PressType;  //0-psi, 1-kpa
signed int Oxygen;

signed int OilPress; //Psi
signed int BrakePress; //Psi
signed int SteerAngle; //Degrees

signed int FinalDrive; //Hz (frequency 1)
signed int WheelSpeedR; //Hz (frequency 2)
signed int WheelSpeedL; //Hz (frequency 3)

signed int Voltage;  //battery voltage V
signed int AirTemp;  //C or F (refer to 7th byte of frame)
signed int CoolantTemp;  //C or F (refer to 7th byte of frame)
unsigned char TempType; //0-F or 1-C

/*****************************************
Setup()
*****************************************/
void setup()
{ 
  Serial.begin(9600);
  Wire.begin();    //Master
  Wire1.begin(12); //Slave 
  
  //1.CAN
  //Set CAN speed. Note: Speed is now 250kbit/s so adjust your CAN monitor
  CAN.begin(canBitrate);
  
  SetFileNumber();
}

/*****************************************
Loop
*****************************************/
void loop()
{
  //1. Read CAN data
  readMessage();
  //Display Numerical Data over serial port  
  Serial.print("RPM: ");
  Serial.print(RPM); 
  Serial.print(", ");
  Serial.print("TPS: ");
  Serial.print(TPS); 
  Serial.print(", ");
  Serial.print("FuelOpenTime: ");
  Serial.print(FuelOpenTime);
  Serial.print(", "); 
  Serial.print("IgnitionAngle: ");
  Serial.print(IgnitionAngle);
  Serial.print(", ");
  Serial.print("Barometer: ");  
  Serial.print(Barometer);
  Serial.print(", ");
  Serial.print("MAP: ");
  Serial.print(MAP); 
  Serial.print(", ");
  Serial.print("Lambda: ");    
  Serial.print(Lambda); 
  Serial.print(", ");
  if(PressType == 0)
  {
    Serial.print("psi");
  } 
  else
  {
    Serial.print("kPa");
  }
  Serial.print(", ");
  Serial.print("Oxygen: ");
  Serial.print(Oxygen);
  Serial.print(", ");
  Serial.print("OilPress: ");
  Serial.print(OilPress);
  Serial.print(", ");
  Serial.print("BrakePress: ");
  Serial.print(BrakePress);
  Serial.print(", "); 
  Serial.print("SteerAngle: ");
  Serial.print(SteerAngle);
  Serial.print(", ");  
  Serial.print("FinalDrive: ");
  Serial.print(FinalDrive);
  Serial.print(", ");  
  Serial.print("WheelDriveR: ");
  Serial.print(WheelSpeedR);
  Serial.print(", ");
  Serial.print("WheelDriveL: ");
  Serial.print(WheelSpeedL);
  Serial.print(", ");  
  Serial.print("Voltage: ");
  Serial.print(Voltage); 
  Serial.print(", ");
  Serial.print("AirTemp: ");
  Serial.print(AirTemp); 
  Serial.print(", ");
  Serial.print("CoolantTemp: ");
  Serial.print(CoolantTemp);
  Serial.print(", ");
  if(TempType == 0)
  {
    Serial.print('F');
  } 
  else
  {
    Serial.print('C');
  }


/*****************************************
Functions
*****************************************/
//1. CAN
void readMessage()
{
  CAN_Frame message; // Create message object to use CAN message structure
  if (CAN.available() == true) // Check to see if a valid message has been received.
  {
    message = CAN.read(); //read message, it will follow the CAN structure of ID,RTR, legnth, data. Allows both Extended or Standard
    if (message.rtr == 1)
    {
      //Serial.print(F(" REMOTE REQUEST MESSAGE ")); //technically if its RTR frame == 1/message will not have data So display this
    }
    else
    { //Store data in appropiate variables
      switch(message.id)
      {
        /*All 2 Byte data is stored as [Low Byte, High Byte]
          Num = HighByte*256 + LowByte
          Conversion from 2 bytes to signed int:
          Num = HighByte*256 + LowByte
          if (Num >  32767) then Num = Num - 65536   
          Note: The indices for the array should be one less
                than the start positions specified in the PE3.
                This is because arrays are indexed from 0 not 1 */
        
        case 0x0CFFF048: //See PE3 pdf
          RPM = message.data[1]*256 + message.data[0];
          TPS = message.data[3]*256 + message.data[2];
            if(TPS > 32767) 
            {
              TPS = TPS - 65536;
            }
            TPS = TPS/10; //Why? Eliminating the decimal place 
          FuelOpenTime = message.data[5]*256 + message.data[4];
            if(FuelOpenTime > 32767) 
            {
              FuelOpenTime = FuelOpenTime - 65536;
            }
          IgnitionAngle = message.data[7]*256 + message.data[6];
            if(IgnitionAngle > 32767) 
            {
              IgnitionAngle = IgnitionAngle - 65536;
            }
          break;
        
        case 0x0CFFF148:
          Barometer = message.data[1]*256 + message.data[0];
            if(Barometer > 32767) 
            {
              Barometer = Barometer - 65536;
            }
            Barometer = Barometer/100;
          MAP = message.data[3]*256 + message.data[2];
            if(MAP > 32767) 
            {
              MAP = MAP - 65536;
            }
            MAP = MAP/100;
          Lambda = message.data[5]*256 + message.data[4];
            if(Lambda > 32767) 
            {
              Lambda = Lambda - 65536;
            }
            //Lambda = Lambda/100;
          PressType = message.data[6];
          break;
        
        case 0x0CFFF248:
          Oxygen = message.data[1]*256 + message.data[0];
            if(Oxygen > 32767) 
            {
              Oxygen = Oxygen - 65536;
            }
            Oxygen = Oxygen/100; //Must be calibrated
          break;  
        
        case 0x0CFFF348:
          OilPress = message.data[1]*256 + message.data[0];
            if(OilPress > 32767) 
            {
              OilPress = OilPress - 65536;
            }
            OilPress = OilPress/100; //Must be calibrated
          SteerAngle = message.data[3]*256 + message.data[2];
            if(SteerAngle > 32767) 
            {
              SteerAngle = SteerAngle - 65536;
            }
            SteerAngle = SteerAngle/10; //Must be calibrated                
          BrakePress = message.data[7]*256 + message.data[6];
            if(BrakePress > 32767) 
            {
              BrakePress = BrakePress - 65536;
            }
            BrakePress = BrakePress/100; //Must be calibrated 
          break;  
        
        case 0x0CFFF448:
          FinalDrive = message.data[1]*256 + message.data[0];
            if(FinalDrive > 32767) 
            {
              FinalDrive = FinalDrive - 65536;
            }
            FinalDrive = FinalDrive/10; //Must be calibrated
          WheelSpeedR = message.data[3]*256 + message.data[2];
            if(WheelSpeedR > 32767) 
            {
              WheelSpeedR = WheelSpeedR - 65536;
            }
            WheelSpeedR = WheelSpeedR/10; //Must be calibrated      
          WheelSpeedL = message.data[5]*256 + message.data[4];
            if(WheelSpeedL > 32767) 
            {
              WheelSpeedL = WheelSpeedL - 65536;
            }
            WheelSpeedL = WheelSpeedL/10; //Must be calibrated              
          break;            
        
        case 0x0CFFF548:
          Voltage = message.data[1]*256 + message.data[0];
            if(Voltage > 32767) 
            {
              Voltage = Voltage - 65536;
            }
            Voltage = Voltage/100;
          AirTemp = message.data[3]*256 + message.data[2];
            if(AirTemp > 32767) 
            {
              AirTemp = AirTemp - 65536;
            }
            AirTemp = AirTemp/10;
          CoolantTemp = message.data[5]*256 + message.data[4];
            if(CoolantTemp > 32767) 
            {
              CoolantTemp = CoolantTemp - 65536;
            }
            CoolantTemp = CoolantTemp/10;
          TempType = message.data[6];
          break;    
      } 
    }
  }
}

}
