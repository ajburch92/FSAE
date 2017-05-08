
#include <FlexCAN.h>

float lastSample;
float startTime;
int led = 13;
FlexCAN CANbus(500000);
static CAN_message_t msg,rxmsg;
static uint8_t hex[17] = "0123456789abcdef";



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


// -------------------------------------------------------------
static void hexDump(uint8_t dumpLen, uint8_t *bytePtr)
{
  uint8_t working;
  while( dumpLen-- ) {
    working = *bytePtr++;
    Serial.write( hex[ working>>4 ] );
    Serial.write( hex[ working&15 ] );
  }
  Serial.write('\r');
  Serial.write('\n');
}


// -------------------------------------------------------------
void setup(void)
{
  CANbus.begin();
  pinMode(led, OUTPUT);
  digitalWrite(led, 1);
  Serial.begin(9600);

  delay(1000);
  Serial.println(F("Teensy 3.1 CAN Test."));
  startTime = millis();
}


// -------------------------------------------------------------
void readMessage()
{
  Serial.print(millis() - startTime);
//  //Display Numerical Data over serial port  
//  Serial.print("RPM: ");
//  Serial.print(RPM); 
//  Serial.print(", ");
//  Serial.print("TPS: ");
//  Serial.print(TPS); 
//  Serial.print(", ");
//  Serial.print("FuelOpenTime: ");
//  Serial.print(FuelOpenTime);
//  Serial.print(", "); 
//  Serial.print("IgnitionAngle: ");
//  Serial.print(IgnitionAngle);
//  Serial.print(", ");
//  Serial.print("Barometer: ");  
//  Serial.print(Barometer);
//  Serial.print(", ");
//  Serial.print("MAP: ");
//  Serial.print(MAP); 
//  Serial.print(", ");
//  Serial.print("Lambda: ");    
//  Serial.print(Lambda); 
//  Serial.print(", ");
//  if(PressType == 0)
//  {
//    Serial.print("psi");
//  } 
//  else
//  {
//    Serial.print("kPa");
//  }
//  Serial.print(", ");
//  Serial.print("Oxygen: ");
//  Serial.print(Oxygen);
//  Serial.print(", ");
//  Serial.print("OilPress: ");
//  Serial.print(OilPress);
//  Serial.print(", ");
//  Serial.print("BrakePress: ");
//  Serial.print(BrakePress);
//  Serial.print(", "); 
//  Serial.print("SteerAngle: ");
//  Serial.print(SteerAngle);
//  Serial.print(", ");  
//  Serial.print("FinalDrive: ");
//  Serial.print(FinalDrive);
//  Serial.print(", ");  
//  Serial.print("WheelDriveR: ");
//  Serial.print(WheelSpeedR);
//  Serial.print(", ");
//  Serial.print("WheelDriveL: ");
//  Serial.print(WheelSpeedL);
//  Serial.print(", ");  
//  Serial.print("Voltage: ");
//  Serial.print(Voltage); 
//  Serial.print(", ");
//  Serial.print("AirTemp: ");
//  Serial.print(AirTemp); 
//  Serial.print(", ");
//  Serial.print("CoolantTemp: ");
//  Serial.print(CoolantTemp);
//  Serial.print(", ");
//  if(TempType == 0)
//  {
//    Serial.print('F');
//  } 
//  else
//  {
//    Serial.print('C');
//  }

  if (CANbus.available() == true) // Check to see if a valid message has been received.
  Serial.print("message available"); 
  {
    while ( CANbus.read(rxmsg) ) {
      //hexDump( sizeof(rxmsg), (uint8_t *)&rxmsg );
      Serial.print(millis() - startTime); 
      Serial.write(rxmsg.buf[0]);
      Serial.print (",  ");
      Serial.write(rxmsg.buf[1]);
      Serial.print (",  ");
      Serial.write(rxmsg.buf[2]);
      Serial.println("");
    }
  }
  Serial.print(", "); 
   Serial.print(millis() - startTime); 
//  // if not time-delayed, read CAN messages and print 1st byte
//  if ( !rxTimer ) {
//    while ( CANbus.read(rxmsg) ) {
//      hexDump( sizeof(rxmsg), (uint8_t *)&rxmsg );
//      Serial.write(rxmsg.buf[0]);
//      Serial.write(rxmsg.buf[1]);
//      Serial.write(rxmsg.buf[2]);
//      Serial.write(rxmsg.buf[3]);
//      Serial.write(rxmsg.buf[4]);
//      Serial.write(rxmsg.buf[5]);
//      Serial.write(rxmsg.buf[6]);
//      Serial.write(rxmsg.buf[7]);                        
//      Serial.write(rxmsg.buf[8]);
//      rxCount++;
//    }

    switch(rxmsg.id)
    {
//        /*All 2 Byte data is stored as [Low Byte, High Byte]
//          Num = HighByte*256 + LowByte
//          Conversion from 2 bytes to signed int:
//          Num = HighByte*256 + LowByte
//          if (Num >  32767) then Num = Num - 65536   
//          Note: The indices for the array should be one less
//                than the start positions specified in the PE3.
//                This is because arrays are indexed from 0 not 1 */
//        
        case 0x0CFFF048: //See PE3 pdf
          RPM = rxmsg.buf[1]*256 + rxmsg.buf[0];
          TPS = rxmsg.buf[3]*256 + rxmsg.buf[2];
            if(TPS > 32767) 
            {
              TPS = TPS - 65536;
            }
            TPS = TPS/10; //Why? Eliminating the decimal place 
          FuelOpenTime = rxmsg.buf[5]*256 + rxmsg.buf[4];
            if(FuelOpenTime > 32767) 
            {
              FuelOpenTime = FuelOpenTime - 65536;
            }
          IgnitionAngle = rxmsg.buf[7]*256 + rxmsg.buf[6];
            if(IgnitionAngle > 32767) 
            {
              IgnitionAngle = IgnitionAngle - 65536;
            }
          break;
        
        case 0x0CFFF148:
          Barometer = rxmsg.buf[1]*256 + rxmsg.buf[0];
            if(Barometer > 32767) 
            {
              Barometer = Barometer - 65536;
            }
            Barometer = Barometer/100;
          MAP = rxmsg.buf[3]*256 + rxmsg.buf[2];
            if(MAP > 32767) 
            {
              MAP = MAP - 65536;
            }
            MAP = MAP/100;
          Lambda = rxmsg.buf[5]*256 + rxmsg.buf[4];
            if(Lambda > 32767) 
            {
              Lambda = Lambda - 65536;
            }
            //Lambda = Lambda/100;
          PressType = rxmsg.buf[6];
          break;
        
        case 0x0CFFF248:
          Oxygen = rxmsg.buf[1]*256 + rxmsg.buf[0];
            if(Oxygen > 32767) 
            {
              Oxygen = Oxygen - 65536;
            }
            Oxygen = Oxygen/100; //Must be calibrated
          break;  
        
        case 0x0CFFF348:
          OilPress = rxmsg.buf[1]*256 + rxmsg.buf[0];
            if(OilPress > 32767) 
            {
              OilPress = OilPress - 65536;
            }
            OilPress = OilPress/100; //Must be calibrated
          SteerAngle = rxmsg.buf[3]*256 + rxmsg.buf[2];
            if(SteerAngle > 32767) 
            {
              SteerAngle = SteerAngle - 65536;
            }
            SteerAngle = SteerAngle/10; //Must be calibrated                
          BrakePress = rxmsg.buf[7]*256 + rxmsg.buf[6];
            if(BrakePress > 32767) 
            {
              BrakePress = BrakePress - 65536;
            }
            BrakePress = BrakePress/100; //Must be calibrated 
          break;  
        
        case 0x0CFFF448:
          FinalDrive = rxmsg.buf[1]*256 + rxmsg.buf[0];
            if(FinalDrive > 32767) 
            {
              FinalDrive = FinalDrive - 65536;
            }
            FinalDrive = FinalDrive/10; //Must be calibrated
          WheelSpeedR = rxmsg.buf[3]*256 + rxmsg.buf[2];
            if(WheelSpeedR > 32767) 
            {
              WheelSpeedR = WheelSpeedR - 65536;
            }
            WheelSpeedR = WheelSpeedR/10; //Must be calibrated      
          WheelSpeedL = rxmsg.buf[5]*256 + rxmsg.buf[4];
            if(WheelSpeedL > 32767) 
            {
              WheelSpeedL = WheelSpeedL - 65536;
            }
            WheelSpeedL = WheelSpeedL/10; //Must be calibrated              
          break;            
        
        case 0x0CFFF548:
          Voltage = rxmsg.buf[1]*256 + rxmsg.buf[0];
            if(Voltage > 32767) 
            {
              Voltage = Voltage - 65536;
            }
            Voltage = Voltage/100;
          AirTemp = rxmsg.buf[3]*256 + rxmsg.buf[2];
            if(AirTemp > 32767) 
            {
              AirTemp = AirTemp - 65536;
            }
            AirTemp = AirTemp/10;
          CoolantTemp = rxmsg.buf[5]*256 + rxmsg.buf[4];
            if(CoolantTemp > 32767) 
            {
              CoolantTemp = CoolantTemp - 65536;
            }
            CoolantTemp = CoolantTemp/10;
          TempType = rxmsg.buf[6];
          break;    
      } 
  Serial.println(); // adds a line
  }



void loop()
{
  
  if (((millis()- startTime) - lastSample) >= 100) {
  readMessage();
  lastSample = millis()-startTime;
  }
  
}

