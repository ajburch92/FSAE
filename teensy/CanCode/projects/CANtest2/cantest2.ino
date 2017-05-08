// Flexcan CANBus libray for Teensy can be found at https://github.com/teachop/FlexCAN_Library
// Teensy pins 3/4 (canbus Tx/Rx) go to separate canbus transceiver Rx/Tx. Transceiver CAN-H/CAN-L go to Solo CAN-H/CAN-L.
#include <FlexCAN.h>

// Specify CAN Baudrate. Currently 125k, 250k, 500k, 1M are supported in teensyduino 1.20
int baud = 250000;

int led = 13;
FlexCAN CANbus(baud);
static CAN_message_t rxmsg;

void setup(void)
{
  Serial.begin(9600);
  while ( ! Serial) ; // wait for Arduino Serial Monitor
  Serial.println("Teensy 3.1 CANlisten Example"); 
  
  CANbus.begin();
  
  //Light on to indicate on-bus state
  pinMode(led, OUTPUT);
  digitalWrite(led, 1);
}

void loop(void){
  
  //Poll Rx FIFO
  while ( CANbus.read(rxmsg) ) {
    //when message available

    //light off to indicate FIFO is not being polled
    digitalWrite(led, 0);

    //construct string  
    String text = "ID: 0x";
    text = String(text + String(rxmsg.id, HEX));
    text = String(text + " DLC: ");
    text = String(text + String(rxmsg.len, HEX));

    //check if DLC is >0 append string as required
    if (rxmsg.len >= 1)
    {
      text = String(text + " DATA: ");
    }
  
    //construct string for available bytes (trunctate to DLC to avoid reading garbage)
    for( int idx=0; idx < rxmsg.len; ++idx )
    {
       text = String(text + String(rxmsg.buf[idx],HEX));
       text = String(text + " ");
    }
  
    //print result
    Serial.println(text);

    //LED back on to indicate watching Rx FIFO  
    digitalWrite(led, 1); 
  } 
}
