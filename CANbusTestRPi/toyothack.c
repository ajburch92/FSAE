#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <curses.h>
#include <endian.h>

#define __packed __attribute__((packed))

static int sk;

enum frame_ids {
	PE1 = 0x8CFFF048,
	PE2 = 0x8CFFF148,
    	PE3 = 0x8CFFF248,
	PE4 = 0x8CFFF348,
    	PE5 = 0x8CFFF448,
	PE6 = 0x8CFFF548,
    	PE7 = 0x8CFFF648,
};

#define UNKNOWN_COUNT 1024
static int unknown[UNKNOWN_COUNT];

union toyoframe {
	struct __packed {
		uint16_t RPM;
		int16_t TPS;
		int16_t fuelOpenTime;
		int16_t ignitionAngle;
	} PE1data;
	struct __packed {
		int16_t barometer;
		int16_t MAP;
		int16_t lambda;
		unsigned char pressureType;
	} PE2data;
	struct __packed {
		int16_t analogIN5;
		int16_t analogIN6;
		int16_t analogIN7;
		int16_t analogIN8;
	} PE4data;
	struct __packed {
		int16_t voltage;
		int16_t airTemp;
		int16_t ETC;
		unsigned char tempType;
	} PE6data;
};

static void unknown_frame(int id)
{
	int i;
    
	for (i = 0; i < UNKNOWN_COUNT; i++)
		if (unknown[i] == 0 || unknown[i] == id)
			break;
	if (i == UNKNOWN_COUNT)
		return;
    
	unknown[i] = id;
    
	//move(LINES - 3, 1);
	//clrtoeol();
	printf("unk:");
	for (i = 0; i < UNKNOWN_COUNT; i++) {
		if (unknown[i] == 0)
			break;
		printf(" %02x", unknown[i]);
	}
	printf(" (%d)", i);
}

static void process_one(struct can_frame *frm)
{
	int i;
	union toyoframe *toy;
    
	toy = (union toyoframe *)frm->data;
    
	switch (frm->can_id) {
            
            /*All 2 Byte data is stored as [Low Byte, High Byte]
             Num = HighByte*256 + LowByte
             Conversion from 2 bytes to signed int:
             Num = HighByte*256 + LowByte
             if (Num >  32767) then Num = Num - 65536
             Note: The indices for the array should be one less
             than the start positions specified in the PE3.
             This is because arrays are indexed from 0 not 1 */
            
        case PE1:
	printf("PE1: RPM=%6d TPS=%4d fuelOT=%3d ignitionAngle=3%d",
            be16toh(toy->PE1data.RPM),
            (toy->PE1data.TPS/10),
            be16toh(toy->PE1data.fuelOpenTime),
            be16toh(toy->PE1data.ignitionAngle));
            
            break;
            
            /*
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
             */
            
        case PE2:
            i = (frm->can_id == PE1) ? 1 : 2;
            //move(i, 1);
            //clrtoeol();
            printf( "PE2: bar=%5d MAP=%5d lambda=%5d pressureType=%02x",
                     be16toh(toy->PE2data.barometer),
                     be16toh(toy->PE2data.MAP),
                     be16toh(toy->PE2data.lambda),
                     toy->PE2data.pressureType);
            break;
            
            /*Barometer = message.data[1]*256 + message.data[0];
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
             */
            
        case PE4:
            //move(3, 1);
            //clrtoeol();
            printf( "PE4: Analog5=%4d Analog6=%4d Analog7=%4d Analog8=%4d",
                     (toy->PE4data.analogIN5/1000),
                     (toy->PE4data.analogIN6/1000),
                     (toy->PE4data.analogIN7/1000),
                     (toy->PE4data.analogIN8/1000)
                     );
            break;
            
            /*
             Oxygen = message.data[1]*256 + message.data[0];
             if(Oxygen > 32767)
             {
             Oxygen = Oxygen - 65536;
             }
             Oxygen = Oxygen/100; //Must be calibrated
             break;
             */
            
        case PE6:
            //move(6, 1);
            //clrtoeol();
            printf( "PE6: Voltage=%f airTemp=%3d ETC=%3d, tempType=%3hhd",
                     (toy->PE6data.voltage/100.0),
                     (toy->PE6data.airTemp/10),
                     (toy->PE6data.ETC/10),
                     toy->PE6data.tempType
                     );
            break;
            
            /*
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
             */
            
        default:
            unknown_frame(frm->can_id);
		printf ("UNKNOWN CAN ID");            
//            refresh();
    }
}
    
    int net_init(char *ifname)
    {
        int recv_own_msgs;
        struct sockaddr_can addr;
        struct ifreq ifr;
        
        sk = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (sk < 0) {
            perror("socket");
            exit(1);
        }
        
        memset(&ifr.ifr_name, 0, sizeof(ifr.ifr_name));
        strncpy(ifr.ifr_name, ifname, IFNAMSIZ);
        if (ioctl(sk, SIOCGIFINDEX, &ifr) < 0) {
            perror("SIOCGIFINDEX");
            exit(1);
        }
        
        memset(&addr, 0, sizeof(addr));
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        if (bind(sk, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
            perror("bind");
            return 1;
        }
        
        recv_own_msgs = 0; /* 0 = disabled (default), 1 = enabled */
        setsockopt(sk, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS,
                   &recv_own_msgs, sizeof(recv_own_msgs));
        
        return 0;
    }
    
    void receive_one(void)
    {
        struct can_frame frm;
        struct sockaddr_can addr;
        int ret;
        socklen_t len;
        
        ret = recvfrom(sk, &frm, sizeof(struct can_frame), 0,
                       (struct sockaddr *)&addr, &len);
        if (ret < 0) {
            perror("recvfrom");
            exit(1);
        }
        
        process_one(&frm);
	printf("\n");
    }
    
    int main(int argc, char **argv)
    {
        if (argc != 2) {
            printf("syntax: %s IFNAME\n", argv[0]);
            exit(1);
        }
        
        memset(unknown, 0, sizeof(unknown));
        
  //      initscr();
        
        net_init(argv[1]);
        
        for (;;)
            receive_one();
        
  //      endwin();
        
        return 0;
    }
