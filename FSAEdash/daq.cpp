#include "daq.h"
#include <QtCore>
#include <QFileInfo>

#include "MAIN2.h"
#include "structures.h"

#define ISR_PIN 6

#define __packed __attribute__((packed))
static int sk;


enum frame_ids {
    PE1 = 0x8CFFF048,
    PE2 = 0x8CFFF148,
    PE3 = 0x8CFFF248,
    PE4 = 0x8CFFF348,
    PE5 = 0x8CFFF448,
    PE6 = 0x8CFFF548,
    PE7 = 0x8CFFF648
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



i2cBus i2c;
RTC rtc;
ADC adc;
IMU imu;
GPS gps;
Adafruit_INA219 ina219;
//STORAGE storage; //duplicate in the main.cpp file

ADCdata ADCdata;
ECUdata ECUdata;
RTCdata RTCdata;
IMUdata IMUdata;
THARdata THARdata;
GPSdata GPSdata;
INAdata INAdata;
DASHdata DASHdata;


void interrupt();
int count2 = 0;
int countOverload = 0;
int countMin = 0;
int speed = 0;
int countDash =0;
float speedCumsum =0;
float filteredxAccel = 0;



int rpm =0;
int tps = 0;
int fot = 0;
int iga = 0;
int bar = 0;
int mapp = 0;
int lam = 0;
int pty = 0;
int an5 = 0;
int an6 = 0;
int an7 = 0;
int an8 = 0;
int bat = 0;
int air = 0;
int etc = 0;
int tty = 0;



DAQ::DAQ(QObject *parent) :
    QThread(parent)
{
    printf("starting daq thread...\n");
    printf("setting up can comm\n");
    char *arg[] = {"test" , "can0"};
    memset(unknown, 0, sizeof(unknown));
    net_init(arg[1]);
    printf("setting up current sensor\n");
    ina219.setup();
    printf("setting up accelerometers\n");
    imu.setupAccel();
    printf("setting up gyros\n");
    imu.setupGyro();
    printf("getting time stamp...\n");
    rtc.getTimeStamp();
    printf("setting up storage file...\n");
    storageInit();
    printf ("starting square wave...\n");
    rtc.initSKW(DS3231_1kHz);
    delay(500);
    printf ("beginning interrupts ...\n ") ;
    wiringPiSetup();
    wiringPiISR(ISR_PIN, INT_EDGE_RISING, &interrupt);

}


void DAQ::run()
{
    while (1) {

        QMutex mutex;


        receive_one();


        if (countOverload >= 100) {
	    countOverload = 0;
            printf("dumping data\n");
            saveData();

        }
        mutex.lock();
        if (this->StopThread) break;
        mutex.unlock(); //WHEN DO I NEED THESE

        if (countDash >= 5) {
            emit onInterrupt(speed, rpm,etc,bat,tps); // emit RPM, Battery, AFR, ETC, Speed,
            countDash = 0;
            printf("updating dash\n");
        }
    }

}

void interrupt () {
    //
    //
    countDash++;
    countMin++;
    countOverload++;
    count2++;
    printf("\ninterrupt");
    printf(" %i ",  countOverload);
    printf(" %i ",  countMin);

    //
    printf("getting adcs.1");
    ADCdata.steerAngle[countOverload] = adc.getValue(1);
    ADCdata.airSpeed[countOverload] = adc.getValue(2);
    ADCdata.brakePressB[countOverload] = adc.getValue(3);
    ADCdata.fuelTankLevel[countOverload] = adc.getValue(4);
    printf(".2");
    ADCdata.shockDispFL[countOverload] = adc.getValue(5);
    ADCdata.shockDispFR[countOverload] = adc.getValue(6);
    ADCdata.shockDispBR[countOverload] = adc.getValue(7);
    ADCdata.shockDispBL[countOverload] = adc.getValue(8);
    printf(".3");
    ADCdata.channel9[countOverload] = adc.getValue(9);
    ADCdata.channel10[countOverload] = adc.getValue(10);
    ADCdata.channel11[countOverload] = adc.getValue(11);
    ADCdata.channel12[countOverload] = adc.getValue(12);
    printf("getting accel ");
    IMUdata.xAccel[countOverload] = imu.getAccelValue(1);
    IMUdata.yAccel[countOverload] = imu.getAccelValue(2);
    IMUdata.zAccel[countOverload] = imu.getAccelValue(3);
    printf("getting gyro");
    IMUdata.xRot[countOverload] = imu.getGyroValue(1);
    IMUdata.yRot[countOverload] = imu.getGyroValue(2);
    IMUdata.zRot[countOverload] = imu.getGyroValue(3);
    IMUdata.xRot[countOverload] = (atan2( sqrt(IMUdata.yAccel[countOverload]*IMUdata.yAccel[countOverload] + IMUdata.xAccel[countOverload]*IMUdata.xAccel[countOverload]), IMUdata.yAccel[countOverload]) * 180/M_PI);
    IMUdata.yRot[countOverload] = atan2( sqrt(IMUdata.xAccel[countOverload]*IMUdata.xAccel[countOverload] + IMUdata.zAccel[countOverload]*IMUdata.zAccel[countOverload]), IMUdata.yAccel[countOverload]) * 180/M_PI;
    IMUdata.yRot[countOverload] = atan2( sqrt(IMUdata.xAccel[countOverload]*IMUdata.xAccel[countOverload] + IMUdata.zAccel[countOverload]*IMUdata.zAccel[countOverload]), IMUdata.yAccel[countOverload]) * 180/M_PI;
    printf("getting ina");
    INAdata.brakePressF[countOverload] = ina219.getValue();
    
    
    if (countMin >= 5) {
        printf("getting GPS");
        if (countOverload >= 5)
        {
            filteredxAccel = (IMUdata.xAccel[countOverload] + IMUdata.xAccel[countOverload-1]* .8+ IMUdata.xAccel[countOverload-2]* .6+ IMUdata.xAccel[countOverload-3]* .4+ IMUdata.xAccel[countOverload-4]*.2  )/5;
        }
        else {
            filteredxAccel = IMUdata.xAccel[countOverload];
        }
        
        GPSdata.speed[countOverload] = gps.getValue(0x01);
        GPSdata.longitude[countOverload] = gps.getValue(0x02);
        GPSdata.latitude[countOverload] = gps.getValue(0x03);


        GPSdata.speedAccel[countOverload] = speedCumsum + 0.05*filteredxAccel;
        speedCumsum = speedCumsum + GPSdata.latitude[countOverload];
        speed= GPSdata.speedAccel[countOverload];///////
        if (speed < 0) { speed = 0;}
        printf("speed %i " , speed);

        DASHdata.rpm[countOverload] = rpm;
        DASHdata.etc[countOverload] = etc;
        DASHdata.bat[countOverload] = bat;

        countMin = 0;

    }


}




// *******************************************************************
// Functions for STORAGE Class
// *******************************************************************

//------------------initialization------------------
/*
 * sets up storage file and initializes datasets
 *
 * @return error_flag as boolean
 */

void DAQ::storageInit(){

    //ofstream myfile;
    // myfile.open ("Data.txt");
    //myfile << "\n\n\n";
    //myfile.close();
    //return 0;
    int i=0;
    int run = 1;
    FILE *stream;
    //    char filename[] = "DAQdata1.txt";
    //    if (fexists(RTCdata.filename) >= 1) {
    //do nothing
    //printf("textfile exhists");
    //}
    //else {

    std::string startTime_minString = std::to_string(RTCdata.startTime_min);
    std::string startTime_hourString = std::to_string(RTCdata.startTime_hour);
    std::string startTime_dayString = std::to_string(RTCdata.startTime_day);
    std::string startTime_monthString = std::to_string(RTCdata.startTime_month);
    static const char* const chars[] = {"DAQdata01.txt", "DAQdata02.txt", "DAQdata03.txt", "DAQdata04.txt", "DAQdata05.txt", "DAQdata06.txt", "DAQdata07.txt", "DAQdata08.txt", "DAQdata09.txt", "DAQdata10.txt", "DAQdata11.txt", "DAQdata12.txt", "DAQdata13.txt", "DAQdata14.txt", "DAQdata15.txt", "DAQdata16.txt", "DAQdata17.txt", "DAQdata18.txt", "DAQdata19.txt", "DAQdata20.txt","DAQdata21.txt", "DAQdata22.txt", "DAQdata23.txt", "DAQdata24.txt", "DAQdata25.txt", "DAQdata26.txt", "DAQdata27.txt", "DAQdata28.txt", "DAQdata29.txt", "DAQdata30.txt", "DAQdata31.txt", "DAQdata32.txt", "DAQdata33.txt", "DAQdata34.txt", "DAQdata35.txt", "DAQdata36.txt", "DAQdata37.txt", "DAQdata38.txt", "DAQdata39.txt", "DAQdata40.txt"};

    RTCdata.filename = "DAQdata" + startTime_monthString + "  "  + startTime_dayString +startTime_hourString + " " + startTime_hourString + ".txt";


    //const char *c = RTCdata.filename.c_str();

    while (run >= 1 ) {
        if (fileExists(chars[i]) == true) {
            printf("textfile exhists");
            //RTCdata.filename = "DAQdata" + startTime_monthString + "  "  + startTime_dayString +startTime_hourString + " " + startTime_hourString + startTime_minString + std::to_string(i) + ".txt";
            //const char * c = RTCdata.filename.c_str();
            i++;
        }
        else {
            printf("no exhisting file");
            run = 0;
        }
    }
    std::string s = chars[i];
    RTCdata.filename = s;
    stream = fopen(chars[i],"a+"); // adjust to format title as DAQdata_*date*
    fprintf(stream, "%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s\n", "ADC.steerAngle", "ADC.airSpeed", "ADC.brakePressB", "ADC.fuelTankLevel", "ADC.shockDispFL", "ADC.shockDispFR", "ADC.shockDispBR", "ADC.shockDispBL", "ADC.channel9", "ADC.channel10", "ADC.channel11", "ADC.channel12", "INA.brakePressF", "IMU.xAccel", "IMU.yAccel", "IMU.zAccel", "IMU.xRot", "IMU.yRot", "IMU.zRot", "IMU.temp", "ECU.RPM", "ECU.ECT", "ECU.batVolt", "GPS.speed", "GPS.speedAccel", "GPS.latitude", "GPS.longitude");
    fclose (stream);

    printf("txt file initialized\n");
    //}

}


//------------------Save Data Set------------------
/*
 * saves the current data set to sd card's text file
 *
 * @return error_flag as boolean
 */

void DAQ::saveData(){

    countOverload = 0;
    FILE *stream;
    k=0;
    //char filename2[] = "DAQdata1.txt";
    for (k=0; k<=99; k++ ) {
        printf("%i", k);
        //    if (fexists(filename2) >=1) {
        //	stream = fopen("DAQdata2.txt","a+");
        //    }
        //    else {
        //stream = fopen("DAQdata1.txt","a+"); // adjust to format title as DAQdata_*date*
        //const char * c = RTCdata.filename.c_str();

        const char *a = RTCdata.filename.c_str();

        stream = fopen(a,"a+"); // adjust to format title as DAQdata_*date*

        //    }

        fprintf(stream, "%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %i, %i,%i, %i, %i,%i,%i\n", ADCdata.steerAngle[k], ADCdata.airSpeed[k], ADCdata.brakePressB[k], ADCdata.fuelTankLevel[k], ADCdata.shockDispFL[k], ADCdata.shockDispFR[k], ADCdata.shockDispBR[k], ADCdata.shockDispBL[k], ADCdata.channel9[k], ADCdata.channel10[k], ADCdata.channel11[k], ADCdata.channel12[k], INAdata.brakePressF[k], IMUdata.xAccel[k], IMUdata.yAccel[k], IMUdata.zAccel[k], IMUdata.xRot[k], IMUdata.yRot[k], IMUdata.zRot[k], GPSdata.speed[k], DASHdata.rpm[k], DASHdata.etc[k], DASHdata.bat[k], GPSdata.speed[k], GPSdata.speedAccel[k], GPSdata.latitude[k], GPSdata.longitude[k]);
        fclose(stream);
    }

    //printf("clearing gps");
    //GPSdata.GPS_sentenceVector.clear(); //clear gps vector
    printf("data save complete\n");
}

bool DAQ::fileExists(QString path) {
    QFileInfo check_file(path);
    //check if file exhists and if yes, is it really a file and no directory?
    if (check_file.exists() && check_file.isFile())  {
        return true;
    }
    else {
        return false;
    }
}

//*******************************************************************
//Functions for ECU
//*******************************************************************



void DAQ::unknown_frame(int id)
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

void DAQ::process_one(struct can_frame *frm)
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
               (toy->PE1data.RPM),
               (toy->PE1data.TPS/10),
               (toy->PE1data.fuelOpenTime),
               (toy->PE1data.ignitionAngle));
        rpm = toy->PE1data.RPM;
        tps = toy->PE1data.TPS/10;
        fot = (toy->PE1data.fuelOpenTime);
        iga = (toy->PE1data.ignitionAngle);
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
                (toy->PE2data.barometer),
                (toy->PE2data.MAP),
                (toy->PE2data.lambda),
                toy->PE2data.pressureType);
        bar = toy->PE1data.RPM;
        mapp = toy->PE1data.TPS/10;
        lam = (toy->PE1data.fuelOpenTime);
        pty = (toy->PE1data.ignitionAngle);
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

        an5 = toy->PE4data.analogIN5/1000;
        an6 = toy->PE4data.analogIN6/1000;
        an7 = (toy->PE4data.analogIN7/1000);
        an8 = (toy->PE4data.analogIN8/1000);
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
        bat = toy->PE6data.voltage/100;
        air = toy->PE6data.airTemp/10;
        etc = toy->PE6data.ETC/10;
        tty = toy->PE6data.tempType;

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
    }
}

int DAQ::net_init(char *ifname)
{
    int recv_own_msgs;
    struct sockaddr_can addr;
    struct ifreq ifr;

    sk = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sk < 0) {
        perror("socket");
        return 1;
        //    exit(1);
    }

    memset(&ifr.ifr_name, 0, sizeof(ifr.ifr_name));
    strncpy(ifr.ifr_name, ifname, IFNAMSIZ);
    if (ioctl(sk, SIOCGIFINDEX, &ifr) < 0) {
        perror("SIOCGIFINDEX");
        return 1;
        //     exit(1);
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

void DAQ::receive_one(void)
{
    struct can_frame frm;
    struct sockaddr_can addr;
    int ret;
    socklen_t len;



    ret = recvfrom(sk, &frm, sizeof(struct can_frame), 0,
                   (struct sockaddr *)&addr, &len);




    if (ret < 0) {
        perror("recvfrom");
    }

    process_one(&frm);
    printf("\n");
}

