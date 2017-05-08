#ifndef STRUCTURES_H
#define STRUCTURES_H

#include <vector>
#include <string>

using namespace std;

struct ADCdata {
    float steerAngle[1024]; // channel 1
    float airSpeed[1024]; // channel 2
    float brakePressB[1024]; // channel 3
    float fuelTankLevel[1024]; // channel 4
    float shockDispFL[1024]; // channel 5
    float shockDispFR[1024]; // channel 6
    float shockDispBR[1024]; // channel 7
    float shockDispBL[1024]; // channel 8
    float channel9[1024];
    float channel10[1024];
    float channel11[1024];
    float channel12[1024];
};

struct INAdata {
    float brakePressF[1024]; // current sensor
};

struct ECUdata {

    short engineTemp[1024];
    short waterTemp[1024];
    short rpm[1024];
    short throttlePos[1024];
    short oilTemp[1024];
    short oilPress[1024];
    short lambda[1024];

};

struct RTCdata {

    short interval[1024];
    short RTCcount[1024];
    short startTime[1024];// [7]?
    short elapsedTime[1024];
    int timeStamp[7];//[7]
    int startTime_min;
    int startTime_hour;
    int startTime_day;
    int startTime_month;
    std::string filename;

};

struct IMUdata {
    float xAccel[1024];
    float yAccel[1024];
    float zAccel[1024];
    float xRot[1024];
    float yRot[1024];
    float zRot[1024];
    float temp[1024];

};

struct THARdata {

};

struct GPSdata {
    //std::vector <std::string> GPSRMC_sentence; // 1024 strings with 56 char and a null terminator. ... use stirng copy ... strcpy(destination, sentance);
    vector<string> GPS_sentenceVector;

    //char GPSRMC[1024][56];
    //int speed[1024];
    float speed [1024];
    uint8_t latitude [1024];
    uint8_t longitude [1024];

};





#endif // STRUCTURES_H
