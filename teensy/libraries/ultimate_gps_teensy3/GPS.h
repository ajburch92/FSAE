/*
 * Copyright (C) 2014 by KC3ARY Richard Nash
 * 
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#ifndef _GPS_H
#define _GPS_H

#include <WProgram.h>

// Return from "parseSentence()"
#define PARSED_NONE     0
#define PARSED_UNKNOWN  1
#define PARSED_ERROR    2
#define PARSED_GGA      3
#define PARSED_RMC      4
#define PARSED_ACK      5

// different commands to set the update rate from once per ten seconds (0.1 Hz) to 10 times a second (10Hz)
#define UPDATE_RATE_10000   "$PMTK220,10000*2F"
#define UPDATE_RATE_5000    "$PMTK220,5000*1B"
#define UPDATE_RATE_2000    "$PMTK220,2000*1C"
#define UPDATE_RATE_1000    "$PMTK220,1000*1F"
#define UPDATE_RATE_200     "$PMTK220,200*2C"
#define UPDATE_RATE_100     "$PMTK220,100*2F\r\n" //"$PMTK220,100*2F"


#define PMTK_API_SET_SBAS_ENABLED  "$PMTK313,1*0A\r\n"


// turn on only the second sentence (GPRMC)
#define OUTPUT_RMC      "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"
// turn on just GGA
#define OUTPUT_GGA      "$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"
// turn on GPRMC and GGA
#define OUTPUT_RMC_GGA  "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
// turn on ALL THE DATA
#define OUTPUT_ALLDATA  "$PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
// turn off output
#define OUTPUT_OFF      "$PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"

class GPS {
public:
    GPS(HardwareSerial * const, bool ); // Constructor
    void startSerial(const uint16_t);
    void poll(void);
    uint8_t parseSentence(void);
    bool sentenceAvailable(void);
    void sendCommand(const char*, bool, const uint32_t);

    void standby();
    void setUpdateRate(const char*);
    void setSentencesToReceive(const char*);

    void dataRead();
    bool newValuesSinceDataRead();
    
    // No accessors, just feel free to read this stuff directly.
    // Don't set it though
    uint32_t millisHeadingAcquired, millisAltitudeAcquired, millisPositionAcquired, millisDataRead;
    uint8_t hour, minute, seconds, year, month, day;
    float latitudeDegreesAndFractionalMinutes; 
    float longitudeDegreesAndFractionalMinutes;
    float latitude; // In decimal degrees (no funny degrees and fractional minutes)
    float longitude; // In degrees
    float altitude; // In meters
    float speed; // In knots
    uint16_t heading; // Degrees 0-North, 90-East, 180-South, 270-West
    boolean fix;
    uint8_t fixquality, satellites;

private:
    // Private Methods
    uint8_t parseGGA(const char*, const uint32_t);
    uint8_t parseRMC(const char*, const uint32_t);
    uint8_t parse(const char*, const uint32_t);
    void waitForAck(const uint32_t);

    // Private Members
    IntervalTimer pollTimer;
    bool runTimer;
    float degreesAndFractionalMinutesToDegrees(float, const char);
    HardwareSerial *  gpsSwSerial;

#ifdef SIMULATE
    long millisSinceLastSend;
    int simCount;
#endif
};


#endif
