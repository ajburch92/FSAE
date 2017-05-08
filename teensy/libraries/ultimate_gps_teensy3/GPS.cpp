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

#include "GPS.h"

#define PMTK_SET_BAUD_57600 "$PMTK251,57600*2C"
#define PMTK_SET_BAUD_9600 "$PMTK251,9600*17"
#define PMTK_SET_BAUD_4800 "$PMTK251,4800*14"

// standby command & boot successful message
#define PMTK_STANDBY "$PMTK161,0*28"
#define PMTK_STANDBY_SUCCESS "$PMTK001,161,3*3"
#define PMTK_AWAKE "$PMTK010,002*2D"

// ask for the release and version
#define PMTK_Q_RELEASE "$PMTK605*31"

// Max number of characters we can read in a GPS sentence
#define GPS_BUFFER_SIZE 100

#define POLL_TIME 100  // In microseconds, 1/10th of a seconds should be fast enough

// Double buffered interrupt driven reading
static char buf1[GPS_BUFFER_SIZE];
static char buf2[GPS_BUFFER_SIZE];
static char *onBuffer;
static int nCharsRead;
static char *bufferComplete;

// Constructor using SoftwareSerial
// Note: interruptDrivenPolling uses up an IntervalTimer, which will
// be running constantly.
// Note: There is a known race condition in this interrupt driven code
//       In the polling loop which is called at 10 Hz when the buffer fills
//       up, the state of the variable "bufferComplete" is assigned which
//       buffer just filled. That is also the flag for indicating a sentence
//       is ready for parsing. If you check that variable, make sure you
//       parse the sentence before the next one completes or the double
//       buffering could get corrupted.
// Note: If you are not using interruptDrivenPolling, then make sure you
// call the "poll" function fast enough to keep up with the incoming data.
// Ohterwise you can lose characters and get corrupted sentences.
//
// Finally: When using interruptDrivenPolling, just check for "sentenceAvailable"
// and call "parseSentence" faster than the next sentence coming available.
GPS::GPS(HardwareSerial * const ser, bool interruptDrivenPolling )
{
    runTimer = interruptDrivenPolling;
    bufferComplete = 0;
    onBuffer = buf1;
    nCharsRead = 0;
    hour = minute = seconds = year = month = day =
    fixquality = satellites = 0; // uint8_t
    fix = false; // boolean
    latitude = longitude = altitude = speed = 0.0; // float
    heading = 0;
    gpsSwSerial = ser;
    millisHeadingAcquired = 0;
    millisAltitudeAcquired = 0;
    millisPositionAcquired = 0;
#ifdef SIMULATE
    millisSinceLastSend = millis();
    simCount = 0;
#endif
}

void GPS::dataRead()
{
  millisDataRead = millis();
}

bool GPS::newValuesSinceDataRead()
{
  if (millisHeadingAcquired > millisDataRead && millisAltitudeAcquired > millisDataRead && millisPositionAcquired > millisDataRead) return true;
  return false;
}

static GPS *mainGPS; // The only GPS instance that gets callbacks. It's probably best to only have one
// Turn the function call into a method call on this class;
static void poll_fun()
{
  mainGPS->poll();
}

void GPS::startSerial(const uint16_t baud)
{
    gpsSwSerial->begin(baud);
    delay(10);
    sendCommand(OUTPUT_OFF, true, 500);
    if (runTimer) {
      delay(500);
      mainGPS = this;
      pollTimer.begin( poll_fun, POLL_TIME );
    }
}

void GPS::standby()
{
    sendCommand(PMTK_STANDBY, true, 500);
}

void GPS::setUpdateRate(const char*updateString)
{
    sendCommand(updateString, true, 500);
}

void GPS::setSentencesToReceive(const char*sentences)
{
    sendCommand(sentences, true, 500);
}

float GPS::degreesAndFractionalMinutesToDegrees(float deg, const char hemi)
{
    const float degPart = (int)(deg/100.0);
    deg = (deg - degPart*100.0) / 60.0 + degPart;
    if (hemi == 'S' || hemi == 'W') {
        return -deg;
    } else {
        return deg;
    }
}

uint8_t GPS::parseGGA(const char* nmea, const uint32_t now)
{
    // Assuming a GGA sentence
    const char* p = nmea;
    // get time
    p = strchr(p, ',')+1;
    float timef = atof(p);
    uint32_t time = timef;
    hour = time / 10000;
    minute = (time % 10000) / 100;
    seconds = (time % 100);
    
    // parse out latitude
    p = strchr(p, ',')+1;
    latitude = atof(p);
    
    p = strchr(p, ',')+1; // Should be 'N' or 'S'
    latitude = degreesAndFractionalMinutesToDegrees(latitude, p[0]);
    
    // parse out longitude
    p = strchr(p, ',')+1;
    longitude = atof(p);
    
    p = strchr(p, ',')+1; // Should be 'W' or 'E'
    longitude = degreesAndFractionalMinutesToDegrees(longitude, p[0]);
    
    p = strchr(p, ',')+1;
    fixquality = atoi(p);
    
    p = strchr(p, ',')+1;
    satellites = atoi(p);
    
    p = strchr(p, ',')+1; // HDOP, throw away
    
    p = strchr(p, ',')+1;
    altitude = atof(p);
    
    millisAltitudeAcquired = now;
    millisPositionAcquired = now;
    
    return PARSED_GGA;    
}

uint8_t GPS::parseRMC(const char* nmea, const uint32_t now)
{
    // found RMC
    const char* p = nmea;
    
    // get time
    p = strchr(p, ',')+1;
    float timef = atof(p);
    uint32_t time = timef;
    hour = time / 10000;
    minute = (time % 10000) / 100;
    seconds = (time % 100);
    
    p = strchr(p, ',')+1;
    fix = (p[0] == 'A');
    
    // parse out latitude
    p = strchr(p, ',')+1;
    latitude = atof(p);
    
    p = strchr(p, ',')+1; // Should be 'N' or 'S'
    latitude = degreesAndFractionalMinutesToDegrees(latitude, p[0]);
    
    // parse out longitude
    p = strchr(p, ',')+1;
    longitude = atof(p);
    
    p = strchr(p, ',')+1; // Should be 'W' or 'E'
    longitude = degreesAndFractionalMinutesToDegrees(longitude, p[0]);
    
    // speed
    p = strchr(p, ',')+1;
    speed = atof(p)* 0.868976242; // Speed in knots to mph
    
    // heading
    p = strchr(p, ',')+1;
    heading = (uint16_t)atof(p);
    
    p = strchr(p, ',')+1;
    uint32_t fulldate = atof(p);
    day = fulldate / 10000;
    month = (fulldate % 10000) / 100;
    year = (fulldate % 100);
    
    millisHeadingAcquired = now;
    millisPositionAcquired = now;
    
    return PARSED_RMC;
}

uint8_t GPS::parse(const char* nmea, const uint32_t now)
{
  //Serial.print("P: ");
  //Serial.println(nmea);
    if (*nmea != '$') {
        return PARSED_ERROR;
    }
    if (strstr(nmea, "$GPGGA") == nmea) {
        if (strlen(nmea) < 65) return PARSED_UNKNOWN; // No fix
        return parseGGA(nmea, now);
    } else if (strstr(nmea, "$GPRMC") == nmea) {
        if (strlen(nmea) < 65) return PARSED_UNKNOWN; // No fix
        return parseRMC(nmea, now);
    } else if (strstr(nmea, "$PMTK001") == nmea) {
        return PARSED_ACK;
    }
    return PARSED_UNKNOWN;
}

void GPS::sendCommand(const char* str, const bool wait, const uint32_t timeout) {
#ifdef SIMULATE
    // Do nothing if in simulation mode
#else
  //Serial.print("GPS(");
  //Serial.print(str);
  //Serial.println(")");
    gpsSwSerial->println(str);
    if (wait) waitForAck(timeout);
#endif
}

// Wait until the command sent gets an ACK back
void GPS::waitForAck(const uint32_t timeout)
{
    uint32_t giveUpTime = 0;
    if (timeout) {
        giveUpTime = millis()+timeout;
    }
    while (!giveUpTime || millis() < giveUpTime) {
        if (parseSentence() == PARSED_ACK) break;
    }
    
}

// Call to check for available characters
void GPS::poll(void)
{
  while (gpsSwSerial->available()) {
    const char c = gpsSwSerial->read();
    if (c != -1) {
        onBuffer[nCharsRead++] = c;
        if (nCharsRead == GPS_BUFFER_SIZE-1 || c == '\n') {
            onBuffer[nCharsRead] = 0;
            nCharsRead = 0;
            if (onBuffer == buf1) {
                onBuffer = buf2;
                bufferComplete = buf1;
            } else {
                onBuffer = buf1;
                bufferComplete = buf2;
            }
        }
    }
  }
}

 // Has a sentence been made available?
bool GPS::sentenceAvailable(void)
{
  return bufferComplete != NULL;
}

// Call this from loop to parse any available sentences
uint8_t GPS::parseSentence(void)
{
    uint8_t retVal = PARSED_NONE;
    if (sentenceAvailable()) {
      retVal = parse(bufferComplete, millis());
      bufferComplete = 0;
    }
    return retVal;
}

