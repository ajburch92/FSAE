// GPSExample.cpp - How to use the GPS code for Adafruit Ultimate GPS

/*
 * Copyright (C) 2014 by Richard Nash (KC3ARY)
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

#include <WProgram.h>
#include "GPS.h"
float time = 0;
HardwareSerial &gpsSerial = Serial1;
GPS gps(&gpsSerial,true);

// setup() method runs once, when the sketch starts
void setup()
{
  Serial.begin(38400); // For debugging output over the USB port
  Serial.print("beginning");
  gps.startSerial(9600);
  gps.sendCommand(PMTK_API_SET_SBAS_ENABLED, true, 500);
  gps.setUpdateRate(UPDATE_RATE_100);
  delay(1000);
  gps.setSentencesToReceive(OUTPUT_RMC_GGA);

}

// the loop() methor runs over and over again,
// as long as the board has power
void loop()
{
  if (gps.sentenceAvailable()) gps.parseSentence();

  if (gps.newValuesSinceDataRead()) {
    gps.dataRead();
    time = millis() / 1000;
          Serial.print(time);
    Serial.printf("  - Speed: %f, Location: %f, %f heading %f (Degrees 0-North, 90-East, 180-South, 270-West) \n\r",
		  gps.speed, gps.latitude, gps.longitude, gps.heading);

  }
}
