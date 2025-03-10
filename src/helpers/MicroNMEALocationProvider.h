#pragma once

#include "LocationProvider.h"
#include <MicroNMEA.h>
#include <RTClib.h>

class MicroNMEALocationProvider : public LocationProvider {
    char _nmeaBuffer[100];
    MicroNMEA nmea;
    Stream* _gps_serial;

public :
    MicroNMEALocationProvider(Stream& ser) : _gps_serial(&ser), nmea(_nmeaBuffer, sizeof(_nmeaBuffer)) {
    }

    void reset() override {}
    long getLatitude() override { return nmea.getLatitude(); }
    long getLongitude() override { return nmea.getLongitude(); }
    bool isValid() override { return nmea.isValid(); }

    long getTimestamp() override { 
        DateTime dt(nmea.getYear(), nmea.getMonth(),nmea.getDay(),nmea.getHour(),nmea.getMinute(),nmea.getSecond());
        return dt.unixtime();
    } 

    void loop() override {
        while (_gps_serial->available()) {
            char c = _gps_serial->read();
            #ifdef GPS_NMEA_DEBUG
            Serial.print(c);
            #endif
            nmea.process(c);
        }
    }
};