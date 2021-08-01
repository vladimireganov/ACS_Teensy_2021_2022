/**
 * @file gps.h
 * @author Kazybek Mizam (kzm0099@auburn.edu)
 * @brief GPS Sensor wrapper for teensy software.
 * @version 0.1
 * @date 2021-03-23
 * 
 * @copyright Copyright (c) AURA Embedded Systems 2021
 * 
 */

#ifndef GPS_MODULE
#define GPS_MODULE
#include <Adafruit_GPS.h>
#include <HardwareSerial.h>

/**
 * @brief GPS class that handles all gps processes,
 * and contains required methods that are used in controllers,
 * such as latitude and longitude
 */
class GPS {
    private:
    unsigned int gps_time = 0;
    Adafruit_GPS gps = Adafruit_GPS(&Serial5);
    bool connected = false;

    public:
    const int GPS_NORMAL_DELAY_MS = 2000;

    GPS(HardwareSerial *gserial) {
        gps = Adafruit_GPS(gserial);
    }

    void activate() {
        if (gps.begin(9600)) {
            gps.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
            gps.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
            #ifdef inotesting
            Serial.println("GPS is OK!");
            #endif
        }
        else {
            #ifdef inotesting
            Serial.println("GPS is not connected!");
            #endif
        }
    }

    void read() {
        for (int i = 0; i < 32 && gps.read() != 0; i++) {}
        if (gps.newNMEAreceived()) {
            gps.parse(gps.lastNMEA());
        }

        if (gps.fix) {
            connected = true;
        } else {
            connected = false;
        }
    }

    bool ready(unsigned int board_time) {
        if (gps_time < board_time) {
            gps_time = board_time + GPS_NORMAL_DELAY_MS;
            return true;
        }
        return false;
    }

    bool is_fixed() {
        return this->gps.fix;
    }

    bool is_working() {
        return connected;
    }

    double latitude() {
        // Serial.print(gps.latitude, 4); Serial.print(gps.lat);
        // Serial.print(", ");
        return this->gps.latitude;
    }

    double longitude() {
        // Serial.print(gps.longitude, 4); Serial.println(gps.lon);
        return this->gps.longitude;
    }

    void location(float *latitude, float *longitude, char *lat, char *lon) {
        *latitude = gps.latitude;
        *longitude = gps.longitude;
        *lat = gps.lat;
        *lon = gps.lon;
    }
};

#endif