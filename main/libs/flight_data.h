/**
 * @file flight_data.h
 * @author Kazybek Mizam (kzm0099@auburn.com)
 * @brief Flight Data library
 * Library of Data controller class that is teensy for fullscale 2021 flights.
 * This library handles all data manipulation and its usage.
 * 
 * Uses SD library to work with file for data logging.
 * Uses Sensor library wrappers BMP, BNO, GPS to get data from them.
 * Uses flight_math.h library for data processing.
 * Uses fallback_packet_t from beagle_serial.h to update its values that are transmitter to
 * BeagleBone Blue
 * Uses teensy_data_t from flight_data_packet that are transmitted to ground station
 * 
 * @version 0.1
 * @date 2021-03-19
 * 
 * @copyright Copyright (c) AURA Embedded Systems 2021
 * 
 */
#ifndef FLIGHT_DATA_H
#define FLIGHT_DATA_H
#include <SD.h> // File
#include "sensors/bmp390.h"
#include "sensors/bno055.h"
#include "sensors/gps.h"
#include "flight_math.h"
#include "communications/beaglebone/beagle_serial.h"
#include "communications/ground_station/radio_packet.h"
#include "data/flight_data_packet.h"

/**
 * @brief Data class handles all data proccesses in flight
 * such as reciveing fresh data from sensors, processing to useful data
 * and logging.
 */
class Data
{
    private:
    // Data logging variables
    File backup_data_file;   // File instance of backup data
    teensy_data_t log_data;
    int log_data_size;

    BNO * bno;          // Instance of initialized BNO in Data class
    BMP390 * bmp;       // Instance of initialized BMP390 in Data class
    GPS * gps;      // Instance of initialized Radio in Data class
    fallback_packet_t * teensy_fallback_packet;
    realtime_data_packet_t * realtime_data_packet;

    public:
    int iterator;       // Data number

    // Below here raw data from BMP altimiter/temp sensor
    bool available = false;     // Whether data was updated and ready to be written in sd card
    bool bmp_available = false; // Whether bmp data was updated to fresh one
    bool bno_available = false; // Whether bno data was updated to fresh one

    double pressure;            // Pressure read from BMP sensor
    double temperature;         // Tempurature read from BMP sensor

    // Below here raw data from BNO 9DOF sensor
    imu::Vector<3> linear_acceleration; // 3-axis Linear acceleration from BNO sensor
    imu::Quaternion quaternions;        // Quaternions from BNO sensor
    imu::Vector<3> magnetometer;        // 3-axis Magnetometer from BNO sensor
    imu::Vector<3> gyroscope;           // 3-axis Gyroscope from BNO sensor
    imu::Vector<3> euler;               // Euler angles from BNO sensor
    imu::Vector<3> acceleration;        // 3-axis Acceleration (including gravity) from BNO sensor
    imu::Vector<3> gravity;             // 3-axis Gravity from BNO sensor

    // Below here data from GPS
    float latitude;     // Latitude value from GPS
    float longitude;    // Longitude value from GPS
    char lat;           // Latitude direction from GPS
    char lon;           // Longitude direction from GPS

    // Below here processed data 
    double ground_altitude;     // Ground altitude recorded when armed
    double prev_altitude;       // Previous recorded altitude
    double altitude;            // Current recorded altitude

    double apogee;        // Apogee altitude recorded
    unsigned long apogee_time;   // Time when apogee altitude recorded

    double land_altitude;       // Altitude of landed rocket
    unsigned long land_time;     // Time when rocket landed

    double prev_vertical_acceleration;  // Previous calculated vertical acceleration
    double vertical_acceleration;       // Current calculated vertical acceleration

    int relative_vertical_direction;        // Direction of BNO sensor when activated
    double relative_vertical_acceleration;  // Relative vertical accelration of rocket (direction up relative to BNO)

    double prev_total_linear_acceleration;  // Previous calculated total acceleration (without gravity)
    double total_linear_acceleration;       // Current calculated total acceleration (without gravity)

    double prev_vertical_velocity;  // Previous calculated vertical velocity
    double vertical_velocity;       // Current vertical velocity

    unsigned long bno_fixtime;    // Time of previous recorded data
    unsigned long previous_altitude_fixtime;    // Time of previous recorded data
    unsigned long current_altitude_fixtime;    // Time of previous recorded data
    unsigned long iteration_time;         // Time of current recorded data

    unsigned int elapsed;           // Elapsed time between current and previous recorded data in seconds

    double projected_altitude;      // Calculated projected altitude of rocket

    Data(BNO * bno, BMP390 * bmp, GPS *gps, fallback_packet_t * teensy_fallback_packet, realtime_data_packet_t * realtime_data_packet)
    {
        this->bno = bno;
        this->bmp = bmp;
        this->gps = gps;
        this->teensy_fallback_packet = teensy_fallback_packet;
        this->realtime_data_packet = realtime_data_packet;
        log_data_size = sizeof(teensy_data_t);

        latitude = 7.7;
        longitude = 8.8;
        lat = 'N';
        lon = 'W';
        log_data.latitude = latitude;
        log_data.longitude = longitude;

        apogee = -1000.0;
    }

    /**
     * @brief Function reset previous apogee recordings to current one
     * Because sometimes when sensor just started it sends bad values.
    */
    void reset_apogee() {
        apogee = altitude;
        apogee_time = current_altitude_fixtime;
    }

    /**
     * @brief Function that set time for apogee time. 
     * It is needed to start fixing apogee after time set.
     * 
     */
    void prepare_for_apogee()
    {
        apogee_time = current_altitude_fixtime;
    }
    

    /**
     * @brief Function that set time for land time. 
     * It is needed to start fixing landed event after time set.
     * 
     */
    void prepare_for_land()
    {
        land_time = current_altitude_fixtime;
    }

    /**
     * @brief Function that current data instance
     * with fresh data from IMU Sensors.
     */
    void get()
    {
        // Each time we call get means one loop iteration happened
        log_data.iteration_time = millis();
        gps->read(); // Parse gps data

        if(bmp->ready(millis()))
        {
            pressure = bmp->pressure();
            current_altitude_fixtime = millis();
            temperature = bmp->temperature();
            available = true;
            bmp_available = true;

            log_data.bmp_fixtime = current_altitude_fixtime;
            log_data.pressure = pressure;
            log_data.temperature = temperature;
        }
        
        if(bno->ready(millis()))
        {
            linear_acceleration = bno->getLinearAcc();
            quaternions = bno->getQutornions();
            gyroscope = bno->getGyro();
            euler = bno->getEuler();
            acceleration = bno->getAccel();
            gravity = bno->getGrav();
            bno_fixtime = millis();
            available = true;
            bno_available = true;

            log_data.bno_fixtime = bno_fixtime;

            log_data.linear_acceleration_x = linear_acceleration[0];
            log_data.linear_acceleration_y = linear_acceleration[1];
            log_data.linear_acceleration_z = linear_acceleration[2];

            log_data.quaternions_w = quaternions.w();
            log_data.quaternions_i = quaternions.x();
            log_data.quaternions_j = quaternions.y();
            log_data.quaternions_k = quaternions.z();

            log_data.acceleration_x = acceleration[0];
            log_data.acceleration_y = acceleration[1];
            log_data.acceleration_z = acceleration[2];

            log_data.gravity_x = gravity[0];
            log_data.gravity_y = gravity[1];
            log_data.gravity_z = gravity[2];
        }

        if (bno->ready_magnit(millis()))
        {
            magnetometer = bno->getMagnit();

            log_data.magnetometer_x = magnetometer[0];
            log_data.magnetometer_y = magnetometer[1];
            log_data.magnetometer_z = magnetometer[2];
        }

        if (gps->is_fixed() && gps->ready(millis())) {
            gps->location(&latitude, &longitude, &lat, &lon);
            log_data.latitude = latitude;
            log_data.longitude = longitude;
            realtime_data_packet->latitude = latitude;
            realtime_data_packet->longitude = longitude;
        }
    }

    /**
     * @brief Function that processes raw data to to useful data such as: 
     * altitude, apogee, vertical velocity, vertical acceleration, 
     * relative vertical acceleration and total linear acceleration.
     */
    void analyze()
    {
        if (bmp_available)
        {
            bool process_vertical_velocity = true;
            altitude = RocketIMUMath::altitude_from_pressure(this->pressure);
            altitude -= ground_altitude;    // Relative

            if (altitude == prev_altitude) {
                // Guard condition in case if sensor reading same
                process_vertical_velocity = false;
            } else {
                // Correct elapsed time measurement
                elapsed = (current_altitude_fixtime - previous_altitude_fixtime);
                previous_altitude_fixtime = current_altitude_fixtime;
            }

            if (apogee < altitude)
            {
                apogee = altitude;          // Relative
                apogee_time = current_altitude_fixtime;
            }
            
            if (process_vertical_velocity) {
                prev_vertical_velocity = vertical_velocity;
                vertical_velocity = RocketIMUMath::vertical_speed(this->altitude, this->prev_altitude, this->elapsed);
                prev_altitude = altitude;   // Relative
            }

            bmp_available = false;

            log_data.altitude = altitude;
            log_data.apogee = apogee;
            log_data.vertical_velocity = vertical_velocity;
            realtime_data_packet->altitude = altitude;
            realtime_data_packet->apogee = apogee;
            realtime_data_packet->vertical_velocity = vertical_velocity;
        }

        if (bno_available)
        {
            prev_vertical_acceleration = vertical_acceleration;
            vertical_acceleration = RocketIMUMath::vertical_acceleration(this->linear_acceleration, RocketIMUMath::transformation_matrix(this->quaternions));
            
            relative_vertical_acceleration = RocketIMUMath::relative_vertical_acceleration(this->relative_vertical_direction, this->linear_acceleration);
        
            prev_total_linear_acceleration = total_linear_acceleration;
            total_linear_acceleration = RocketIMUMath::total_linear_acceleration(this->linear_acceleration);

            bno_available = false;

            log_data.vertical_acceleration = vertical_acceleration;
            realtime_data_packet->vertical_acceleration = vertical_acceleration;
        }

        projected_altitude = RocketIMUMath::projected_altitude(altitude, vertical_velocity, vertical_acceleration);
        log_data.projected_altitude = projected_altitude;
        realtime_data_packet->projected_altitude = projected_altitude;
    }

    /**
     * @brief Starts (opens file) backup data log.
     * Sets data headers to first line.
     * Logs error if file can not be opened.
     * @param backup_file Backup file from logger
     * @param key Unique key to match flight log file
     * @return true 
     * @return false 
     */
    bool backup_start(File *backup_file, unsigned int key)
    {
        backup_data_file = *backup_file;
        if (backup_data_file)
        {
            backup_data_file.print("KEY\t");
            backup_data_file.println(key);
            backup_data_file.print("Iteration,");
            backup_data_file.print("Iteration_Time,");
            backup_data_file.print("BMP_Time,");
            backup_data_file.print("BNO_Time,");
            backup_data_file.print("Pressure,");
            backup_data_file.print("Temperature,");
            backup_data_file.print("Lin_Acc_x,");
            backup_data_file.print("Lin_Acc_y,");
            backup_data_file.print("Lin_Acc_z,");
            backup_data_file.print("Qutornions_w,");
            backup_data_file.print("Qutornions_x,");
            backup_data_file.print("Qutornions_y,");
            backup_data_file.print("Qutornions_z,");
            backup_data_file.print("Magnetometer_x,");
            backup_data_file.print("Magnetometer_y,");
            backup_data_file.print("Magnetometer_z,");
            backup_data_file.print("Gyroscope_x,");
            backup_data_file.print("Gyroscope_y,");
            backup_data_file.print("Gyroscope_z,");
            backup_data_file.print("Acceleration_x,");
            backup_data_file.print("Acceleration_y,");
            backup_data_file.print("Acceleration_z,");
            backup_data_file.print("Gravity_x,");
            backup_data_file.print("Gravity_y,");
            backup_data_file.print("Gravity_z,");
            backup_data_file.print("Altitude,");
            backup_data_file.print("Apogee,");
            backup_data_file.print("Vertical_acceleration,");
            backup_data_file.print("Vertical_velocity,");
            backup_data_file.print("Projected_Altitude,");
            backup_data_file.print("Latitude,");
            backup_data_file.println("Longitude,");
            backup_data_file.flush();
            return true;
        }
        else
        {
            #ifdef inotesting
            Serial.println("Error opening backup data file");
            #endif
        }
        return false;
    }

    /**
     * @brief Writes fresh data to backup data logging file in binary format.
     * WARNING! Currently without super-safe parsing.
     * 
     * @return true if could open file
     * @return false if could not open file
     */
    bool log_binary() {
        if (backup_data_file) {
            // Available needed to check if any data is fresh to avoid dublicates
            if (available) {
                char * binary_data = (char *) & log_data;
                for (int i = 0; i < log_data_size; i++) {
                    backup_data_file.print(binary_data[i]);
                }
                backup_data_file.flush();
                log_data.iterator++;
                available = false;
            }
            return true;
        }

        #ifdef inotesting
        Serial.println("Error opening backup data file");
        #endif
        return false;
    }

    /**
     * @brief Writes fresh data to backup data logging file.
     * 
     * @return true if could open file
     * @return false if could not open file
     */
    bool log()
    {
        if (backup_data_file)
        {
            // Available needed to check if any data is fresh to avoid dublicates
            if (available) {
                backup_data_file.print(log_data.iterator++);
                backup_data_file.print(',');
                backup_data_file.print(log_data.iteration_time);
                backup_data_file.print(',');
                backup_data_file.print(current_altitude_fixtime);
                backup_data_file.print(',');
                backup_data_file.print(bno_fixtime);
                backup_data_file.print(',');
                backup_data_file.print(pressure);
                backup_data_file.print(',');
                backup_data_file.print(temperature);
                backup_data_file.print(',');
                backup_data_file.print(linear_acceleration[0]);
                backup_data_file.print(',');
                backup_data_file.print(linear_acceleration[1]);
                backup_data_file.print(',');
                backup_data_file.print(linear_acceleration[2]);
                backup_data_file.print(',');
                backup_data_file.print(quaternions.w());
                backup_data_file.print(',');
                backup_data_file.print(quaternions.x());
                backup_data_file.print(',');
                backup_data_file.print(quaternions.y());
                backup_data_file.print(',');
                backup_data_file.print(quaternions.z());
                backup_data_file.print(',');
                backup_data_file.print(magnetometer[0]);
                backup_data_file.print(',');
                backup_data_file.print(magnetometer[1]);
                backup_data_file.print(',');
                backup_data_file.print(magnetometer[2]);
                backup_data_file.print(',');
                backup_data_file.print(gyroscope[0]);
                backup_data_file.print(',');
                backup_data_file.print(gyroscope[1]);
                backup_data_file.print(',');
                backup_data_file.print(gyroscope[2]);
                backup_data_file.print(',');
                backup_data_file.print(acceleration[0]);
                backup_data_file.print(',');
                backup_data_file.print(acceleration[1]);
                backup_data_file.print(',');
                backup_data_file.print(acceleration[2]);
                backup_data_file.print(',');
                backup_data_file.print(gravity[0]);
                backup_data_file.print(',');
                backup_data_file.print(gravity[1]);
                backup_data_file.print(',');
                backup_data_file.print(gravity[2]);
                backup_data_file.print(',');
                backup_data_file.print(altitude);
                backup_data_file.print(',');
                backup_data_file.print(apogee);
                backup_data_file.print(',');
                backup_data_file.print(vertical_acceleration);
                backup_data_file.print(',');
                backup_data_file.print(vertical_velocity);
                backup_data_file.print(',');
                backup_data_file.print(projected_altitude);
                backup_data_file.print(',');
                backup_data_file.print(latitude);
                backup_data_file.print(',');
                backup_data_file.print(longitude);
                backup_data_file.println(',');
                backup_data_file.flush();
                available = false;
            }
            return true;
        }
        else
        {
            #ifdef inotesting
            Serial.println("Error opening backup data file");
            #endif
        }
        return false;
    }

    /**
     * @brief Function that fixes rocket relative to BNO orientation
     * 
     * @return String representation of most closest relative sensor orientation
     */
    String calibrate()
    {
        /*
            Reset apogee settings in case if when sensor turned on and measured
            bad data.
        */
        prepare_for_apogee();

        double x = gravity[0];
        double y = gravity[1];
        double z = gravity[2];

        ground_altitude = altitude;
        String rvd = "Wrong starting position";

        if (x > 9.0) {
            relative_vertical_direction = 1;
            rvd = "X+";
        } 
        else if (x < -9.0) 
        {
            relative_vertical_direction = 2;
            rvd = "X-";
        }
        else if (y > 9.0)
        {
            relative_vertical_direction = 3;
            rvd = "Y+";
        }
        else if (y < -9.0)
        {
            relative_vertical_direction = 4;
            rvd = "Y-";
        }
        else if (z > 9.0)
        {
            relative_vertical_direction = 5;
            rvd = "Z+";
        }
        else if (z < -9.0)
        {
            relative_vertical_direction = 6;
            rvd = "Z-";
        }

        relative_vertical_direction = 0;
        return rvd;
    }
};
#endif