/**
 * @file flight_data_packet.h
 * @author Kazybek Mizam (kzm0099@auburn.edu)
 * @brief Contains data logging struct
 * @version 0.1
 * @date 2021-03-19
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef FLIGHT_DATA_PACKET
#define FLIGHT_DATA_PACKET
#include <stdint.h>

// Please look flight_states.h for more information
typedef struct teensy_data_t {
    // Data number
    uint32_t iterator;

    // Time
    uint32_t iteration_time;
    uint32_t bmp_fixtime;
    uint32_t bno_fixtime;
    
    // Raw BMP Data
    double pressure;            // Pressure read from BMP sensor
    double temperature;         // Tempurature read from BMP sensor

    // Raw BNO Data
    double linear_acceleration_x;
    double linear_acceleration_y;
    double linear_acceleration_z;
    double quaternions_w;
    double quaternions_i;
    double quaternions_j;
    double quaternions_k;
    double magnetometer_x;
    double magnetometer_y;
    double magnetometer_z;
    double gyroscope_x;
    double gyroscope_y;
    double gyroscope_z;
    double acceleration_x;
    double acceleration_y;
    double acceleration_z;
    double gravity_x;
    double gravity_y;
    double gravity_z;

    // Processed Data
    double altitude;
    double apogee;
    double vertical_acceleration;
    double vertical_velocity;
    double projected_altitude;

    float latitude;     // Latitude value from GPS
    float longitude;    // Longitude value from GPS
    
} teensy_data_t;

#endif