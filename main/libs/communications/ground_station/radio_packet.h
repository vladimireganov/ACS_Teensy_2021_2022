#ifndef RADIO_PACKET
#define RADIO_PACKET
#include "../beaglebone/beagle_serial.h"
typedef struct bno_calib_packet_t
{
	uint8_t system;
	uint8_t gyro;
	uint8_t accel;
    uint8_t mag;
   
} bno_calib_packet_t;

typedef struct sensor_connection_packet_t
{
	bool bno;
	bool bmp;
	bool sd;
	bool gps;
} sensor_connection_packet_t;

typedef struct realtime_data_packet_t
{
  	uint32_t time;						///< Unique id or time for synchronizing transmition
    arm_state_t armed_state;			///< Use this channel to send ARMED or DISARMED command
	uint8_t use_external_state_estimation;	///< Use data computed externally (1)
	flight_status_t flight_state;	///< Use this channel to send current flight status
	uint8_t use_external_estimation;	///< Use data computed externally (1)
	double altitude;
    double apogee;
    double projected_altitude;
	double vertical_acceleration;
	double vertical_velocity;
	float latitude;
	float longitude;
} realtime_data_packet_t;

bno_calib_packet_t bno_calib_packet;
sensor_connection_packet_t sensor_connection_packet;
realtime_data_packet_t realtime_data_packet;

size_t b_data = sizeof(bno_calib_packet_t);
size_t s_data = sizeof(sensor_connection_packet_t);
size_t rt_data = sizeof(realtime_data_packet_t);
#endif