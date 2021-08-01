#ifndef BEAGLE_COMM_H
#define BEAGLE_COMM_H
#include <stdint.h>
#include <HardwareSerial.h>

// #define BEAGLE_COMM_DEBUG

/**
 * This structure contains flight statuses.
 */
typedef enum flight_status_t {
	WAIT,
	STANDBY,
	POWERED_ASCENT,
	UNPOWERED_ASCENT,
	DESCENT_TO_LAND,
	LANDED,
    TEST
} flight_status_t;

typedef enum arm_state_t {
	DISARMED,
	ARMED,
	RESTARTED
} arm_state_t;

typedef struct fallback_packet_t
{
  	int time;						///< Unique id or time for synchronizing transmition
	arm_state_t armed_state;			///< Use this channel to send ARMED or DISARMED command
	int start_pre_flight_checks;

	int use_external_flight_state;	///< Use data computed externally (1)
	flight_status_t flight_state;	///< Use this channel to send current flight status
	
	int use_external_flight_estimation;	///< Use data computed externally (1)
	double roll;					///< Rotation about x of the body 
	double pitch;					///< Rotation about y of the body 
	double yaw;						///< Rotation about z of the body 
	double proj_app;				///< estimated projected appogee the vehicle will reach given current flight condition
	double alt;						///< altitude estimate
	double alt_vel;					///< vertical velocity estimate 
	double alt_accel;				///< vertical accel estimate 
   
} fallback_packet_t;

typedef enum beagle_parsing_state_t {
	START_BYTE1,
	START_BYTE2,
	PACKET
} beagle_parsing_state_t;

typedef struct beagle_responce_packet_t {
	uint32_t time;
	flight_status_t flight_state;
} beagle_responce_packet_t;


class BeagleBoneCommunication {
	HardwareSerial *bserial;
	uint32_t baud_rate;
	fallback_packet_t packet;

	const int beagle_transmit_delay = 100;
	unsigned long beagle_serial_timer = 0;
	size_t packet_length = sizeof(fallback_packet_t);
	beagle_parsing_state_t parsing_state;
	char parsed_bytes[50];
	size_t parsed_bytes_counter;

	char start_byte1 = 0x81;
	char start_byte2 = 0xA1;

	beagle_responce_packet_t beagle_responce_packet;

	inline uint16_t fletcher16(char* buf, size_t buflen)
	{
		//Compute Fletcher16 CRC
		uint8_t chksm0 = 0, chksm1 = 0;
		for(unsigned i = 0; i < buflen; i++)
		{
			chksm0 += buf[i];
			chksm1 += chksm0;
		}
		
		return (uint16_t) ((chksm1 << 8) | chksm0);
	}

	void serial_byte_printer(HardwareSerial *bserial, char * arr, int len) {
		for(int i = 0; i < len; i++) {
			bserial->print(arr[i]);
		}
	}


	bool verify_checksum(char *data, size_t size) {
		char data_packet[sizeof(beagle_responce_packet_t)];
		memcpy(data_packet, data, sizeof(beagle_responce_packet_t));
		uint16_t checksum_counted = fletcher16(data_packet, sizeof(beagle_responce_packet_t));

		char chksm0 = data[size-2];
		char chksm1 = data[size-1];
		#ifdef BEAGLE_COMM_DEBUG
		Serial.print("Checksum res 0:");
		Serial.println(chksm0, HEX);
		Serial.print("Checksum res 1:");
		Serial.println(chksm1, HEX);
		#endif
		uint16_t checksum_recieved = (uint16_t) ((chksm1 << 8) | chksm0);

		#ifdef BEAGLE_COMM_DEBUG
		char cal1 = (char)checksum_counted;
		char cal2 = (char)(checksum_counted >> 8);
		Serial.print("Checksum calc 0:");
		Serial.println((char)cal1, HEX);
		Serial.print("Checksumr calc 1:");
		Serial.println((char)cal2, HEX);
		#endif

		if (checksum_counted == checksum_recieved) {
			return true;
		}
		
		return false;
	}

public:

	BeagleBoneCommunication(HardwareSerial *serial) {
		bserial = serial;
		packet.armed_state = RESTARTED;
		packet.use_external_flight_estimation = 0;	///< Use data computed externally (1)
		packet.flight_state = WAIT;	///< Use this channel to send current flight status
		parsing_state = START_BYTE1;
		beagle_responce_packet.flight_state = WAIT;
	}

	BeagleBoneCommunication(HardwareSerial *serial, uint32_t baud_rate) {
		bserial = serial;
		baud_rate = baud_rate;
		packet.armed_state = RESTARTED;
		packet.use_external_flight_estimation = 0;	///< Use data computed externally (1)
		packet.flight_state = WAIT;	///< Use this channel to send current flight status
		parsing_state = START_BYTE1;
		beagle_responce_packet.flight_state = WAIT;
	}

	int activate()
    {
        bserial->begin(1000000);
		delay(1000);
        if (!bserial)
        {
            #ifdef inotesting
            Serial.println("Beagle Device init error!");
            #endif
            return -1;
        }
        else {
            #ifdef inotesting
            Serial.println("Beagle Device is working!");
            #endif
        }
        return 0;
    }

	void run() {
		while(bserial->available()) {
			char symbol = bserial->read();

			#ifdef BEAGLE_COMM_DEBUG
			Serial.print("Beagle Symbol:");
			Serial.println(symbol, HEX);
			#endif

			if (parsing_state == START_BYTE1) {
				if (symbol == start_byte1) {
					#ifdef BEAGLE_COMM_DEBUG
					Serial.println("START_BYTE1");
					#endif
					parsing_state = START_BYTE2;
				}
			}

			else if (parsing_state == START_BYTE2) {
				if (symbol == start_byte2) {
					#ifdef BEAGLE_COMM_DEBUG
					Serial.println("START_BYTE2");
					#endif
					parsing_state = PACKET;
					parsed_bytes_counter = 0;
				}
			}

			else if (parsing_state == PACKET) {
				if (parsed_bytes_counter < sizeof(beagle_responce_packet_t) + 2) {
					parsed_bytes[parsed_bytes_counter++] = symbol;
					if (parsed_bytes_counter == sizeof(beagle_responce_packet_t) + 2) {
						#ifdef BEAGLE_COMM_DEBUG
						Serial.println("PACKET RECEIVED");
						Serial.print("flight_status_t size: ");
						Serial.println(sizeof(flight_status_t));
						Serial.print("beagle_responce_packet_t size: ");
						Serial.println(sizeof(beagle_responce_packet_t));
						Serial.print("Packet size without start bytes: ");
						Serial.println(sizeof(beagle_responce_packet_t) + 2);
						#endif
						if (verify_checksum(parsed_bytes, parsed_bytes_counter)) {
							memcpy(&beagle_responce_packet, parsed_bytes,
								sizeof(beagle_responce_packet_t));
							#ifdef BEAGLE_COMM_DEBUG
							Serial.println("CHECKSUM MATCH");
							Serial.print("Flight state: ");
							Serial.println(beagle_responce_packet.flight_state);
							#endif
						} else {
							#ifdef BEAGLE_COMM_DEBUG
							Serial.println("CHECKSUM NOT MATCH");
							#endif
						}
						#ifdef BEAGLE_COMM_DEBUG
						Serial.print("BBB time: ");
						Serial.println(beagle_responce_packet.time);
						#endif
						parsing_state = START_BYTE1;
					}
				}
			}
		}
	}

	fallback_packet_t * get_packet() {
		return &packet;
	}

	arm_state_t current_arm_state() {
		return packet.armed_state;
	}

	int current_external_flight_state() {
		return packet.use_external_flight_state;
	}

	int current_external_flight_estimation() {
		return packet.use_external_flight_estimation;
	}

	flight_status_t current_flight_state() {
		return packet.flight_state;
	}

	flight_status_t current_flight_state_received() {
		#ifdef BEAGLE_COMM_DEBUG
		Serial.print("Flight state current_flight_state_received: ");
		Serial.println(beagle_responce_packet.flight_state);
		#endif
		return beagle_responce_packet.flight_state;
	}

	arm_state_t change_arm_state() {
		if (packet.armed_state == ARMED) {
            packet.armed_state = DISARMED;
        } else {
            packet.armed_state = ARMED;
        }

		return packet.armed_state;
	}

	int change_external_flight_state() {
		if (packet.use_external_flight_state == 0) {
			packet.use_external_flight_state = 1;
		} else {
			packet.use_external_flight_state = 0;
		}

		return packet.use_external_flight_state;
	}

	int change_external_flight_estimation() {
		if (packet.use_external_flight_estimation == 0) {
			packet.use_external_flight_estimation = 1;
		} else {
			packet.use_external_flight_estimation = 0;
		}
		
		return packet.use_external_flight_estimation;
	}

	int set_start_pre_flight_checks() {
		if (packet.start_pre_flight_checks == 0) {
			packet.start_pre_flight_checks = 1;
		} else {
			packet.start_pre_flight_checks = 0;
		}
		
		return packet.start_pre_flight_checks;
	}

	bool ready(unsigned long current_time) {
		if (beagle_serial_timer + (1/beagle_transmit_delay) * 10e3 <= current_time) {
			beagle_serial_timer = current_time;
			return true;
		}

		return false;
	}

	void next_flight_state() {
		if (packet.flight_state == WAIT) {
			packet.flight_state = STANDBY;
		}
		else if (packet.flight_state == STANDBY) {
			packet.flight_state = POWERED_ASCENT;
		}
		else if (packet.flight_state == POWERED_ASCENT) {
			packet.flight_state = UNPOWERED_ASCENT;
		}
		else if (packet.flight_state == UNPOWERED_ASCENT) {
			packet.flight_state = DESCENT_TO_LAND;
		}
		else if (packet.flight_state == DESCENT_TO_LAND) {
			packet.flight_state = LANDED;
		}
	}

	void transmit(unsigned long current_time) {
		if (!(beagle_serial_timer + (1/beagle_transmit_delay) * 10e3 <= current_time)) {
			return;
		}

		beagle_serial_timer = current_time;
		char * data = (char *) &packet;
		uint16_t total_checksum = fletcher16(data, packet_length);
		char a = 0x81;
		char b = 0xA1;
		// char c = 0x25;
		// char d = 0x26;
		
		bserial->print(a);
		bserial->print(b);
		// bserial->print(id);
		serial_byte_printer(bserial, data, packet_length);
		bserial->print((char)total_checksum);
		bserial->print((char)(total_checksum >> 8));
		// bserial->print(c);
		// bserial->print(d);

		// bserial->flush();
	}
};

#endif
