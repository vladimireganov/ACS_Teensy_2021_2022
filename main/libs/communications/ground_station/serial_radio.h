#ifndef RADIO_SERIAL_H
#define RADIO_SERIAL_H
#include <HardwareSerial.h>
#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>
#include <string>
#include "radio_command.h"

class Radio: public RadioCommand{
    private:
    HardwareSerial * radio_serial;
    uint32_t baud_rate;

    bool connected = false;
    bool active = true;

    unsigned long radio_time;
    const int RADIO_NORMAL_DELAY_MS = 100;
    
    char start_byte = '%';
    char end_byte = ';';

    bool parsing = false;
    std::string parsed_packet;

    char request_device_id;

    void byte_printer(char * arr, size_t size) {
        for(uint8_t i = 0; i < size; i++) {
            radio_serial->print(arr[i]);
        }
    }

    void byte_printer(const char * arr, size_t size) {
        for(uint8_t i = 0; i < size; i++) {
            radio_serial->print(arr[i]);
        }
    }


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

    public:
    Radio(HardwareSerial * serial) : RadioCommand() {
        radio_serial = serial;
        baud_rate = 57600;
    }

    Radio(HardwareSerial * serial, uint32_t baud_rate) : RadioCommand() {
        radio_serial = serial;
        baud_rate = baud_rate;
    }

    Radio(HardwareSerial * serial, uint32_t baud_rate,
        char start_byte, char end_byte) : RadioCommand() {
        radio_serial = serial;
        this->start_byte = start_byte;
        this->end_byte = end_byte;
    }

    int activate()
    {
        radio_serial->begin(230400);
        delay(1000);
        if (!radio_serial)
        {
            #ifdef inotesting
            Serial.println("Radio init error!");
            #endif
            return -1;
        }
        else {
            connected = true;
            #ifdef inotesting
            Serial.println("Radio is working!");
            #endif
            radio_serial->print(start_byte);
            radio_serial->print(command_id_from_name["start"]);
            radio_serial->print(end_byte);
            radio_serial->flush();
        }
        return 0;
    }

    bool ready(double board_time) {
        if (radio_time < board_time) {
            radio_time = board_time + RADIO_NORMAL_DELAY_MS;
            return true;
        }
        return false;
    }

    bool is_working() {
        return connected;
    }

    bool enable() {
        radio_serial->clear();
        active = true;
        return active;
    }

    bool disable() {
        active = false;
        return active;
    }

    void run() {
        if (active) {
            for(int i = 0; i <= 50 && radio_serial->available(); i++) {
                char symbol = (char)radio_serial->read();
                if (symbol == start_byte) {
                    if (parsing) {
                        parsed_packet = "";
                    }
                    parsing = true;
                    continue;
                }
                
                else if(symbol == end_byte){
                    if(parsing) {
                        parse_request(parsed_packet);
                    }
                    parsing = false;
                    parsed_packet = "";
                    continue;
                }

                if (parsing) {
                    parsed_packet += symbol;
                }
            }
        }
    }

    void parse_request(std::string parsed_packet) {
        if (parsed_packet.size() == 1) {
            char command = parsed_packet[0];
            if (command_name_from_id.find(command) != command_name_from_id.end()) {
                command_state[command_name_from_id[command]] = true;
            } else {
                responce(0x44); // Not found
            }
        }
        else {
            responce(0x40); // Bad request
        }
    }

    void responce(char command) {
        radio_serial->print(start_byte);
        radio_serial->print(command);
        radio_serial->print(end_byte);
        radio_serial->flush();
    }

    void responce(char command, char * msg, size_t size) {
        radio_serial->print(start_byte);
        radio_serial->print(command);
        byte_printer(msg, size);
        radio_serial->print(end_byte);
        radio_serial->flush();
    }

    void responce(char command, std::string data) {
        #ifdef RR_DEBUG
        char data_size_arr[sizeof(size_t)];
        memcpy(data_size_arr, &data_size, sizeof(data_size));
        Serial.print("Data size: ");
        Serial.println(data_size);
        Serial.print("Size of data_size: ");
        Serial.println(sizeof(data_size));
        Serial.print("Datasize[0]: ");
        Serial.println(data_size_arr[0], HEX);
        Serial.print("Datasize[1]: ");
        Serial.println(data_size_arr[1], HEX);
        Serial.print("Datasize[2]: ");
        Serial.println(data_size_arr[2], HEX);
        Serial.print("Datasize[3]: ");
        Serial.println(data_size_arr[3], HEX);
        #endif
        radio_serial->print(start_byte);
        radio_serial->print(command);
        if (data.size() > 0) {
            byte_printer(data.c_str(), data.size());              // Send actual data
        }
        radio_serial->print(end_byte);
        radio_serial->flush();
    }
};
#endif