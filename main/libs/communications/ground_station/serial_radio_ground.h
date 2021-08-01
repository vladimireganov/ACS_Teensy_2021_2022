#ifndef RADIO_SERIAL_H
#define RADIO_SERIAL_H
#include <HardwareSerial.h>
#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>
#include <string>
#include "radio_command.h"

#define RR_DEBUG

typedef enum parsing_state_t {
    START_BYTE,
    COMMAND,
    REQUEST_ID,
    DATA_SIZE,
    DATA_PARSE,
    END_BYTE
} parsing_state_t;

class Radio: public RadioCommand {
    private:
    HardwareSerial * radio_serial;
    uint32_t baud_rate;

    bool connected = false;
    bool active = true;

    unsigned long radio_time;
    const int RADIO_NORMAL_DELAY_MS = 100;
    
    char start_byte = '%';
    char end_byte = ';';

    parsing_state_t parsing_state;
    char parsed_command;
    std::string parsed_packet_data;
    int command_counter;
    int request_id_counter;
    int data_size_counter;

    size_t request_id;

    void byte_printer(char * arr, size_t size) {
        #ifdef RR_DEBUG
        Serial.println("byte printer start");
        for(uint8_t i = 0; i < size; i++) {
            Serial.println(arr[i], HEX);
        }
        Serial.println("byte printer end\n");
        #endif
        for(size_t i = 0; i < size; i++) {
            radio_serial->print(arr[i]);
        }
    }

    void byte_printer(const char * arr, size_t size) {
        for(size_t i = 0; i < size; i++) {
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
        request_id = 777;
    }

    Radio(HardwareSerial * serial, uint32_t baud_rate) : RadioCommand() {
        radio_serial = serial;
        baud_rate = baud_rate;
        request_id = 777;
    }

    Radio(HardwareSerial * serial, uint32_t baud_rate,
        char start_byte, char end_byte) : RadioCommand() {
        radio_serial = serial;
        this->start_byte = start_byte;
        this->end_byte = end_byte;
        request_id = 777;
    }

    int activate()
    {
        radio_serial->begin(57600);
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
            byte_printer((char *) &request_id, sizeof(request_id));
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
            while(radio_serial->available()) {
                char symbol = (char)radio_serial->read();
                // #ifdef RR_DEBUG
                // Serial.print("Symbol is: ");
                // Serial.println(symbol);
                // #endif
                if (parsing_state == START_BYTE) {
                    if (symbol == start_byte) {
                        parsed_packet_data = "";
                        request_id_counter = 0;
                        data_size_counter = 0;
                        parsing_state = COMMAND;
                        #ifdef RR_DEBUG
                        Serial.println("STARTBYTE");
                        #endif
                    } else {
                        #ifdef RR_DEBUG
                        Serial.println("Trash");
                        #endif
                    }
                }
                
                else if (parsing_state == COMMAND) {
                    parsed_packet_data += symbol;
                    #ifdef RR_DEBUG
                    Serial.println("COMMAND");
                    #endif
                    parsing_state = REQUEST_ID;
                }

                else if (parsing_state == REQUEST_ID) {
                    if (request_id_counter < 4) {
                        #ifdef RR_DEBUG
                        Serial.print("RI symbol: ");
                        Serial.println(symbol, HEX);
                        #endif
                        parsed_packet_data += symbol;
                        request_id_counter++;
                    }
                    else {
                        #ifdef RR_DEBUG
                        Serial.println("REQUEST ID");
                        #endif
                        parsing_state = END_BYTE;
                    }
                }

                else if (parsing_state == END_BYTE) {
                    if (symbol == end_byte) {
                        #ifdef RR_DEBUG
                        Serial.println("END BYTE");
                        #endif
                        parse_request(parsed_packet_data.c_str(), parsed_packet_data.size());
                        parsing_state = START_BYTE;
                    }
                }
            }
        }
    }

    void parse_request(const char * parsed_packet, size_t size) {
        if (size == 5) {
            char command = parsed_packet[0];
            memcpy(&request_id, parsed_packet + 1, 4);
            if (command_name_from_id.find(command) != command_name_from_id.end()) {
                command_state[command_name_from_id[command]] = true;
                #ifdef RR_DEBUG
                Serial.print("Request id: ");
                Serial.println(request_id);
                Serial.println("COMMAND FOUND\n\n");
                #endif
            } else {
                Serial.println("Command not found!\n\n");
            }
        } else {
            #ifdef RR_DEBUG
            Serial.print("Size does not match!");
            Serial.println(size);
            Serial.println("\n");
            #endif
        }
    }

    void responce(char command) {
        radio_serial->print(start_byte);
        radio_serial->print(command);
        byte_printer((char *) &request_id, sizeof(request_id));
        radio_serial->print(end_byte);
        radio_serial->flush();
    }


    void responce(char command, std::string data) {
        size_t data_size = data.size();
        

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
        byte_printer((char *) &request_id, sizeof(request_id));
        byte_printer((char *) &data_size, sizeof(data_size));      // Send 4 bytes of size
        if (data_size > 0) {
            byte_printer(data.c_str(), data.size());              // Send actual data
        }
        radio_serial->print(end_byte);
        radio_serial->flush();
    }

    void responce_data(char command, char * msg, int size) {
        request_id = 777;
        // char data_size_arr[4];
        // sprintf(data_size_arr, "%d", size);

        radio_serial->print(start_byte);
        radio_serial->print(command);
        byte_printer((char *) &request_id, sizeof(request_id)); // Send 4 bytes of size
        byte_printer((char *) &size, sizeof(size));             // Send actual data
        byte_printer(msg, size);
        radio_serial->print(end_byte);
        radio_serial->flush();
    }
};
#endif