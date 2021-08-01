/**
 * flight_logger.h
 * 
 * Developed for AURA Embedded Systems 2021
 * 
 * The is wrapper for Adafruit SD Breakout Board that also works uses Teensy SD.h
 * library with pre-defined logging methods for fullscale 2021 flights.
 * This library mainly for flight logging such as statuses flight states and errors.
 * 
 * Data logging is still in flight_data.h however it uses this library to get file
 * reference.
 * 
 * Uses Teensy SD Library, Adafruit SD library
 * 
 * TODO: I am not exactly confident on what is better and easier, therefore there might be
 * some major changes after I get some knowlendge
 * 
 * This library may look confusing because it both setup sensor and also has pre-defined
 * flight logging methods, but data logging is in other library.
 * 
 * 
 * @author Kazybek Mizam
 * @github @1kzpro
 * @version 09/10/2020
 * 
 * @copyright Copyright (c) AURA Embedded Systems 2021
*/

#ifndef LOGGER_H
#define LOGGER_H
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include <string>

#include <SD.h>

class Logger {
    private:
    int pin = 10;
    int iterator = 0;
    File flight_log_file;

    public:
    unsigned int key;
    bool connected = false;
    
    Logger() {}
    
    Logger(int pin) {
        this->pin = pin;
    }
    void activate()
    {
        /*
            SD begin has 2 methods:
            bool begin(uint8_t csPin = SD_CHIP_SELECT_PIN);
            bool begin(uint32_t clock, uint8_t csPin);

            Current is SD_CHIP_SELECT_PIN
            10 is pin number that sd on
        */
        if (!SD.begin(pin))
        {
            #ifdef inotesting
            Serial.println("SD Card init error!");
            #endif
            connected = false;
        }
        else {
            #ifdef inotesting
            Serial.println("SD is OK");
            #endif
            connected = true;

            srand(millis());
            key = rand() % (int)10E9 + 1;

            flight_log_file = SD.open(create_file_name("LOG"), FILE_WRITE);

            flight_log_file.print("KEY\t");
            flight_log_file.println(key);

            // Serial.print("flight_log_file_name: ");
            // Serial.println(flight_log_file_name);
        }
    }

    bool is_working() {
        return connected;
    }

    const char * create_file_name(const char *prefix)
    {
        int number_of_files = numberOfFiles(prefix);
        
        String nf = prefix;
        nf += String(number_of_files);
        nf += ".txt";

        while(SD.exists(nf.c_str())) {
            #ifdef inotesting
            Serial.print("File Exists: ");
            #endif
            Serial.println(nf);
            number_of_files++;
            nf = String(prefix) + String(number_of_files) + ".txt";
        }

        #ifdef inotesting
        Serial.println("File is only one");
        Serial.print("Filename: ");
        Serial.println(nf);
        #endif

        return nf.c_str();
    }

    int numberOfFiles(const char *prefix) {
        int num_files = 0;
        File root = SD.open("/"); // start at root and walk all files

        while (true)
        {
            File entry = root.openNextFile();
            if (!entry) // no more files
                break;
            else if (entry.isDirectory()) // skip over directories
                continue;
            else
            {
                String current_file = entry.name();
                // Serial.print("Current file: ");
                // Serial.print(current_file);
                if (current_file.startsWith(prefix)) {
                    // Serial.println("; This file starts with YES");
                    num_files++;
                } else {
                    // Serial.println("; This file starts with NO");
                }
            }
        }
        return num_files;
    }

    void logInfo(String info)
    {
        if (flight_log_file)
        {
            flight_log_file.print(millis());
            flight_log_file.print("\tINFO\t");
            flight_log_file.println(info);
            flight_log_file.flush();
        } else {
            #ifdef inotesting
            Serial.println("Error opening log status file");
            #endif
        }
    }

    void logSensors(bool bmp, bool bno, bool sd)
    {
        if (flight_log_file)
        {
            flight_log_file.print(millis());
            flight_log_file.print("\tSENSORS\t");
            flight_log_file.print(bmp);
            flight_log_file.print('\t');
            flight_log_file.print(bno);
            flight_log_file.print('\t');
            flight_log_file.println(sd);
            flight_log_file.flush();
        } else {
            #ifdef inotesting
            Serial.println("Error opening log status file");
            #endif
        }
    }

    void logEvent(String event)
    {
        if (flight_log_file)
        {
            flight_log_file.print(millis());
            flight_log_file.print("\tEVENT\t");
            flight_log_file.println(event);
            flight_log_file.flush();
        } else {
            #ifdef inotesting
            Serial.println("Error opening log status file");
            #endif
        }
    }

    void logCalibrate(double ground_altitude, String direction)
    {
        if (flight_log_file)
        {
            flight_log_file.print(millis());
            flight_log_file.print("\tCALIBRATE\t");
            flight_log_file.print(ground_altitude);
            flight_log_file.print('\t');
            flight_log_file.println(direction);
            flight_log_file.flush();
        } else {
            #ifdef inotesting
            Serial.println("Error opening log status file");
            #endif
        }
    }

    void logError(String error){
        

        if (flight_log_file)
        {
            flight_log_file.print(millis());
            flight_log_file.print("\tERROR\t");
            flight_log_file.println(error);
            flight_log_file.flush();
        } else {
            #ifdef inotesting
            Serial.println("Error opening log status file");
            #endif
        }
    }

    void logTrigger(String info){
        if (flight_log_file)
        {
            flight_log_file.print(millis());
            flight_log_file.print("\tTRIGGER\t");
            flight_log_file.println(info);
            flight_log_file.flush();
        } else {
            #ifdef inotesting
            Serial.println("Error opening log status file");
            #endif
        }
    }

    template <typename T>
    void logTrigger(String info, T msg){
        if (flight_log_file)
        {
            flight_log_file.print(millis());
            flight_log_file.print("\tTRIGGER\t");
            flight_log_file.print(info);
            flight_log_file.print("\t");
            flight_log_file.println(msg);
            flight_log_file.flush();
        } else {
            #ifdef inotesting
            Serial.println("Error opening log status file");
            #endif
        }
    }
};
#endif
