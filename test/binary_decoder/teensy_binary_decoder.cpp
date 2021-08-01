#include <fstream>
#include <iostream>
#include <string>
#include <map>  //Same as dict in python
#include "../../main/libs/data/flight_data_packet.h"
using namespace std;

void setup_teensy_parse(ifstream * data_file, ofstream * decoded_file) {
    string line;
    if (data_file->is_open())
    {
        if (getline(*data_file, line))   // Start data parsing
        {
            *decoded_file << line << endl;
            getline(*data_file, line); // Get data names
            *decoded_file << line << endl;
        } else {
            cout << "Unable to fetch initial datas" << endl;
        }
    }
}

void parse(ofstream * decoded_file, teensy_data_t * teensy_decoded_data) {
    *decoded_file << teensy_decoded_data->iterator << ',';

    *decoded_file << teensy_decoded_data->iteration_time << ',';
    *decoded_file << teensy_decoded_data->bmp_fixtime << ',';
    *decoded_file << teensy_decoded_data->bno_fixtime << ',';

    *decoded_file << teensy_decoded_data->pressure << ',';
    *decoded_file << teensy_decoded_data->temperature << ',';

    *decoded_file << teensy_decoded_data->linear_acceleration_x << ',';
    *decoded_file << teensy_decoded_data->linear_acceleration_y << ',';
    *decoded_file << teensy_decoded_data->linear_acceleration_z << ',';

    *decoded_file << teensy_decoded_data->quaternions_w << ',';
    *decoded_file << teensy_decoded_data->quaternions_i << ',';
    *decoded_file << teensy_decoded_data->quaternions_j << ',';
    *decoded_file << teensy_decoded_data->quaternions_k << ',';

    *decoded_file << teensy_decoded_data->magnetometer_x << ',';
    *decoded_file << teensy_decoded_data->magnetometer_y << ',';
    *decoded_file << teensy_decoded_data->magnetometer_z << ',';

    *decoded_file << teensy_decoded_data->gyroscope_x << ',';
    *decoded_file << teensy_decoded_data->gyroscope_y << ',';
    *decoded_file << teensy_decoded_data->gyroscope_z << ',';

    *decoded_file << teensy_decoded_data->acceleration_x << ',';
    *decoded_file << teensy_decoded_data->acceleration_y << ',';
    *decoded_file << teensy_decoded_data->acceleration_z << ',';

    *decoded_file << teensy_decoded_data->gravity_x << ',';
    *decoded_file << teensy_decoded_data->gravity_y << ',';
    *decoded_file << teensy_decoded_data->gravity_z << ',';

    *decoded_file << teensy_decoded_data->altitude << ',';
    *decoded_file << teensy_decoded_data->apogee << ',';
    *decoded_file << teensy_decoded_data->vertical_acceleration << ',';
    *decoded_file << teensy_decoded_data->vertical_velocity << ',';
    *decoded_file << teensy_decoded_data->projected_altitude << ',';
    *decoded_file << teensy_decoded_data->latitude << ',';
    *decoded_file << teensy_decoded_data->longitude << ',';
    *decoded_file << endl;
}

void run(ifstream * data_file, ofstream * decoded_file) {
    setup_teensy_parse(data_file, decoded_file);

    char * memblock;
    char confirm_char1;
    char confirm_char2;
    teensy_data_t teensy_decoded_data;
    int tds = sizeof(teensy_data_t);
    unsigned long position = data_file->tellg();

    // Debug
    // cout << tds << endl;
    // cout << data_file->tellg() << endl;
    // data_file->seekg(0, ios::beg);

    // for (int i = 0; i < position; i++) {
    //     data_file->read(&confirm_char1, 1);
    //     printf("Char is: %X\n", confirm_char1);
    // }

    data_file->seekg(position, ios::beg); // Put position to beginnning of binary data
    cout << "Position is: " << data_file->tellg() << endl;
    while(!data_file->eof()) {
        memblock = new char [tds];

        data_file->read(memblock, tds);

        memcpy(&teensy_decoded_data, memblock, tds);
        parse(decoded_file, &teensy_decoded_data);
        // data_file->read(&confirm_char1, 1);
        // data_file->read(&confirm_char2, 1);

        // while(confirm_char1 != 0xD) {
        //     data_file->read(&confirm_char1, 1);
        // }
        // data_file->read(&confirm_char2, 1);
        // cout << "Position is: " << data_file->tellg() << endl;

        // if (confirm_char1 == 0xD && confirm_char2 == 0xA) {
        //     memcpy(&teensy_decoded_data, memblock, tds);
        //     parse(decoded_file, &teensy_decoded_data);
        // } else {
        //     printf("Char is: %X\n", confirm_char1);
        //     printf("Char is: %X\n", confirm_char2);
        //     cout << "Error size does not match!" << endl;

        //     // for (int i = 0; i < tds; i++) {
        //     //     printf("MemChar is: %X\n", memblock[i]);
        //     // }
        //     break;
        // }

        delete[] memblock;
    }

    data_file->close();
    decoded_file->close();
}

int main() {
    string binary_file;

    ifstream teensy_file;
    ofstream data_file;

    teensy_file.open("BACKUP4.TXT");
    data_file.open("parsed_data_4.csv");

    if (!teensy_file.is_open()) {
        cout << "Error opening teensy file" << endl;
        return -1;
    }
    if (!data_file.is_open()) {
        return -1;
    }

    run(&teensy_file, &data_file);
}