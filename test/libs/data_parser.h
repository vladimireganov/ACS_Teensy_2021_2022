/**
 * @file data_parser.h
 * @author Kazybek Mizam (kzm0099@auburn.edu)
 * @brief This code test emulatates teensy data from logged data
 * @version 0.1
 * @date 2021-03-12
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef TEENSY_DATA_PARSER
#define TEENSY_DATA_PARSER

#include <fstream>
#include <iostream>
#include <string>
#include <map>  //Same as dict in python
using namespace std;

// #define output_data_name

class FullscaleDataParser {
    private:

    map<int, string> data_name_map;
    map<int, string> data_value_map;

    string line;

    void data_parser(string line, map<int, string> *data_map) {
        string data_val = "";
        int index = 0;
        for (int i = 0; i < line.length(); i++) {
            if (line[i] == ',') {
                #ifdef output_data_name
                cout << data_number << ":" << data_name << endl;
                #endif
                data_map->insert(pair<int, string>(index++, data_val));
                data_val = "";
                continue;
            }
            data_val += line[i];
        }
    }

    void data_combiner(map<string, string> &data_combined_map, map<int,
        string> *data_name_map, map<int, string> *data_value_map) {
        map<int, string>::iterator it_val=data_value_map->begin();

        for (map<int, string>::iterator it=data_name_map->begin(); it!=data_name_map->end(); ++it) {
            data_combined_map.insert(pair<string, string>(it->second, it_val->second));
            ++it_val;
        }
        
    }

    template<typename Map>
    void PrintMap(Map& m)
    {
        cout << "[ ";
        for (auto &item : m) {
            cout << item.first << ":" << item.second << " ";
        }
        cout << "]\n";
    }

    public:
    FullscaleDataParser() {
    }

    void print_data_keys(){
        PrintMap(data_name_map);
    }

    void print_data_values(){
        PrintMap(data_value_map);
    }

    bool start(ifstream *myfile, int skips = 0) {
        if (myfile->is_open())
        {
            for (int i = 0; i < skips; i++) {
                getline(*myfile, line); // Skip before data start
            }
            getline(*myfile, line); // Get data keys name
            data_parser(line, &data_name_map); // Parse data keys to correspoding index
            return true;
        }

        else cout << "Unable to open file";
        return false;
    }

    bool get(ifstream *myfile, map<string, string> &data_map) {
        data_map.clear();
        data_value_map.clear();
        if (myfile->is_open()) {
            if (getline(*myfile, line))   // Start data parsing
            {
                // cout << line << endl; // debug
                data_parser(line, &data_value_map);
                data_combiner(data_map, &data_name_map, &data_value_map);
                return true;
            }

            myfile->close();
        } else {
            cout << "Can not open file!" << endl;
        }
        return false;
    }

};
#endif