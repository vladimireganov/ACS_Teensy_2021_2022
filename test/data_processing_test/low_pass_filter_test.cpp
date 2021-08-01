/*
 * To complie this code use for mac:
 * g++ -std=c++11 flight_state_simulation.cpp -o flight_state_simulation.o
 * To complie this code use for windows:
 * Use g++ flight_state_simulation.cpp -o flight_state_simulation.exe
*/

// Please make sure pathes are correct
#include "../../main/libs/flight_math.h"
#include "../libs/data_parser.h"

#define use_teensy_data

double test_velocity(map<string, string> &data) {
    #ifdef use_teensy_data
    int data_number = stoi(data["Iteration"]);
    unsigned long current_time = stoul(data["Time"]);
    static double vertical_velocity = stod(data["Vertical_velocity"]);
    #else
    #endif

    vertical_velocity = RocketIMUMath::low_pass_filter(stod(data["Vertical_velocity"]), vertical_velocity, 50*10e-3);
    // cout << "Smoothed vertical velocity is: " << vertical_velocity << endl;
    return vertical_velocity;
}

int main(int argc, char *argv[]) {

    FullscaleDataParser FSDP = FullscaleDataParser();
    map<string, string> data;
    
    ifstream myfile("../data/fullscale_v1_data.txt");
    ofstream outfile("smoothed_velocity.csv");
    
    // FSDP.start(&myfile);
    outfile.write("Data number,Vertical_velocity,\n", 31);

    if (FSDP.start(&myfile)) {
        while (FSDP.get(&myfile, data))
        {
            outfile << data["Iteration"] << "," << to_string(test_velocity(data)) << ",\n";
        }
    }
    outfile.close();
    return 0;
}