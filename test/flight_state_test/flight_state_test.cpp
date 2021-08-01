/*
 * To complie this code use for mac:
 * g++ -std=c++11 flight_state_test.cpp -o flight_state_test.o
 * To complie this code use for windows:
 * Use g++ flight_state_test.cpp -o flight_state_test.exe
*/

// Please make sure pathes are correct
#include "../../main/libs/flight_states.h"
#include "../../main/libs/flight_math.h"
#include "../libs/data_parser.h"

#define use_teensy_data // else uses bno data
#define launch_by_altitude

enum FlightStates {
    STANDBY,
	POWERED_ASCENT,
	UNPOWERED_ASCENT,
	DESCENT_TO_LAND,
	LANDED,
};

void test_flight_state(map<string, string> &data) {
    static FlightStates mystate = STANDBY;

    #ifdef use_teensy_data
    int data_number = stoi(data["Iteration"]);
    unsigned long current_time = stoul(data["Time"]);
    double vertical_velocity = stod(data["Vertical_velocity"]);
    double vertical_acceleration = stod(data["Vertical_acceleration"]);
    double current_altitude = stod(data["Altitude"]);
    static double max_altitude; // = stod(data["Apogee"]);

    static unsigned long apogee_fixed_time = 0;
    if(max_altitude < current_altitude) {
        max_altitude = current_altitude;
        apogee_fixed_time = current_time;
    }

    imu::Vector<3> linear_acceleration;
    linear_acceleration[0] = stod(data["Lin_Acc_x"]);
    linear_acceleration[1] = stod(data["Lin_Acc_y"]);
    linear_acceleration[2] = stod(data["Lin_Acc_z"]);
    #else
    int data_number = stoi(data["loop_index"]);
    unsigned long current_time = stoull(data["last_step_ns"]) / 10e6;
    double vertical_velocity = stod(data["alt_bmp_vel"]);
    double vertical_acceleration = stod(data["alt_bmp_accel"]);
    double current_altitude = stod(data["alt_bmp"]);
    static double max_altitude;

    static unsigned long apogee_fixed_time = 0;
    if(max_altitude < current_altitude) {
        max_altitude = current_altitude;
        apogee_fixed_time = stoull(data["last_step_ns"]) / 10e6;
        // cout << max_altitude << endl;
    }

    imu::Vector<3> linear_acceleration;
    linear_acceleration[0] = stod(data["accel_X"]);
    linear_acceleration[1] = stod(data["accel_Y"]);
    linear_acceleration[2] = stod(data["accel_Z"]);
    #endif
    double net_linear_acceleration = RocketIMUMath::total_linear_acceleration(linear_acceleration);

    // if (vertical_acceleration >= 20)  {
    //     cout << "Vertical acceleration: " << vertical_acceleration << endl;
    // }

    static double land_altitude;
    static unsigned long land_time;
    static bool run_once = true;

    #ifdef launch_by_altitude
    if(mystate == STANDBY) {
        if(rocket_flight_state.if_launched_by_altitude(current_altitude, vertical_acceleration, current_time)) {
            mystate = POWERED_ASCENT;

            cout << "Time: " << current_time << endl;
            cout << "Launched at data number: " << data_number << endl;
            cout << "Current altitude: " << current_altitude << endl;
            cout << "Vertical acceleration: " << vertical_acceleration << endl;
            cout << "Vertical velocity: " << vertical_velocity << endl;
            cout << endl;
        }
    }
    #else
    if(mystate == STANDBY) {
        if(rocket_flight_state.if_launched(vertical_velocity, vertical_acceleration, current_time)) {
            mystate = POWERED_ASCENT;

            cout << "Time: " << current_time << endl;
            cout << "Launched at data number: " << data_number << endl;
            cout << "Current altitude: " << current_altitude << endl;
            cout << "Vertical acceleration: " << vertical_acceleration << endl;
            cout << "Vertical velocity: " << vertical_velocity << endl;
            cout << endl;
        }
    }
    #endif
    else if(mystate == POWERED_ASCENT) {
        if(rocket_flight_state.if_burnt_out(current_altitude, vertical_velocity,
            vertical_acceleration, current_time)) {
            mystate = UNPOWERED_ASCENT;
            apogee_fixed_time = current_time;
            max_altitude = current_altitude;

            cout << "Time: " << current_time << endl;
            cout << "Burnt out at data number: " << data_number << endl;
            cout << "Current altitude: " << current_altitude << endl;
            cout << "Vertical acceleration: " << vertical_acceleration << endl;
            cout << "Vertical velocity: " << vertical_velocity << endl;
            cout << endl;
        }
    }
    else if(mystate == UNPOWERED_ASCENT) {
        if(rocket_flight_state.if_reached_apogee(max_altitude, current_altitude,
            current_time, apogee_fixed_time)) {
            mystate = DESCENT_TO_LAND;
            land_altitude = current_altitude;
            land_time = current_time;

            cout << "Time: " << current_time << endl;
            cout << "Reached apogee at data number: " << data_number << endl;
            cout << "Max altitude: " << max_altitude << endl;
            cout << "Current altitude: " << current_altitude << endl;
            cout << endl;
        }
    }
    else if(mystate == DESCENT_TO_LAND) {
        if(rocket_flight_state.if_landed(current_altitude, &land_altitude,
            net_linear_acceleration, current_time, &land_time)) {
            mystate = LANDED;

            cout << "Time: " << current_time << endl;
            cout << "Landed at data number: " << data_number << endl;
            cout << "Current altitude: " << current_altitude << endl;
            cout << "Land altitude: " << land_altitude << endl;
            cout << endl;
        }
    }
    else if(mystate == LANDED) {
        if (run_once) {
            printf("Done!\n");
            run_once = false;
        }
    }
}

int main(int argc, char *argv[]) {

    FullscaleDataParser FSDP = FullscaleDataParser(); // Data parsing and processing class
    map<string, string> data; // data instance stored here

    /**
     * @brief Open desired data file
     * fullscale_v1_data.txt // Teensy data for Fullscale 2021 V1 Launch
     * fullscale_v2_data.txt // Teensy data for Fullscale 2021 V2 Launch
     * fv2_teensy_vel_filtered_full.csv // Teensy data for Fullscale 2021 V2 Launch full and same altitude filtered
     * fv2_teensy_vel_filtered.csv // Teensy data for Fullscale 2021 V2 Launch full and same altitude range filtered
     */
    ifstream myfile("fullscale_v2_data.txt");

    if (FSDP.start(&myfile)) {
        while (FSDP.get(&myfile, data))
        {
            test_flight_state(data);
        }
    }
    return 0;
}