/**
 * @file flight_states.h
 * @author Kazybek Mizam (kzm0099@auburn.edu)
 * @brief Controller library of rocket flight states
 * used for transitioning from one flight state to another that is
 * needed to make some procedures in exact states
 * 
 * There are 4 main states
 * 1. Lauched - motor ignited rocket launched
 * 2. Burnt out - motor burnt out, free fall with initial velocity
 * 3. Apogee - reached highest altitude, fixed after some dt
 * 4. Landed - no altitude change and movement of rocket
 * 
 * @version 0.2
 * @date 2021-03-12
 * 
 * @copyright Copyright (c) AURA Embedded Systems 2021
 * 
 */

#ifndef ROCKET_FLIGHT_STATES_H
#define ROCKET_FLIGHT_STATES_H
#include <math.h>
#include <stdlib.h>

#define LAUNCH_GUARD_TIMER 200                          // in millis
#define LAUNCH_VERTICAL_ACCELERATION_THRESHOLD 20.0     // m/s^2
#define LAUNCH_VERTICAL_VELOCITY_THRESHOLD 5.0          // m/s
#define LAUNCH_ALTITUDE_THRESHOLD 10.0              // meter

#define BURNT_OUT_GUARD_TIMER 200                       // in millis
#define BURNT_OUT_ALTITUDE_THRESHOLD 50.0               // meter
#define BURNT_OUT_VERTICAL_ACCELERATION_THRESHOLD 0.0   // m/s^2

#define APOGEE_GUARD_TIMER 5000                         // in millis
#define APOGEE_ALTITUDE_DIFFERENCE_MINIMUM 3.0          // meter

#define LAND_GUARD_TIMER 5000                           // in millis
#define LAND_MAX_ALTITUDE_DIFFERENCE 1.0                // meter
#define LAND_MAX_ACCELERATION_VIBRATION 0.8             // m/s^2

class RocketFlightStates {
    unsigned long timer;

    public:
    void setup_timer(unsigned long current_time) {
        timer = current_time;
    }

    /**
     * @brief Function that checks wether rocket has launched, 
     * uses either altitude change or vertical motion detection
     * 
     * @param vertical_velocity current vertical velocity
     * @param vertical_acceleration current vertical acceleration
     * @param current_time current board time or any elapsing time unit
     * @return true if laucnhed
     * @return false if not launched
     */
    bool if_launched(double vertical_velocity, double vertical_acceleration, unsigned long current_time) {
        // (vertical_acceleration >= 20) && (dh >= 50) // for testing
        if (vertical_acceleration >= LAUNCH_VERTICAL_ACCELERATION_THRESHOLD 
            && (fabs(vertical_velocity) >= LAUNCH_VERTICAL_VELOCITY_THRESHOLD)) {
            if (timer + LAUNCH_GUARD_TIMER < current_time) {
                return true;
            }
            return false;
        }
        timer = current_time;
        return false;
    }

    /**
     * @brief Function that checks wether rocket has launched, 
     * uses either altitude change or vertical motion detection
     * 
     * @param current_altitude current altitude
     * @param vertical_acceleration current vertical acceleration
     * @param current_time current board time or any elapsing time unit
     * @return true if laucnhed
     * @return false if not launched
     */
    bool if_launched_by_altitude(double current_altitude, double vertical_acceleration, unsigned long current_time) {
        // (vertical_acceleration >= 20) && (dh >= 50) // for testing
        if (vertical_acceleration >= LAUNCH_VERTICAL_ACCELERATION_THRESHOLD 
            && (current_altitude >= LAUNCH_ALTITUDE_THRESHOLD)) {
            if (timer + LAUNCH_GUARD_TIMER < current_time) {
                return true;
            }
            return false;
        }
        timer = current_time;
        return false;
    }

    /**
     * @brief Function that checks wether rocket motor burnt out, 
     * uses either altitude change or negative acceleration
     * 
     * @param current_altitude current altitude
     * @param vertical_velocity current vertical velocity
     * @param vertical_acceleration current vertical acceleration
     * @param current_time current board time or any elapsing time unit
     * @return true if motor burnt out
     * @return false if motor not burnt out
     */
    bool if_burnt_out(double current_altitude, double vertical_velocity, double vertical_acceleration,
        unsigned long current_time) {
        // return false if vertical acceleration is less than 0
        // Velocity unit is m/s and accel m/s^2
        // (vertical_velocity <= 100) && (vertical_acceleration <= 0.0)
        if ((vertical_acceleration <= BURNT_OUT_VERTICAL_ACCELERATION_THRESHOLD)
            && (current_altitude >= BURNT_OUT_ALTITUDE_THRESHOLD)){
            if (timer + BURNT_OUT_GUARD_TIMER < current_time) {
                return true;
            }
            return false;
        }
        timer = current_time;
        return false;
    }

    /**
     * @brief Function that checks wether rocket has reached its apogee, 
     * uses altitude and time of max altitude record time, to check how
     * long it was fixed.
     * 
     * @param max_altitude max registered altitude value
     * @param current_altitude current altitude value
     * @param current_time current board time or any elapsing time unit
     * @param previous_time max altitude fixed board time or any elapsing time unit
     * @return true if max altitude is verified for time period
     * @return false if not apogee
     */
    bool if_reached_apogee(double max_altitude, double current_altitude, unsigned int current_time,
        double previous_time) {
        double dh = max_altitude - current_altitude;
        double dt = current_time - previous_time;
        // Apogee is same as max altitude
        if (dh >= APOGEE_ALTITUDE_DIFFERENCE_MINIMUM && dt >= APOGEE_GUARD_TIMER) {
            return true;
        }
        return false;
    }

    /**
     * @brief Function that checks wether rocket has landed.
     * It changes land time and altitude if rocket still flying to current data
     * 
     * @param current_altitude current altitude value
     * @param land_altitude land recorded altitude value
     * @param net_linear_acceleration (x^2+y^2+z^2)^(1/2) of linear acceleration value
     * @param current_time current board time or any elapsing time unit
     * @param land_time land recorded time
     * @return true if land detected
     * @return false if land not detected
     */
    bool if_landed(double current_altitude, double *land_altitude, double net_linear_acceleration,
        unsigned int current_time, unsigned long *land_time)  {
        double dh = current_altitude - *land_altitude;
        double dt = current_time - *land_time;
        if (fabs(dh) <= LAND_MAX_ALTITUDE_DIFFERENCE
            && net_linear_acceleration <= LAND_MAX_ACCELERATION_VIBRATION) {
            if(dt >= LAND_GUARD_TIMER) {
                return true;
            }
            return false;
        } else {
            *land_altitude = current_altitude;
            *land_time = current_time;
        }
        return false;
    }
};

RocketFlightStates rocket_flight_state = RocketFlightStates();
#endif