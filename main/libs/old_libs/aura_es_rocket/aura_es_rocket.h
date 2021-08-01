#ifndef AURA_ES_Rocket
#define AURA_ES_Rocket
#include <math.h>
#include "./utility/imumaths.h"

#ifndef GRAV
#define GRAV 9.80665 // Gravity in SI Unit
#endif

/**
 * Class Rocket
 * 
 * This class contains all imu calculations of rocket
 * Events logic
 * Data logging
 * Data analyzing
*/
class Rocket
{
    public:
    // Constructor
    Rocket();

    /**
     * Matrix to transform from rockets frame towards ground frame. 
     * Useful for finding vertical acceleration
     * @param quaternions Quaternions of imu::Quaternion type
     * @return transformation matrix
     * @see RocketMath::vertical_acceleration(imu::Vector<3> linear_acceleration);
    */
    static imu::Matrix<3> transformation_matrix(imu::Quaternion quaternions);

    /**
     * Function that returns vertical acceleration measured with ground frame. 
     * Useful to check if rocket engine burnt out
     * @param linear_acceleration Linear Acceleration from accelerometer
     * @return vertical acceleration
    */
    static double vertical_acceleration(imu::Vector<3> linear_acceleration, imu::Matrix<3> transformation_matrix);

    /**
     * Function that returns total linear (without gravity) acceleration from linear acceleration. 
     * Useful to detect any motion
     * @param linear_acceleration Linear Acceleration from accelerometer
     * @return total acceleration
    */
    static double total_linear_acceleration(imu::Vector<3> linear_acceleration);

    /**
     * Function that calculates elapsed time between data read time. 
     * Useful to find vertical velocity
     * @param data_time Current elapsed time of data read
     * @param prev_data_time Previous elapsed time of data read
     * @return elapsed time 
    */
    static double elapsed_time(unsigned int data_time, unsigned int prev_data_time);

    /**
     * Function that calculates vertical velocity from altitude change. 
     * Useful to check events such as launch, burnt out.
     * @param altitude Current altitude read
     * @param prev_altitude Previous altitude read
     * @param elapsed_time Elapsed time betweet current and previous altitude read
     * @return vertical speed
    */
    static double vertical_speed(double altitude, double prev_altitude, unsigned int elapsed_time);

    /**
     * Function that calculates projected altitude from current altitude, velocity and acceleration. 
     * Useful to control rocket fins
     * @param altitude Current elapsed time of data read
     * @param velocity Previous elapsed time of data read
     * @param acceleration Previous elapsed time of data read
     * @return projected altitude
    */
    static double projected_altitude(double altitude, double velocity, double acceleration);

    /**
     * Function that calculates altitude from pressure. 
     * Useful to find projected altitude, apogee detection, etc.
     * @param pressure Current pressure read
     * @return altitude
    */
    static double altitude_from_pressure(double pressure);

    /**
     * Function that calculates altitude from pressure. 
     * Useful to find projected altitude, apogee detection, etc.
     * @param relative_vertical_direction Relative vertical direction from calibration
     * @return relative vertical acceleration
     * @see calibrate() function in Data class
    */
    static double relative_vertical_acceleration(int relative_vertical_direction, imu::Vector<3> linear_acceleration);

    /**
     * Function that checks wether rocket has launched, 
     * uses either altitude change or motion detection
     * @return false if launched, true if awaiting
    */
    static bool launch(double altitude_difference, double vertical_acceleration);

    /**
     * Function that checks wether rocket motor burnt out, 
     * uses either altitude change or negative acceleration
     * @return false if motor burnt out, true if motor still active
    */
    static bool burnt_out(double vertical_velocity, double vertical_acceleration);

    /**
     * Function that checks wether rocket reached its apogee, 
     * uses altitude and checks wether max altitude bigger than current altitude
     * @return false if reached apogee, true if still flything vertically upwards
    */
    static bool apogee(double altitude_difference, unsigned long delta_time);

    /**
     * Function that checks wether rocket has landed, 
     * uses altitude change in 5 second time range
     * @return false if landed, true if landing
    */
    static bool landed(double altitude_difference, double total_linear_acceleration, unsigned long delta_time);
};

#endif