/**
 * @file flight_math.h
 * @author Kazybek Mizam (kzm0099@auburn.edu)
 * @author Austin Le... (you@domain.com) // TODO last name and email
 * @brief Library with IMU math functions that are used to process
 * raw data like pressure, quaternions to useful data such as vertical velocity
 * and vertical acceleration.
 * 
 * Used by flight_data.h
 * 
 * @version 0.1
 * @date 2021-03-19
 * 
 * @copyright Copyright (c) AURA Embedded Systems 2021
 * 
 */
#ifndef ROCKET_IMU_H
#define ROCKET_IMU_H
#include <math.h>
#include <stdlib.h>

#include "imu_utility/imumaths.h" // From Adafruit BNO055

const double GRAVITY = 9.80665;

class RocketIMUMath
{
    public:
    /**
     * Matrix to transform from rockets frame towards ground frame. 
     * Useful for finding vertical acceleration
     * @param quaternions Quaternions of imu::Quaternion type
     * @return transformation matrix
     * @see RocketMath::vertical_acceleration(imu::Vector<3> linear_acceleration);
    */
    static imu::Matrix<3> transformation_matrix(imu::Quaternion quaternions)
    {
        imu::Quaternion quat = quaternions;
        double a = quat.w();
        double b = quat.x();
        double c = quat.y();
        double d = quat.z();

        imu::Vector<3> row1 =
            imu::Vector<3>(a * a + b * b - c * c - d * d, 2 * b * c - 2 * a * d,
                        2 * b * d + 2 * a * c);
        imu::Vector<3> row2 =
            imu::Vector<3>(2 * b * c + 2 * a * d, a * a - b * b + c * c - d * d,
                        2 * c * d - 2 * a * b);
        imu::Vector<3> row3 =
            imu::Vector<3>(2 * b * d - 2 * a * c, 2 * c * d + 2 * a * b,
                        a * a - b * b - c * c + d * d);

        imu::Matrix<3> trans_mat;
        trans_mat.vector_to_row(row1, 0);
        trans_mat.vector_to_row(row2, 1);
        trans_mat.vector_to_row(row3, 2);
        return trans_mat;
    }

    /**
     * Function that returns vertical acceleration measured with ground frame. 
     * Useful to check if rocket engine burnt out
     * @param linear_acceleration Linear Acceleration from accelerometer
     * @return vertical acceleration
    */
    static double vertical_acceleration(imu::Vector<3> linear_acceleration, imu::Matrix<3> transformation_matrix)
    {
        imu::Matrix<3> trans_mat = transformation_matrix;
        imu::Vector<3> accel = linear_acceleration;
        imu::Vector<3> inertial_accel;
        for (int i = 0; i < 3; i++)
        {
            imu::Vector<3> row = trans_mat.row_to_vector(i);
            inertial_accel[i] = row.dot(accel);
        }
        return inertial_accel[2];
    }

    /**
     * Function that returns total linear (without gravity) acceleration from linear acceleration. 
     * Useful to detect any motion
     * @param linear_acceleration Linear Acceleration from accelerometer
     * @return total acceleration
    */
    static double total_linear_acceleration(imu::Vector<3> linear_acceleration)
    {
        double x = linear_acceleration[0];
        double y = linear_acceleration[1];
        double z = linear_acceleration[2];
        return sqrt(x*x + y*y + z*z);
    }

    /**
     * Function that calculates vertical velocity from altitude change. 
     * Useful to check events such as launch, burnt out.
     * @param altitude Current altitude read
     * @param prev_altitude Previous altitude read
     * @param elapsed_time_millis Elapsed time in milliseconds between current and previous altitude read
     * @return vertical speed
    */
    static double vertical_speed(double altitude, double prev_altitude, unsigned int elapsed_time_millis)
    {
        double delta_alt = (altitude - prev_altitude) * 1000;
        return (double)(delta_alt / elapsed_time_millis);
    }

    /**
     * Function that calculates projected altitude from current altitude, velocity and acceleration. 
     * Useful to control rocket fins
     * @param altitude Current elapsed time of data read
     * @param velocity Previous elapsed time of data read
     * @param acceleration Previous elapsed time of data read
     * @return projected altitude
    */
    static double projected_altitude(double altitude, double velocity, double acceleration)
    {
        // return abs((velocity * velocity) /
        //         (2.0 * acceleration + GRAVITY))) *
        //         log(abs(acceleration / GRAVITY)) +
        //     altitude;

        return fabs(velocity * velocity) / (2.0 * fabs(acceleration + GRAVITY)) * 
		log(fabs(acceleration) / GRAVITY) + altitude;
    }

    /**
     * Function that calculates altitude from pressure. 
     * Useful to find projected altitude, apogee detection, etc.
     * @param pressure Current pressure read
     * @return altitude
    */
    static double altitude_from_pressure(double pressure)
    {
        double seaLevelhPa = 1013.25;
        double alt;

        pressure /= 100;
        alt = 44330 * (1.0 - pow(pressure / seaLevelhPa, 0.1903));

        return alt;
    }

    /**
     * Function that calculates altitude from pressure. 
     * Useful to find projected altitude, apogee detection, etc.
     * @param relative_vertical_direction Relative vertical direction from calibration
     * @return relative vertical acceleration
     * @see calibrate() function in Data class
    */
    static double relative_vertical_acceleration(int relative_vertical_direction, imu::Vector<3> linear_acceleration)
    {
        switch (relative_vertical_direction)
        {
        case 1:
            return linear_acceleration[0];
            break;
        case 2:
            return linear_acceleration[1];
            break;
        case 3:
            return linear_acceleration[2];
            break;
        case 4:
            return linear_acceleration[0] * (-1);
            break;
        case 5:
            return linear_acceleration[1] * (-1);
            break;
        case 6:
            return linear_acceleration[2] * (-1);
            break;
        
        default:
            return 0.0;
            break;
        }

        return 0.0;
    }

    static double low_pass_filter(double previous_value, double current_value, unsigned long elapsed_time) {
        const double RC = 0.3;
        const double alpha = elapsed_time / (RC + elapsed_time);
        return (alpha * current_value) + (1.0 - alpha) * previous_value;
    }
};
#endif