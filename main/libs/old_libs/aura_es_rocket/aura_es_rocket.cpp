#include "aura_es_rocket.h"
/**
 * Class RocketMath
 * 
 * This class contains all imu calculations of rocket
*/

Rocket::Rocket() {}

imu::Matrix<3> Rocket::transformation_matrix(imu::Quaternion quaternions)
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
double Rocket::vertical_acceleration(imu::Vector<3> linear_acceleration, imu::Matrix<3> transformation_matrix)
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

double Rocket::total_linear_acceleration(imu::Vector<3> linear_acceleration)
{
    double x = linear_acceleration[0];
    double y = linear_acceleration[1];
    double z = linear_acceleration[2];
    return sqrt(x * x + y * y + z * z);
}

double Rocket::elapsed_time(unsigned int data_time, unsigned int prev_data_time)
{
    return data_time - prev_data_time;
}

double Rocket::vertical_speed(double altitude, double prev_altitude, unsigned int elapsed_time)
{
    double delta_alt = (altitude - prev_altitude) * 1000;
    return (double)(delta_alt / elapsed_time);
}

double Rocket::projected_altitude(double altitude, double velocity, double acceleration)
{
    return abs((velocity * velocity) /
               (2 * (acceleration + GRAV))) *
               log(abs(acceleration / GRAV)) +
           altitude;
}

double Rocket::altitude_from_pressure(double pressure)
{
    double seaLevelhPa = 1013.25;
    double alt;

    pressure /= 100;
    alt = 44330 * (1.0 - pow(pressure / seaLevelhPa, 0.1903));

    return alt;
}

double Rocket::relative_vertical_acceleration(int relative_vertical_direction, imu::Vector<3> linear_acceleration)
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

bool Rocket::launch(double altitude_difference, double vertical_acceleration)
{
    if ((vertical_acceleration >= 20) && (altitude_difference >= 50))
    {
        // sds->logTrigger_double("Vertical acceleration", vertical_acceleration);
        // sds->logTrigger_double("Altitude change", dh);
        // buzzer->multiple_async_sound(2000, 300, 2, 300);
        return false; // when false returned while loop teminates
    }
    return true;
}

bool Rocket::burnt_out(double vertical_velocity, double vertical_acceleration)
{
    // return false if vertical acceleration is less than 0
    // Velocity unit is m/s and accel m/s^2
    if ((vertical_velocity <= 100) && (vertical_acceleration <= 0.0))
    {
        // sds->logTrigger_double("Vertical velocity", vertical_velocity);
        // sds->logTrigger_double("Vertical acceleration", vertical_acceleration);
        return false; // when false returned while loop teminates
    }
    return true;
}

bool Rocket::apogee(double altitude_difference, unsigned long delta_time)
{
    // double dh = apogee - altitude;
    // double dt = millis() - apogee_time;
    // Apogee is same as max altitude
    if (altitude_difference >= 3 && delta_time >= 5000)
    {
        // sds->logTrigger<double>("Hight difference", dh);
        // sds->logTrigger<unsigned int>("Await time", dt);
        // buzzer->multiple_async_sound(2000, 300, 2, 300);
        return false; // when false returned while loop teminates
    }
    return true;
}

bool Rocket::landed(double altitude_difference, double total_linear_acceleration, unsigned long delta_time)
{
    // return false if altitude change less that 5 m in 5 sec
    if (abs(altitude_difference) <= 1 && total_linear_acceleration <= 0.8)
    {
        if (delta_time >= 5000)
        {
            // sds->logTrigger<double>("Altitude difference: ", dh);
            // sds->logTrigger<double>("Total linear acceleration: ", total_linear_acceleration);
            // buzzer->multiple_async_sound(2000, 300, 3, 100);
            return false; // when false returned while loop teminates
        }
        return true;
    }
    else
    {
        // land_altitude = altitude;
        // land_time = millis();
    }
    return true;
}