#ifndef SENSOR_LIBS
#define SENSOR_LIBS
#include "../sensors/bmp390.h"
#include "../sensors/bno055.h"
#include "../logger.h"
#include "../sensors/radio.h"
#include "../sensors/button.h"
#include "../sensors/buzzer.h"
#include "../analyze.h"
#endif

/**
 * Data Class
 * 
 * Creates Class to represent all data from sensors and analyzed data in one stored instance.
*/
class Data
{
    private:
    int iterator;       // Data number

    File main_data;     // File instance of main data
    File backup_data;   // File instance of backup data

    BNO * bno;          // Instance of initialized BNO in Data class
    BMP390 * bmp;       // Instance of initialized BMP390 in Data class
    Logger * sds;          // Instance of initialized SDS in Data class
    Radio * radio;      // Instance of initialized Radio in Data class
    Button * button;    // Instance of initialized Button in Data class
    Buzzer * buzzer;    // Instance of initialized Buzzer in Data class

    public:
    bool available = false;     // Whether data was updated and ready to be written in sd card
    bool bmp_available = false; // Whether bmp data was updated
    bool bno_available = false; // Whether bno data was updated

    double pressure;            // Pressure read from BMP sensor
    double temperature;         // Tempurature read from BMP sensor

    imu::Vector<3> linear_acceleration; // 3-axis Linear acceleration from BNO sensor
    imu::Quaternion quaternions;        // Quaternions from BNO sensor
    imu::Vector<3> magnetometer;        // 3-axis Magnetometer from BNO sensor
    imu::Vector<3> gyroscope;           // 3-axis Gyroscope from BNO sensor
    imu::Vector<3> euler;               // Euler angles from BNO sensor
    imu::Vector<3> acceleration;        // 3-axis Acceleration (including gravity) from BNO sensor
    imu::Vector<3> gravity;             // 3-axis Gravity from BNO sensor


    double ground_altitude;     // Ground altitude recorded when armed
    double prev_altitude;       // Previous recorded altitude
    double altitude;            // Current recorded altitude

    double apogee = -100;        // Apogee altitude recorded
    unsigned int apogee_time;   // Time when apogee altitude recorded

    double land_altitude;       // Altitude of landed rocket
    unsigned int land_time;     // Time when rocket landed

    double prev_vertical_acceleration;  // Previous calculated vertical acceleration
    double vertical_acceleration;       // Current calculated vertical acceleration

    int relative_vertical_direction;        // Direction of BNO sensor when activated
    double relative_vertical_acceleration;  // Relative vertical accelration of rocket (direction up relative to BNO)

    double prev_total_linear_acceleration;  // Previous calculated total acceleration (without gravity)
    double total_linear_acceleration;       // Current calculated total acceleration (without gravity)

    double prev_vertical_velocity;  // Previous calculated vertical velocity
    double vertical_velocity;       // Current vertical velocity

    unsigned int prev_data_time;    // Time of previous recorded data
    unsigned int data_time;         // Time of current recorded data
    unsigned int elapsed;           // Elapsed time between current and previous recorded data in seconds

    double projected_altitude;      // Calculated projected altitude of rocket

    float latitude;     // Latitude value from GPS
    float longitude;    // Longitude value from GPS
    char lat;           // Latitude direction from GPS
    char lon;           // Longitude direction from GPS

    Data(BNO * bno, BMP390 * bmp, Logger * sds, Radio * radio, Button * button, Buzzer * buzzer)
    {
        this->bno = bno;
        this->bmp = bmp;
        this->sds = sds;
        this->radio = radio;
        this->button = button;
        this->buzzer = buzzer;
    }

    /**
     * Function that set time for apogee time. 
     * It is needed to start fixing apogee after time set.
    */
    void prepare_for_apogee()
    {
        apogee_time = millis();
    }
    

    /**
     * Function that set time for land time. 
     * It is needed to start fixing landed event after time set.
    */
    void prepare_for_land()
    {
        land_time = millis();
    }

    /**
     * Function that updates data from sensor read.
    */
    void get()
    {
        data_time = millis();   /// Set time of data read

        if(bmp->ready())
        {
            pressure = bmp->pressure();
            temperature = bmp->temperature();
            available = true;
            bmp_available = true;
        }
        
        if(bno->ready(data_time))
        {
            linear_acceleration = bno->getLinearAcc();
            quaternions = bno->getQutornions();
            gyroscope = bno->getGyro();
            euler = bno->getEuler();
            acceleration = bno->getAccel();
            gravity = bno->getGrav();
            available = true;
            bno_available = true;
        }

        if (bno->ready_magnit(data_time))
        {
            magnetometer = bno->getMagnit();
            available = true;
        }
    }

    /**
     * Function that analyzes raw data to calculate: 
     * altitude, apogee, vertical velocity, vertical acceleration, 
     * relative vertical acceleration, total linear acceleration.
    */
    void analyze()
    {
        if (bmp_available)
        {
            elapsed = RocketMath::elapsed_time(this->data_time, this->prev_data_time);
            prev_data_time = data_time;

            prev_altitude = altitude;
            altitude = RocketMath::altitude_from_pressure(this->pressure);

            if (apogee < altitude)
            {
                apogee = altitude;
                apogee_time = data_time;
            }
            
            prev_vertical_velocity = vertical_velocity;
            vertical_velocity = RocketMath::vertical_speed(this->altitude, this->prev_altitude, this->elapsed);

            bmp_available = false;
        }

        if (bno_available)
        {
            prev_vertical_acceleration = vertical_acceleration;
            vertical_acceleration = RocketMath::vertical_acceleration(this->linear_acceleration, RocketMath::transformation_matrix(this->quaternions));
            
            relative_vertical_acceleration = RocketMath::relative_vertical_acceleration(this->relative_vertical_direction, this->linear_acceleration);
        
            prev_total_linear_acceleration = total_linear_acceleration;
            total_linear_acceleration = RocketMath::total_linear_acceleration(this->linear_acceleration);

            bno_available = false;
        }

        projected_altitude = RocketMath::projected_altitude(altitude, vertical_velocity, vertical_acceleration);
    }

    /**
     * Starts (opens file) main data log.
     * Sets data headers to first line.
     * Logs error if file can not be opened.
    */
    void log_start()
    {
        String main_filename = sds->create_file_name("MAIN");
        main_data = SD.open(main_filename, O_CREAT | O_TRUNC | O_WRITE);
        if (main_data)
        {
            main_data.print("KEY\t");
            main_data.println(sds->key);
            main_data.print("Iteration,");
            main_data.print("Time,");
            main_data.print("Pressure,");
            main_data.print("Temperature,");
            main_data.print("Lin_Acc_x,");
            main_data.print("Lin_Acc_y,");
            main_data.print("Lin_Acc_z,");
            main_data.print("Qutornions_w,");
            main_data.print("Qutornions_x,");
            main_data.print("Qutornions_y,");
            main_data.print("Qutornions_z,");
            main_data.print("Magnetometer_x,");
            main_data.print("Magnetometer_y,");
            main_data.print("Magnetometer_z,");
            main_data.print("Gyroscope_x,");
            main_data.print("Gyroscope_y,");
            main_data.print("Gyroscope_z,");
            main_data.print("Acceleration_x,");
            main_data.print("Acceleration_y,");
            main_data.print("Acceleration_z,");
            main_data.print("Gravity_x,");
            main_data.print("Gravity_y,");
            main_data.print("Gravity_z,");
            main_data.print("Altitude,");
            main_data.print("Apogee,");
            main_data.print("Acceleration_z,");
            main_data.print("Vertical_velocity,");
            main_data.println("Projected_Altitude,");
            main_data.flush();
            sds->logInfo("Main data file successfully opened");
        }
        else
        {
            Serial.println("Error opening main data file");
            sds->logError("Error opening main data file");
        }
    }

    /**
     * Starts (opens file) backup data log.
     * Sets data headers to first line.
     * Logs error if file can not be opened.
    */
    void backup_start()
    {
        String backup_filename = sds->create_file_name("BACKUP");
        backup_data = SD.open(backup_filename, O_CREAT | O_TRUNC | O_WRITE);
        if (backup_data)
        {
            backup_data.print("KEY\t");
            backup_data.println(sds->key);
            backup_data.print("Iteration,");
            backup_data.print("Time,");
            backup_data.print("Pressure,");
            backup_data.print("Temperature,");
            backup_data.print("Lin_Acc_x,");
            backup_data.print("Lin_Acc_y,");
            backup_data.print("Lin_Acc_z,");
            backup_data.print("Qutornions_w,");
            backup_data.print("Qutornions_x,");
            backup_data.print("Qutornions_y,");
            backup_data.print("Qutornions_z,");
            backup_data.print("Magnetometer_x,");
            backup_data.print("Magnetometer_y,");
            backup_data.print("Magnetometer_z,");
            backup_data.print("Gyroscope_x,");
            backup_data.print("Gyroscope_y,");
            backup_data.print("Gyroscope_z,");
            backup_data.print("Acceleration_x,");
            backup_data.print("Acceleration_y,");
            backup_data.print("Acceleration_z,");
            backup_data.print("Gravity_x,");
            backup_data.print("Gravity_y,");
            backup_data.print("Gravity_z,");
            backup_data.print("Altitude,");
            backup_data.print("Apogee,");
            backup_data.print("Acceleration_z,");
            backup_data.print("Vertical_velocity,");
            backup_data.println("Projected_Altitude,");
            backup_data.flush();
            sds->logInfo("Backup data file successfully opened");
        }
        else
        {
            Serial.println("Error opening backup data file");
            sds->logError("Error opening backup data file");
        }
    }

    /**
     * Writes updated data to main and backup files.
     * Automatically saves backup data after 100 data update
     * Logs error if file can not be opened.
    */
    void log()
    {
        if (available && main_data)
        {
            main_data.print(iterator);
            main_data.print(',');
            main_data.print(data_time);
            main_data.print(',');
            main_data.print(pressure);
            main_data.print(',');
            main_data.print(temperature);
            main_data.print(',');
            main_data.print(linear_acceleration[0]);
            main_data.print(',');
            main_data.print(linear_acceleration[1]);
            main_data.print(',');
            main_data.print(linear_acceleration[2]);
            main_data.print(',');
            main_data.print(quaternions.w());
            main_data.print(',');
            main_data.print(quaternions.x());
            main_data.print(',');
            main_data.print(quaternions.y());
            main_data.print(',');
            main_data.print(quaternions.z());
            main_data.print(',');
            main_data.print(magnetometer[0]);
            main_data.print(',');
            main_data.print(magnetometer[1]);
            main_data.print(',');
            main_data.print(magnetometer[2]);
            main_data.print(',');
            main_data.print(gyroscope[0]);
            main_data.print(',');
            main_data.print(gyroscope[1]);
            main_data.print(',');
            main_data.print(gyroscope[2]);
            main_data.print(',');
            main_data.print(acceleration[0]);
            main_data.print(',');
            main_data.print(acceleration[1]);
            main_data.print(',');
            main_data.print(acceleration[2]);
            main_data.print(',');
            main_data.print(gravity[0]);
            main_data.print(',');
            main_data.print(gravity[1]);
            main_data.print(',');
            main_data.print(gravity[2]);
            main_data.print(',');
            main_data.print(altitude);
            main_data.print(',');
            main_data.print(apogee);
            main_data.print(',');
            main_data.print(vertical_acceleration);
            main_data.print(',');
            main_data.print(vertical_velocity);
            main_data.print(',');
            main_data.print(projected_altitude);
            main_data.println(',');
            main_data.flush();
        }
        else
        {
            Serial.println("Error opening main data file");
            sds->logError("Error opening main data file");
        }

        if (available && backup_data)
        {
            backup_data.print(iterator);
            backup_data.print(',');
            backup_data.print(data_time);
            backup_data.print(',');
            backup_data.print(pressure);
            backup_data.print(',');
            backup_data.print(temperature);
            backup_data.print(',');
            backup_data.print(linear_acceleration[0]);
            backup_data.print(',');
            backup_data.print(linear_acceleration[1]);
            backup_data.print(',');
            backup_data.print(linear_acceleration[2]);
            backup_data.print(',');
            backup_data.print(quaternions.w());
            backup_data.print(',');
            backup_data.print(quaternions.x());
            backup_data.print(',');
            backup_data.print(quaternions.y());
            backup_data.print(',');
            backup_data.print(quaternions.z());
            backup_data.print(',');
            backup_data.print(magnetometer[0]);
            backup_data.print(',');
            backup_data.print(magnetometer[1]);
            backup_data.print(',');
            backup_data.print(magnetometer[2]);
            backup_data.print(',');
            backup_data.print(gyroscope[0]);
            backup_data.print(',');
            backup_data.print(gyroscope[1]);
            backup_data.print(',');
            backup_data.print(gyroscope[2]);
            backup_data.print(',');
            backup_data.print(acceleration[0]);
            backup_data.print(',');
            backup_data.print(acceleration[1]);
            backup_data.print(',');
            backup_data.print(acceleration[2]);
            backup_data.print(',');
            backup_data.print(gravity[0]);
            backup_data.print(',');
            backup_data.print(gravity[1]);
            backup_data.print(',');
            backup_data.print(gravity[2]);
            backup_data.print(',');
            backup_data.print(altitude);
            backup_data.print(',');
            backup_data.print(apogee);
            backup_data.print(',');
            backup_data.print(vertical_acceleration);
            backup_data.print(',');
            backup_data.print(vertical_velocity);
            backup_data.print(',');
            backup_data.print(projected_altitude);
            backup_data.println(',');
            iterator++;
            backup_data.flush();
            available = false;
        }
        else
        {
            Serial.println("Error opening backup data file");
            sds->logError("Error opening backup data file");
        }
    }

    /**
     * Writes updated data to backup file.
     * Automatically saves backup data after 100 data update
     * Logs error if file can not be opened.
    */
    void log_backup()
    {
        if (available && backup_data)
        {
            backup_data.print(iterator);
            backup_data.print(',');
            backup_data.print(data_time);
            backup_data.print(',');
            backup_data.print(pressure);
            backup_data.print(',');
            backup_data.print(temperature);
            backup_data.print(',');
            backup_data.print(linear_acceleration[0]);
            backup_data.print(',');
            backup_data.print(linear_acceleration[1]);
            backup_data.print(',');
            backup_data.print(linear_acceleration[2]);
            backup_data.print(',');
            backup_data.print(quaternions.w());
            backup_data.print(',');
            backup_data.print(quaternions.x());
            backup_data.print(',');
            backup_data.print(quaternions.y());
            backup_data.print(',');
            backup_data.print(quaternions.z());
            backup_data.print(',');
            backup_data.print(magnetometer[0]);
            backup_data.print(',');
            backup_data.print(magnetometer[1]);
            backup_data.print(',');
            backup_data.print(magnetometer[2]);
            backup_data.print(',');
            backup_data.print(gyroscope[0]);
            backup_data.print(',');
            backup_data.print(gyroscope[1]);
            backup_data.print(',');
            backup_data.print(gyroscope[2]);
            backup_data.print(',');
            backup_data.print(acceleration[0]);
            backup_data.print(',');
            backup_data.print(acceleration[1]);
            backup_data.print(',');
            backup_data.print(acceleration[2]);
            backup_data.print(',');
            backup_data.print(gravity[0]);
            backup_data.print(',');
            backup_data.print(gravity[1]);
            backup_data.print(',');
            backup_data.print(gravity[2]);
            backup_data.print(',');
            backup_data.print(altitude);
            backup_data.print(',');
            backup_data.print(apogee);
            backup_data.print(',');
            backup_data.print(vertical_acceleration);
            backup_data.print(',');
            backup_data.print(vertical_velocity);
            backup_data.print(',');
            backup_data.print(projected_altitude);
            backup_data.println(',');
            iterator++;
            backup_data.flush();
            available = false;
        }
        else
        {
            Serial.println("Error opening backup data file");
            sds->logError("Error opening backup data file");
        }
    }

    /**
     * Saves main and backup data files.
    */
    void save()
    {
        if (main_data) {main_data.flush();}
        else
        {
            Serial.println("Error saving main data file");
            sds->logError("Error saving main data file");
        }

        if (backup_data) {backup_data.flush();}
        else
        {
            Serial.println("Error saving backup data file");
            sds->logError("Error saving backup data file");
        }
    }

    /**
     * Saves and closes main data file.
    */
    void log_end()
    {
        if (main_data) {main_data.close();}
        else
        {
            Serial.println("Error saving and closing main data file");
            sds->logError("Error saving and closing main data file");
        }
    }


    /**
     * Function that verifies if all sensors are working
     * @return true if all sensors are connected and working, false if some issue happened
    */
    bool sensors() {
        sds->logSensors(bno->connected, bmp->connected, sds->connected);

        if (bno->connected && bmp->connected && sds->connected) {
            button->status_init();
            buzzer->async_sound(2000, 300);
            return true;
        }
        
        buzzer->multiple_sound(2000, 300, 5, 100);
        return false;
    }

    /**
     * Function that checks wether to stay or exit from standby mode
     * @return true if all sensors are connected and working, false if some issue happened
    */
    bool standby() {
        bool r = radio->detect_launch_request;
        bool b = button->is_hold_pressed();

        if (r || b) {
            if (b) {
                sds->logTrigger("Switching to detect launch mode from button request");
            }

            if (r) {
                sds->logTrigger("Switching to detect launch mode from radio request");
            }

            radio->detect_launch_request = false;
            button->hold_reset();
            buzzer->multiple_async_sound(2000, 300, 2, 300);
            return false;
        }
        return true;
    }

    /**
     * Function that checks wether rocket has launched, 
     * uses either altitude change or motion detection
     * @return false if launched, true if awaiting
    */
    bool launch() {
        int dh = abs(altitude - ground_altitude); // && (dh >= 50)
        if ((vertical_acceleration >= 20) && (dh >= 50)) {
            sds->logTrigger_double("Vertical acceleration", vertical_acceleration);
            sds->logTrigger_double("Altitude change", dh);
            buzzer->multiple_async_sound(2000, 300, 2, 300);
            return false;   // when false returned while loop teminates
        }
        return true;
    }

    /**
     * Function that checks wether rocket motor burnt out, 
     * uses either altitude change or negative acceleration
     * @return false if motor burnt out, true if motor still active
    */
    bool burnt_out() {
        // return false if vertical acceleration is less than 0
        // Velocity unit is m/s and accel m/s^2
        if ((vertical_velocity <= 100) && (vertical_acceleration <= 0.0)){
            sds->logTrigger_double("Vertical velocity", vertical_velocity);
            sds->logTrigger_double("Vertical acceleration", vertical_acceleration);
            return false;   // when false returned while loop teminates
        }
        return true;
    }

    /**
     * Function that checks wether rocket reached its apogee, 
     * uses altitude and checks wether max altitude bigger than current altitude
     * @return false if reached apogee, true if still flything vertically upwards
    */
    bool c_apogee() {
        double dh = apogee - altitude;
        double dt = millis() - apogee_time;
        // Apogee is same as max altitude
        if (dh >= 3 && dt >= 5000) {
            sds->logTrigger<double>("Hight difference", dh);
            sds->logTrigger<unsigned int>("Await time", dt);
            // buzzer->multiple_async_sound(2000, 300, 2, 300);
            return false;   // when false returned while loop teminates
        }
        return true;
    }

    /**
     * Function that checks wether rocket has landed, 
     * uses altitude change in 5 second time range
     * @return false if landed, true if landing
    */
    bool landed() {
        // return false if altitude change less that 5 m in 5 sec
        double dh = altitude - land_altitude;
        double dt = millis() - land_time;
        if (abs(dh) <= 1 && total_linear_acceleration <= 0.8) {
            if(dt >= 5000) {
                sds->logTrigger<double>("Altitude difference: ", dh);
                sds->logTrigger<double>("Total linear acceleration: ", total_linear_acceleration);
                buzzer->multiple_async_sound(2000, 300, 3, 100);
                return false;   // when false returned while loop teminates
            }
            return true;
        } else {
            land_altitude = altitude;
            land_time = millis();
        }
        return true;
    }

    /**
     * Function that fixes rocket relative to BNO orientation
     * @return false if did not could not fix orientation
    */
    bool calibrate()
    {
        double x = gravity[0];
        double y = gravity[1];
        double z = gravity[2];

        ground_altitude = altitude;
        String rvd = "Wrong starting position";

        if (x > 9.0) {
            relative_vertical_direction = 1;
            rvd = "X+";
        } 
        else if (x < -9.0) 
        {
            relative_vertical_direction = 2;
            rvd = "X-";
        }
        else if (y > 9.0)
        {
            relative_vertical_direction = 3;
            rvd = "Y+";
        }
        else if (y < -9.0)
        {
            relative_vertical_direction = 4;
            rvd = "Y-";
        }
        else if (z > 9.0)
        {
            relative_vertical_direction = 5;
            rvd = "Z+";
        }
        else if (z < -9.0)
        {
            relative_vertical_direction = 6;
            rvd = "Z-";
        }

        relative_vertical_direction = 0;

        sds->logCalibrate(ground_altitude, rvd);
        return true;
    }

    void async_tasks()
    {
        button->check();
        buzzer->check();
        radio->check();

        // Async tasks
        button_tasks();
        radio_tasks();
    }

    private:
    /**
     * Function that checks and handles all radio requests.
    */
    void radio_tasks()
    {
        if (radio->reset_system_request)
        {
            radio->reset_system_request = false;
            sds->logInfo("Reset request from button");
            buzzer->sound(2000, 1000);
            CPU_RESTART;
        }

        if (radio->sensor_info_request) {
            radio->sensor_info_request = false;
            Serial1.print(':');
            Serial1.print('s');
            Serial1.print(bno->connected);
            Serial1.print(',');
            Serial1.print(bmp->connected);
            Serial1.print(',');
            Serial1.print(sds->connected);
            Serial1.print(';');
            Serial1.flush();
        }

        if (radio->bno_calibration_request)
        {
            radio->bno_calibration_request = false;
            uint8_t system, gyro, accel, mag = 0;
            bno->getCalibration(&system, &gyro, &accel, &mag);
            Serial1.print(':');
            Serial1.print('b');
            Serial1.print(system, DEC);
            Serial1.print(',');
            Serial1.print(gyro, DEC);
            Serial1.print(',');
            Serial1.print(accel, DEC);
            Serial1.print(',');
            Serial1.print(mag, DEC);
            Serial1.print(';');
            Serial1.flush();
        }
    }

    /**
     * Function that checks for different states of button, 
     * wether its short pressed or long pressed
    */
    void button_tasks()
    {
        if (button->is_long_hold_pressed())
        {
            sds->logInfo("Reset request from button");
            buzzer->sound(2000, 1000);
            CPU_RESTART;
        }
    }
};