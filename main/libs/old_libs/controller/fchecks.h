#include "./fdata.h"
#include "../sensors/bmp280.h"
#include "../sensors/bno055.h"
#include "../sensors/fsd.h"
#include "../sensors/gps.h"
#include "../sensors/radio.h"
#include "../sensors/button.h"
#include "../sensors/buzzer.h"

/**
 * Class Checks
 * 
 * This class handles all event, module checks and completes requested tasks
*/
class RocketChecks {

    public:

    /**
     * Function that verifies if all sensors are working
     * @return true if all sensors are connected and working, false if some issue happened
    */
    // bool sensors(bool altimeter_working, bool imu_sensor_working, Logger logger) {

    //     logger->logSensors(altimeter_working, imu_sensor_working);

    //     if (altimeter_working && imu_sensor_working) {
    //         return true;
    //     }
        
    //     return false;
    // }

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
     * Function that checks button, buzzer, sd and radio.
     * Complete tasks if requested.
    */
    void async_tasks()
    {
        button->check();
        buzzer->check();
        sds->check();
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
        if (radio->save_data_request)
        {
            radio->save_data_request = false;
            sds->flush();
        }
        
        if (radio->reset_system_request)
        {
            radio->reset_system_request = false;
            buzzer->sound(2000, 1000);
            CPU_RESTART;
        }

        if (radio->bno_calibration_request)
        {
            radio->bno_calibration_request = false;
            uint8_t system, gyro, accel, mag = 0;
            bno->getCalibration(&system, &gyro, &accel, &mag);
            radio->transmit(&system, &gyro, &accel, &mag);
        }

        if (radio->download_request)
        {
            radio->download_request = false;
            sds->data_upload_request();
        }

        if (data->upload_data != "")
        {
            radio->transmit(data->upload_data);
            data->upload_data = "";
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
            // Serial.println("Button Long Hold Pressed");
            buzzer->sound(2000, 1000);
            CPU_RESTART;
        }
    }
}