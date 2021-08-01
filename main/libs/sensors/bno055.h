// BN055 functions
// get linear acceleration
// get qutarnions

/*   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3.3-5V DC
   Connect GROUND to common ground
*/

  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2

/**
 * Library of bno055.h
 * 
 * Developed for AURA Embedded System 2020
 * 
 * The library that works with BNO055 sensor
 * @1kzpro && @vladimireganov
 * @version 09/11/2020
 * 
 * @param BNO Class that controls BNO055 sensor
*/
// Including Adafruit Sensor Libraries
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

class BNO {
    private:
    Adafruit_BNO055 bno;
    unsigned long bno_time;
    unsigned long bno_time_magnit;
    

    public:
    bool connected = false;
    const int BNO055_NORMAL_DELAY_MS = 10;
    const int BNO055_MAGNIT_DELAY_MS = 50;
    const int BNO055_TEMP_DELAY_MS = 1000;

    
    BNO() {
        Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
        bno_time = millis() + BNO055_NORMAL_DELAY_MS;
        bno_time_magnit = millis() + BNO055_MAGNIT_DELAY_MS;
    };

    imu::Vector<3> getLinearAcc() {return bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);}
    imu::Quaternion getQutornions() {return bno.getQuat();}
    imu::Vector<3> getMagnit() {return bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);}
    imu::Vector<3> getGyro() {return bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);}
    imu::Vector<3> getEuler() {return bno.getVector(Adafruit_BNO055::VECTOR_EULER);}
    imu::Vector<3> getAccel() {return bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);}
    imu::Vector<3> getGrav() {return bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);}

    void getTemp() {}

    void getSystemStatus() {
        uint8_t system_status, self_test_result, system_error;
        bno.getSystemStatus(&system_status, &self_test_result, &system_error);
        Serial.print("System: Sys=");
        Serial.print(system_status, DEC);
        Serial.print(" Self Test=");
        Serial.print(self_test_result, DEC);
        Serial.print(" Error=");
        Serial.println(system_error, DEC);
    }

    void getCalibration(uint8_t *system, uint8_t *gyro, uint8_t *accel, uint8_t *mag) {
        // uint8_t system, gyro, accel, mag = 0;
        bno.getCalibration(system, gyro, accel, mag);
        // Serial.print("CALIBRATION: Sys=");
        // Serial.print(system, DEC);
        // Serial.print(" Gyro=");
        // Serial.print(gyro, DEC);
        // Serial.print(" Accel=");
        // Serial.print(accel, DEC);
        // Serial.print(" Mag=");
        // Serial.println(mag, DEC);
    }
    
    void activate() {
         // first parametr is id, second is adress
        if (!bno.begin(Adafruit_BNO055::OPERATION_MODE_NDOF)){
            //raise error
            #ifdef inotesting
            Serial.println("BNO did not connect");
            #endif
            connected = false;
        }
        else {
            #ifdef inotesting
            Serial.println("BNO connected");
            #endif
            connected = true;
        }
    }

    bool ready(double board_time) {
        if (bno_time < board_time) {
            bno_time = board_time + BNO055_NORMAL_DELAY_MS;
            return true;
        }
        return false;
    }

    bool ready_magnit(double board_time) {
        if (bno_time_magnit < board_time) {
            bno_time_magnit = board_time + BNO055_MAGNIT_DELAY_MS;
            return true;
        }
        return false;
    }
};
