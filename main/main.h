// Core Teensy4.0 software restart
#define CPU_RESTART_ADDR (uint32_t *)0xE000ED0C
#define CPU_RESTART_VAL 0x5FA0004
#define CPU_RESTART (*CPU_RESTART_ADDR = CPU_RESTART_VAL);

// Including I2C, SPI and EEPROM libraries
#include <Wire.h>
#include <SPI.h>
#include <EEPROM.h>

// Including SD Card Sensor
#include <SD.h>

// Define default settings of running a main code
// #define logbinary   //If log in binary mode
#define inotesting  // Default debug prints
// #define gs_radio
// #define size_of_debug

// Including local library
#include "libs/communications/beaglebone/beagle_serial.h"
#include "libs/io_devices/button.h"
#include "libs/io_devices/buzzer.h"
#include "libs/flight_logger.h"
#include "libs/flight_data.h"
#include "libs/flight_states.h"
#include "libs/communications/ground_station/radio_packet.h"

#ifdef gs_radio
#include "libs/communications/ground_station/serial_radio_ground.h"
#else
#include "libs/communications/ground_station/serial_radio.h"
#endif

/**
 * @brief Object that represens Adafruit SD breakout board
 * This class is a wrapper of Adafruit_BMP390 library
 * specifically made for this project.
 * 
 * This mini SD board is used for realtime logging of data.
 * The Logger class also includes useful methods such as
 * creating unique file, logging events in separate log file
 * every time when system turns on.
 * 
 * In this project the class is used as data writing manager.
 * It has 2 purposes:
 * 1. Real time log flight data (please refer to Data class
 *  for more information)
 * 2. Log system information such as "Setup started" etc
 *      (please refer to setup and loop functions);
 *  Flight events (Launched, Burnt out...)
 *      (please refer to independent_event_test function);
 * 
 * Please refer to Logger class to see complete functionality.
 */
Logger logger = Logger();

/**
 * @brief Object that represens bno390 sensor
 * This class is a wrapper of Adafruit_BMP390 library
 * specifically made for this project.
 * 
 * This sensor is used to measure pressure and temperature
 * By processing pressure we can get altitude, and
 * using altitude and elapsed time we can get vertical velocity.
 * 
 * Please refer to BMP390 class to see complete functionality.
 */
BMP390 bmp = BMP390();

/**
 * @brief Object that represens bno055 sensor
 * This class is a wrapper of Adafruit_BNO055 library
 * specifically made for this project.
 * 
 * This sensor is used to measure acceleration,
 * gyroscope (rocket orientation) and magnetometer
 * to make all measurements absolute relative to earth.
 * 
 * Please refer to BNO class to see complete functionality.
 */
BNO bno = BNO();

/**
 * @brief Object that represents xbee radio device.
 * Radio is an xbee that works with serial communication.
 * This class is used to simplify implementation and
 * communication between rocket and ground station.
 * 
 * It has predefined methods to receive, process and transmit (reply)
 * data between rocket and ground station.
 * 
 * The Radio class handles encoding and decoding messages
 * with storing requests as flags.
 * 
 * There are many radio commands which are handled in radio_tasks()
 * function below, that shows how radio is used.
 * 
 * Please refer to Radio class to see complete functionality.
 */
Radio radio_device = Radio(&Serial2);

/**
 * @brief Object that represents regular pushbutton.
 * This object has some useful predefined methods,
 * to check different button interrupts.
 * 
 * In this project button is used for 2 purpose:
 * 1. Arm system by holding for 3 seconds
 * 2. Reset system by holding more than 10 seconds
 * 
 * Please refer to Button class to see complete functionality.
 */
Button button = Button(3); // Pin 3 in Fullscale 2021 PCB V3

/**
 * @brief Object that represents buzzer.
 * This object has some useful predefined methods,
 * to easily create different sounds working in background.
 * 
 * In this project it is used to create sounds based on events
 * that are happened such as:
 * 1. Arming system
 * 2. Transition from one flight state to another
 * 3. Verify all sensors are connected and working properly
 * 4. Verify that data logging is started
 * 5. Beep for long time if system was reset.
 * 
 * Please refer to Buzzer class to see complete functionality.
 */
Buzzer buzzer = Buzzer(2); // Pin 2 in Fullscale 2021 PCB V3


/**
 * @brief Object that represents Adafruit GPS
 * This class is a wrapper of Adafruit_GPS library
 * specifically made for this project.
 * 
 * This sensor is used for realtime tracking of location.
 * It is logged and transmitted via radio to the ground station
 * 
 * In this project GPS is useful for finding exact location of
 * rocket, when it lands in too far from launch site, which is
 * untrakable and hard to find.
 * 
 * Please refer to GPS class to see complete functionality.
 */
GPS gps = GPS(&Serial5);

/**
 * @brief Object that represents BeagleBone Blue (BBB) device 
 * This class is serial communication with BBB specifically
 * made for this project.
 * 
 * BeagleBone Blue is a seperate device that is connected to
 * current system via serial communication. BBB is responsible
 * for full Altitude Control during the flight. Also in other words
 * we call it ACS which stands for Altitude Control System.
 * BeagleBone Blue is same as small computer that has built-in
 * sensors and pins to directly connect with servos, motors etc.
 * 
 * In the ACS project it is responsible to active direct control of
 * servo grid fins, which slow downs rocket when reaching apogee.
 * 
 * To know if BBB work properly during the flight, BBB needs to
 * transmit data to ground station. Since current or Teensy Backup
 * System has already radio, we decided to use it as retransmitter.
 * Because adding second radio makes system redundand.
 * 
 * How it works?
 * Basically useful data from BBB sent to Teensy and Teensy combines
 * it with own data sending full pack of data to Ground Station.
 * 
 * The class handles encoding and decoding of data
 * 
 * In this project, it is used for 2 purposes:
 * 1. Arming BBB system
 * 2. Receiving realtime data update such as (Flight events and statuses)
 */
BeagleBoneCommunication beagle_device = BeagleBoneCommunication(&Serial1);

/**
 * @brief Object that represents all Data of Teensy system 
 * This class is responsible for managing, updating, analyzing
 * and logging real time data from sensors.
 * TODO finish documentation.
 * 
 */
Data data = Data(&bno, &bmp, &gps, beagle_device.get_packet(), &realtime_data_packet);

unsigned long rocket_broadcast_timer = millis(); // TODO avoid global timers

void io_tasks();
void radio_tasks();
void button_tasks();
void independent_event_test();
void realtime_data();       // Todo use transmit sending binary instead

void setup()
{
    #ifdef inotesting
    Serial.begin(1000000);
    Serial.println("Setup started!");
    #endif

    delay(1000);

    logger.activate();

    logger.logInfo("Setup started");

    button.activate();
    buzzer.activate();

    bmp.activate();
    bno.activate();
    radio_device.activate();
    beagle_device.activate();
    gps.activate();

    #ifdef size_of_debug
    Serial.print("%iSize of data packet");
    Serial.print(sizeof(realtime_data_packet_t));
    Serial.print(";");
    // Some Debug
    Serial.print("Size of float is: ");
    Serial.println(sizeof(float));
    delay(1000);
    Serial.print("Size of double is: ");
    Serial.println(sizeof(double));
    Serial.print("Size of int is: ");
    Serial.println(sizeof(int));
    Serial.print("Size of unsigned long is: ");
    Serial.println(sizeof(unsigned int));
    Serial.print("Size of teensy_packet is: ");
    Serial.println(data.log_data_size);
    #endif

    File backup_file = SD.open(logger.create_file_name("BACKUP"), FILE_WRITE);
    if (data.backup_start(&backup_file, logger.key)) {
        logger.logInfo("Backup data file successfully opened");
    } else {
        logger.logError("Error opening backup data file");
    }

    data.calibrate();

    logger.logSensors(bno.connected, bmp.connected, logger.connected);
    if (bno.connected && bmp.connected && logger.connected) {
        if (backup_file) {
            buzzer.async_sound(2000, 300);
        } else {
            buzzer.multiple_sound(2000, 500, 3, 500);
        }
        
    } else {
        buzzer.multiple_sound(2000, 300, 5, 100);
    }

    #ifdef inotesting
    Serial.println("Setup done!");
    #endif
    logger.logInfo("Setup done");
}

void loop()
{
    while(true) {
        data.get();
        data.analyze();
        #ifdef logbinary
        data.log_binary();
        #else
        data.log();
        #endif
        beagle_device.transmit(millis());
        
        io_tasks();
        independent_event_test();
    }
    
    logger.logInfo("Flight code finished, this should not have happen!");
}

void io_tasks()
{
    button.run();       // Processes all button interrupts
    buzzer.run();       // Processes active buzzer sound as coroutine
    radio_device.run(); // Receive and process radio requests
    beagle_device.run();// Receive and process ACS requests

    // Coroutine tasks
    button_tasks();
    radio_tasks();
}

#ifdef gs_radio
/**
 * @brief Function that checks requests and handles them.
 * Called in void loop() 
 */
void radio_tasks()
{
    static bool auto_data = false;
    if (radio_device.command_state["echo"]) {
        radio_device.command_state["echo"] = false;
        radio_device.responce(radio_device.command_id_from_name["echo"], "1");
    }

    if (radio_device.command_state["reset"])
    {
        radio_device.command_state["reset"] = false;
        radio_device.responce(radio_device.command_id_from_name["reset"], "1");
        logger.logInfo("Reset request from radio");
        buzzer.sound(2000, 1000);
        CPU_RESTART;
    }

    if (radio_device.command_state["sensor_info"]) {
        radio_device.command_state["sensor_info"] = false;
        sensor_connection_packet.bno = bno.connected;
        sensor_connection_packet.bmp = bmp.connected;
        sensor_connection_packet.sd = logger.connected;
        radio_device.responce_data(radio_device.command_id_from_name["sensor_info"], (char *) &sensor_connection_packet, sizeof(sensor_connection_packet_t));
    }

    if (radio_device.command_state["bno_info"])
    {
        radio_device.command_state["bno_info"] = false;
        bno.getCalibration(&bno_calib_packet.system, &bno_calib_packet.gyro, &bno_calib_packet.accel, &bno_calib_packet.mag);
        radio_device.responce_data(radio_device.command_id_from_name["bno_info"], (char *) &bno_calib_packet, sizeof(bno_calib_packet_t));
    }

    if (radio_device.command_state["arm_bbb"]) {
        radio_device.command_state["arm_bbb"] = false;
        if (beagle_device.change_arm_state() == ARMED) {
            radio_device.responce(radio_device.command_id_from_name["arm_bbb"], "1");
        } else {
            radio_device.responce(radio_device.command_id_from_name["arm_bbb"], "0");
        }
    }

    if (radio_device.command_state["uefe"]) {
        radio_device.command_state["uefe"] = false;
        if (beagle_device.change_external_flight_estimation() == 0) {
            radio_device.responce(radio_device.command_id_from_name["uefe"], "0");
        }
        else {
            radio_device.responce(radio_device.command_id_from_name["uefe"], "1");
        }
    }

    if (radio_device.command_state["uefs"]) {
        radio_device.command_state["uefs"] = false;
        if (beagle_device.change_external_flight_state() == 0) {
            radio_device.responce(radio_device.command_id_from_name["uefs"], "0");
        }
        else {
            radio_device.responce(radio_device.command_id_from_name["uefs"], "1");
        }
    }

    if (radio_device.command_state["auto_data"]) {
        radio_device.command_state["auto_data"] = false;
        if (auto_data) {
            auto_data = false;
            radio_device.responce(radio_device.command_id_from_name["auto_data"], "0");
        } else {
            auto_data = true;
            radio_device.responce(radio_device.command_id_from_name["auto_data"], "1");
        }
    }

    if (auto_data) {
        if (rocket_broadcast_timer + 120 <= millis()) {
            radio_device.responce_data(radio_device.command_id_from_name["broadcast"], (char *) &realtime_data_packet, rt_data);
            rocket_broadcast_timer = millis();
        }
    }
}
void realtime_data() {}
#else
/**
 * @brief Function that checks radio requests and handles them.
 * Called in void loop()
 */
void radio_tasks()
{
    static bool auto_data = false;
    if (radio_device.command_state["echo"]) {
        radio_device.command_state["echo"] = false;
        radio_device.responce(radio_device.command_id_from_name["echo"]);
    }

    if (radio_device.command_state["reset"])
    {
        radio_device.command_state["reset"] = false;
        radio_device.responce(radio_device.command_id_from_name["reset"]);
        logger.logInfo("Reset request from radio");
        buzzer.sound(2000, 1000);
        CPU_RESTART;
    }

    if (radio_device.command_state["sensor_info"]) {
        radio_device.command_state["sensor_info"] = false;
        Serial2.print("%");
        Serial2.print(radio_device.command_id_from_name["sensor_info"]);
        Serial2.print(bno.connected);
        Serial2.print(",");
        Serial2.print(bmp.connected);
        Serial2.print(",");
        Serial2.print(logger.connected);
        Serial2.print(",");
        Serial2.print(gps.is_fixed());
        Serial2.print(";");
    }

    if (radio_device.command_state["bno_info"])
    {
        radio_device.command_state["bno_info"] = false;
        bno.getCalibration(&bno_calib_packet.system, &bno_calib_packet.gyro,
            &bno_calib_packet.accel, &bno_calib_packet.mag);
        Serial2.print("%");
        Serial2.print(radio_device.command_id_from_name["bno_info"]);
        Serial2.print(bno_calib_packet.system);
        Serial2.print(",");
        Serial2.print(bno_calib_packet.gyro);
        Serial2.print(",");
        Serial2.print(bno_calib_packet.accel);
        Serial2.print(",");
        Serial2.print(bno_calib_packet.mag);
        Serial2.print(";");
    }

    if (radio_device.command_state["arm_bbb"]) {
        radio_device.command_state["arm_bbb"] = false;
        if (beagle_device.change_arm_state() == ARMED) {
            radio_device.responce(radio_device.command_id_from_name["arm_bbb"], "1");
        } else {
            radio_device.responce(radio_device.command_id_from_name["arm_bbb"], "0");
        }
    }

    if (radio_device.command_state["uefe"]) {
        radio_device.command_state["uefe"] = false;
        if (beagle_device.change_external_flight_estimation() == 0) {
            radio_device.responce(radio_device.command_id_from_name["uefe"], "0");
        }
        else {
            radio_device.responce(radio_device.command_id_from_name["uefe"], "1");
        }
    }

    if (radio_device.command_state["uefs"]) {
        radio_device.command_state["uefs"] = false;
        if (beagle_device.change_external_flight_state() == 0) {
            radio_device.responce(radio_device.command_id_from_name["uefs"], "0");
        }
        else {
            radio_device.responce(radio_device.command_id_from_name["uefs"], "1");
        }
    }

    if (radio_device.command_state["auto_data"]) {
        radio_device.command_state["auto_data"] = false;
        if (auto_data) {
            auto_data = false;
            radio_device.responce(radio_device.command_id_from_name["auto_data"], "0");
        } else {
            auto_data = true;
            radio_device.responce(radio_device.command_id_from_name["auto_data"], "1");
        }
    }

    if (radio_device.command_state["acs_test"]) {
        radio_device.command_state["acs_test"] = false;
        if (beagle_device.set_start_pre_flight_checks() == 0) {
            radio_device.responce(radio_device.command_id_from_name["acs_test"], "0");
        }
        else {
            radio_device.responce(radio_device.command_id_from_name["acs_test"], "1");
        }
    }

    if (auto_data) {
        if (rocket_broadcast_timer + 120 <= millis()) {
            realtime_data();
            rocket_broadcast_timer = millis();
        }
    }
}

/**
 * @brief Function that generates realtime data message and transmit it
 * by radio (that is actually xbee device that is connected via Serial2 in
 * current project).
 * 
 * Implemented in lazy way, which means instead of using radio_device object
 * as transmitter it directly works with Serial. The reason for that is because
 * lack of experience and time to accomplish task before due.
 * It is planned to improve it and work as an object not with many functions.
 * 
 * Last update: 05/15/2021 by Kazybek Mizam
 */
void realtime_data() {
    String real_data = "";
    real_data += String(beagle_device.current_arm_state());
    real_data += ",";
    real_data += String(beagle_device.current_flight_state());
    real_data += ",";
    real_data += String(beagle_device.current_flight_state_received());
    real_data += ",";
    real_data += String(beagle_device.current_external_flight_state());
    real_data += ",";
    real_data += String(beagle_device.current_external_flight_estimation());
    real_data += ",";
    real_data += String(data.altitude);
    real_data += ",";
    real_data += String(data.projected_altitude);
    real_data += ",";
    real_data += String(data.apogee);
    real_data += ",";
    real_data += String(data.vertical_acceleration);
    real_data += ",";
    real_data += String(data.vertical_velocity);
    real_data += ",";
    real_data += String(data.latitude);
    real_data += ",";
    real_data += String(data.longitude);
    real_data += ",";
    real_data += String(data.temperature);
    
    Serial2.print("%");
    Serial2.print(radio_device.command_id_from_name["broadcast"]);
    Serial2.print(real_data);
    Serial2.print(";");
}
#endif

/**
 * @brief Function that checks for button supported events
 * and handles them.
 * There are currently 4 events that button object can catch:
 * 1. button.one_pressed() –– If button pressed
 * 2. button.double_pressed() –– If button double pressed
 * 3. button.bool hold_pressed() –– If button pressed for at least 3 seconds
 * and less than 10 seconds
 * 4. button.long_hold_pressed() –– If button pressed for more than 10 seconds
 * 
 * Those are work as flag, therefore multiple events can be caught before 
 * reaching this function. For example, if button hold pressed then it also
 * pressed once.
 * 
 * NOTE that one_pressed and double_pressed does not work properly in this project,
 * therefore they are not used. So if you want to use button library then please fix,
 * or look for newer versions.
 * 
 * Last update: 05/15/2021 by Kazybek Mizam
 */
void button_tasks()
{
    if (button.is_long_hold_pressed())
    {
        button.long_hold_reset();
        logger.logInfo("Reset request from button");
        buzzer.sound(2000, 1000);
        CPU_RESTART;
    }
}


void independent_event_test() {
    static bool run_once = true;
    static bool standby = true;
    static bool going_to_launch = true;
    static bool going_to_burnt_out = true;
    static bool reaching_apogee = true;
    static bool landing = true;

    if (run_once) {
        #ifdef inotesting
        Serial.println("Code started");
        #endif

        logger.logInfo("Flight code started");

        logger.logEvent("Standby mode");

        run_once = false;
    }
    
    if (standby) {
        #ifdef events
        Serial.println("Standby");
        #endif
        bool r = radio_device.command_state["arm_system"];
        bool b = button.is_hold_pressed();

        if (r || b) {
            if (b) {
                logger.logTrigger("Switching to detect launch mode from button request");
            }

            if (r) {
                logger.logTrigger("Switching to detect launch mode from radio request");
            }

            data.reset_apogee();

            beagle_device.change_arm_state();
            beagle_device.next_flight_state();
            radio_device.command_state["arm_system"] = false;
            radio_device.responce(radio_device.command_id_from_name["arm_system"], "1");

            rocket_flight_state.setup_timer(millis());

            String rvd = data.calibrate();
            logger.logCalibrate(data.ground_altitude, rvd);

            logger.logEvent("Waiting for launch");

            realtime_data();

            button.hold_reset();
            buzzer.multiple_async_sound(2000, 300, 2, 300);
            standby = false;
        }
    }
    else if (going_to_launch){
        #ifdef events
        Serial.println("Waiting for launch");
        #endif
        if (rocket_flight_state.if_launched_by_altitude(data.altitude, data.vertical_acceleration,
                data.current_altitude_fixtime)) {
            // radio_device.disable(); // Now it can be controlled during all flight

            logger.logEvent("Launched, waiting for burnt out");

            logger.logTrigger<double>("Vertical acceleration", data.vertical_acceleration);
            logger.logTrigger<double>("Altitude", data.altitude);
            logger.logInfo("Data begins in #" + String(data.iterator));

            beagle_device.next_flight_state();
            realtime_data();

            buzzer.multiple_async_sound(2000, 300, 2, 300);
            going_to_launch = false;
        }
    }
    else if (going_to_burnt_out)
    {
        #ifdef events
        Serial.println("Waiting for burnt out");
        #endif

        if(rocket_flight_state.if_burnt_out(data.altitude, data.vertical_velocity,
                data.vertical_acceleration, data.current_altitude_fixtime)) {
            logger.logEvent("Burnt out, waiting for apogee");
            data.prepare_for_apogee();

            logger.logTrigger<double>("Vertical velocity", data.vertical_velocity);
            logger.logTrigger<double>("Vertical acceleration", data.vertical_acceleration);

            beagle_device.next_flight_state();
            realtime_data();

            going_to_burnt_out = false;
        }
    }
    else if (reaching_apogee) {
        #ifdef events
        Serial.println("Waiting for apogee");
        #endif

        if (rocket_flight_state.if_reached_apogee(data.apogee, data.altitude,
                data.current_altitude_fixtime, data.apogee_time)) {
            radio_device.enable();

            logger.logEvent("Descending, preparing for land");
            data.prepare_for_land();

            logger.logTrigger<double>("Height difference", data.apogee - data.altitude);
            buzzer.multiple_async_sound(2000, 300, 2, 300);

            beagle_device.next_flight_state();
            realtime_data();

            reaching_apogee = false;
        }
    }
    else if (landing) {
        #ifdef events
        Serial.println("Landing");
        #endif

        if (rocket_flight_state.if_landed(data.altitude, &data.land_altitude,
                data.total_linear_acceleration, data.current_altitude_fixtime, &data.land_time)) {
            #ifdef events
            Serial.println("Landed!");
            #endif
            logger.logEvent("Landed");

            #ifdef inotesting
            Serial.println("Log end");
            Serial.println("Code finished, continuing logging of back up data");
            #endif

            logger.logTrigger<double>("Altitude difference: ", data.altitude - data.land_altitude);
            logger.logTrigger<double>("Total linear acceleration: ", data.total_linear_acceleration);
            logger.logInfo("Data ends in #" + String(data.iterator));

            beagle_device.change_arm_state();
            beagle_device.next_flight_state();
            realtime_data();

            buzzer.multiple_async_sound(2000, 300, 3, 100);

            logger.logInfo("Continuing logging of back up data");
            landing = false;
        }
    }
}
