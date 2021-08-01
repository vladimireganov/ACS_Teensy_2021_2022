void async_tasks()
    {
        button->check();
        buzzer->check();
        radio->check();

        // Async tasks
        button_tasks();
        radio_tasks();
    }
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
        Serial1.print(bno->is_working());
        Serial1.print(',');
        Serial1.print(bmp->is_working());
        Serial1.print(',');
        Serial1.print(sds->is_working());
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