/*   Connections
   ===========
   Connect SCK to analog 5
   Connect SDI to analog 4
   Connect VIN to 3.3-5V DC
   Connect GROUND to common ground

   // Для подключения по шине I2C

   Adafruit_BMP280 bmp;

   // Для подключения по аппаратному SPI (указываем только номер пина CS)

   #define PIN_CS 10

   Adafruit_BMP280 bmp(PIN_CS);



   // Для подключения по программному SPI (указываем все пины интерфейса)

   #define PIN_SCK 13

   #define PIN_MISO 12

   #define PIN_MOSI 11

   #define PIN_CS   10

   Adafruit_BMP280 bmp(PIN_CS, PIN_MOSI, PIN_MISO, PIN_SCK);

   // Дополнительные настройки BMP280

   bmp.setSampling(Adafruit_BMP280::MODE_NORMAL, // Режим работы

                  Adafruit_BMP280::SAMPLING_X2, // Точность изм. температуры

                   Adafruit_BMP280::SAMPLING_X16, // Точность изм. давления

                   Adafruit_BMP280::FILTER_X16, // Уровень фильтрации

                   Adafruit_BMP280::STANDBY_MS_500); // Период просыпания, мСек

*/

/**
 * Library of bmp280.h
 * 
 * Developed for AURA Embedded System 2020
 * 
 * The library that works with BMP280 sensor
 * @1kzpro && @vladimireganov
 * @version 09/11/2020
 * 
 * @param BMP280 Class that controls BMP280 sensor
*/

#include <Adafruit_BMP280.h>

class BMP280
{
private:
    Adafruit_BMP280 bmp;
    unsigned long bmp_time;

public:
    const int BMP280_NORMAL_DELAY_MS = 39;
    const bool connected;

    BMP280() {
        bmp_time = millis() + BMP280_NORMAL_DELAY_MS;
    }

    void activate()
    {
        // Initializes bmp with pre-defined settings
        if (!bmp.begin())
        {
            Serial.println("Could not find a valid BMP280 sensor, check wiring!");
            connected = false;
        }
        else
        {
            bmp.setSampling(Adafruit_BMP280::MODE_NORMAL, // Режим работы

                            Adafruit_BMP280::SAMPLING_X2, // Точность изм. температуры

                            Adafruit_BMP280::SAMPLING_X16, // Точность изм. давления

                            Adafruit_BMP280::FILTER_OFF, // Уровень фильтрации

                            Adafruit_BMP280::STANDBY_MS_1);
            Serial.println("BMP is working!");
            connected = true;
        }
    }

    bool ready(double board_time) {
        if (bmp_time < board_time) {
            bmp_time = board_time + BMP280_NORMAL_DELAY_MS;
            return true;
        }
        return false;
    }

    bool is_working() {
        return connected;
    }

    float altitude() {
        float altitude_from_tp = bmp.readAltitude();
        return altitude_from_tp;
    }

    float pressure() {
        return bmp.readPressure();
    }
    float temperature() {
        return bmp.readTemperature();
    }
};