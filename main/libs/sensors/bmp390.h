/**
 * Library of bmp390.h
 * 
 * Developed for AURA Embedded System 2021
 * 
 * The library that works with BMP390 sensor
 * @1kzpro
 * @version 01/12/2021
 * 
 * @param BMP90 Class that controls BMP390 sensor
*/
#include <Adafruit_BMP3XX.h>

class BMP390
{
private:
    Adafruit_BMP3XX bmp;
    unsigned long bmp_time;
public:
    const int BMP3XX_NORMAL_DELAY_MS = 40;
    bool connected = false;
    void activate()
    {
        // Initializes bmp with pre-defined settings
        // bmp.begin_SPI(9)
        if (!bmp.begin_SPI(9))
        {
            #ifdef inotesting
            Serial.println("Could not find a valid BMP390 sensor, check wiring!");
            #endif
            connected = false;
        }
        else
        {
            bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
            bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
            bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
            bmp.setOutputDataRate(BMP3_ODR_200_HZ);
            #ifdef inotesting
            Serial.println("BMP is working!");
            #endif
            connected = true;
        }
    }

    bool ready(double board_time) {
        if (bmp_time < board_time) {
            bmp_time = board_time + BMP3XX_NORMAL_DELAY_MS;
            if (bmp.performReading()) {
                return true;
            } else {
                Serial.println("Perform reading failed!");
            }
        }
        
        return false;
    }

    float pressure() {return bmp.pressure;}
    float temperature() {return bmp.temperature;}
};