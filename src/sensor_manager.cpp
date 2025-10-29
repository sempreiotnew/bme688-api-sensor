
#ifndef PIN_CS
#define PIN_CS SS
#endif

#include "bme68xLibrary.h"

void setForcedModeCalib(Bme68x& bme) {
    bme.setTPH(BME68X_OS_1X, BME68X_OS_1X, BME68X_OS_1X); // T,H,P oversampling
    bme.setHeaterProf(350, 100); // heater temp 300°C, 100ms
    bme.setOpMode(BME68X_FORCED_MODE);
}

// Forced mode: single measurement
void setForcedMode(Bme68x& bme) {
    bme.setTPH(BME68X_OS_1X, BME68X_OS_1X, BME68X_OS_1X); // T,H,P oversampling
    bme.setHeaterProf(350, 980); // heater temp 300°C, 100ms
    bme.setOpMode(BME68X_FORCED_MODE);
}

void setForcedModeParameters(Bme68x& bme, uint8_t osTemp, uint8_t osPres, uint8_t osHum) {
    bme.setTPH(osTemp, osPres, osHum); // T,H,P oversampling
    bme.setHeaterProf(350, 980); // heater temp 300°C, 100ms
    bme.setOpMode(BME68X_FORCED_MODE);
}

// Parallel mode: multiple steps heater profile
void setParallelMode(Bme68x& bme) {
    static uint16_t tempProf[] = {200, 250, 300, 350, 400};
    static uint16_t durProf[]  = {100, 100, 100, 100, 100};
    bme.setTPH(BME68X_OS_4X, BME68X_OS_2X, BME68X_OS_16X);
    bme.setHeaterProf(tempProf, durProf, 5); // 5-step heater profile
    bme.setOpMode(BME68X_PARALLEL_MODE);
}

// Parallel mode: multiple steps heater profile
void setParallelModeHP354(Bme68x& bme) {
    static uint16_t tempProf[] = {320, 100, 100, 100, 200, 200, 200, 320, 320, 320};
    // static uint16_t durProf[]  = {5, 2, 10, 30, 5, 5, 5, 5, 5, 5};
    static uint16_t durProf[]  = {700, 280, 1400, 4200, 700, 700, 700, 700, 700, 700};
    bme.setTPH(BME68X_OS_4X, BME68X_OS_2X, BME68X_OS_16X);
    bme.setHeaterProf(tempProf, durProf, 10); // 5-step heater profile
    bme.setOpMode(BME68X_PARALLEL_MODE);
}

void setSleepMode(Bme68x& bme){
    bme.setOpMode(BME68X_SLEEP_MODE);
}

void initializeSensor(Bme68x& bme){
    // Initialize sensor
    bme.begin(PIN_CS, SPI);
    

    if(bme.checkStatus())
    {
        if (bme.checkStatus() == BME68X_ERROR)
        {
            Serial.println("Sensor error:" + bme.statusString());
            return;
        }
        else if (bme.checkStatus() == BME68X_WARNING)
        {
            Serial.println("Sensor Warning:" + bme.statusString());
        }
    }

}