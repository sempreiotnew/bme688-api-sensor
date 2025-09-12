
#ifndef PIN_CS
#define PIN_CS SS
#endif

#include "bme68xLibrary.h"

// Forced mode: single measurement
void setForcedMode(Bme68x& bme) {
    bme.setTPH(BME68X_OS_4X, BME68X_OS_2X, BME68X_OS_16X); // T,H,P oversampling
    bme.setHeaterProf(300, 100); // heater temp 300°C, 100ms
    bme.setOpMode(BME68X_FORCED_MODE);
}

void setForcedModeHeat(Bme68x& bme) {
    bme.setTPH(BME68X_OS_4X, BME68X_OS_2X, BME68X_OS_16X); // T,H,P oversampling
    bme.setHeaterProf(150, 400); // heater temp 300°C, 100ms
    bme.setOpMode(BME68X_FORCED_MODE);
}

void setForcedModeAlcohol(Bme68x& bme) {
    bme.setTPH(BME68X_OS_4X, BME68X_OS_2X, BME68X_OS_16X); // T,H,P oversampling
    bme.setHeaterProf(150, 400); // heater temp 300°C, 100ms
    bme.setOpMode(BME68X_FORCED_MODE);
}

void setForcedModeTemp(Bme68x &bme, uint16_t heaterTemp) {
    bme68xHeatrConf heatrConf;
    heatrConf.enable = BME68X_ENABLE;
    heatrConf.heatr_temp = heaterTemp;   // target heater temperature (°C)
    heatrConf.heatr_dur = 100;           // ms, you can adjust
    heatrConf.shared_heatr_dur = 0;

    bme.setHeaterProf(heaterTemp, heatrConf.heatr_dur); // set heater temperature and duration
    bme.setOpMode(BME68X_FORCED_MODE); // force single measurement
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
    static uint16_t tempProf[] = {320, 250, 300, 200, 200, 200, 200, 320, 320, 320};
    static uint16_t durProf[]  = {5, 2, 10, 30, 5, 5, 5, 5, 5, 5};
    // static uint16_t durProf[]  = {700, 280, 1400, 4200, 700, 700, 700, 700, 700, 700};
    bme.setTPH(BME68X_OS_4X, BME68X_OS_2X, BME68X_OS_16X);
    uint16_t sharedHeatrDur = 140 - (bme.getMeasDur(BME68X_PARALLEL_MODE) / 1000);
    bme.setHeaterProf(tempProf, durProf, sharedHeatrDur, 10); // 5-step heater profile
    bme.setOpMode(BME68X_PARALLEL_MODE);
}

// Parallel mode: multiple steps heater profile
void setParallelModeCigarette(Bme68x& bme) {
    //step 0 and 4 is good for total alcohol
    // static uint16_t tempProf[] = {150, 320, 200, 200, 150, 150};
    // static uint16_t durProf[]  = {5, 10, 10, 10, 5, 10};
    //step 2 and 4 is good for total cigarette
    // static uint16_t tempProf[] = {150, 320, 100, 300, 150, 150};
    // static uint16_t durProf[]  = {5, 10, 10, 10, 5, 10};

    //this works fine for both
    // static uint16_t tempProf[] = {150, 320, 100, 300, 150, 150};
    // static uint16_t durProf[]  = {30, 10, 10, 10, 10, 10};
    static uint16_t tempProf[] = {200, 320, 100, 300, 200, 150};
    static uint16_t durProf[]  = {30, 5, 5, 5, 5, 5};
    bme.setTPH(BME68X_OS_4X, BME68X_OS_2X, BME68X_OS_16X);
    uint16_t sharedHeatrDur = 140 - (bme.getMeasDur(BME68X_PARALLEL_MODE) / 1000);
    bme.setHeaterProf(tempProf, durProf, sharedHeatrDur, 5); // 5-step heater profile
    
    bme.setOpMode(BME68X_PARALLEL_MODE);
}



// Parallel mode: Bosch HP501 heater profile (~26s cycle)
void setParallelModeHP501(Bme68x& bme) {
    // Bosch official HP501 profile
    static uint16_t tempProf[] = {210, 265, 265, 320, 320, 265, 210, 155, 100, 155};
    // Durations converted to ms (AI Studio × 100ms units)
    static uint16_t durProf[]  = {2400, 200, 2200, 200, 2200, 2400, 2400, 2400, 2400, 2400};
    // Set oversampling (same as AI Studio default)
    bme.setTPH(BME68X_OS_4X, BME68X_OS_2X, BME68X_OS_16X);
    // Shared heater duration not used (leave 0)
    uint16_t sharedHeatrDur = 0;
    // Apply 10-step profile
    bme.setHeaterProf(tempProf, durProf, sharedHeatrDur, 10);
    // Switch to parallel mode
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