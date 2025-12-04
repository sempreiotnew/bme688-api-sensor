/**
 * Copyright (C) 2021 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 * 
 */

#include "Arduino.h"
#include <cmath>
#include "bme68xLibrary.h"
#include <led_controller.h>
#include <log_serial.h>
#include <sensor_manager.h>

#define STABILIZATION_COUNTER   50
#define STABILIZATION_DELAY_MS  1000
#define SLEEP_MAX_READING 1
#define NEW_GAS_MEAS (BME68X_GASM_VALID_MSK | BME68X_HEAT_STAB_MSK | BME68X_NEW_DATA_MSK)


struct DataReading {
    float baseline;
    float gas_resistance;
    float temperature;
    float pressure;
    float humidity;
};

Bme68x bme;
bme68xData data;
DataReading dataReading;

float getDropPercentage(float R, float baseline);

void setup(void)
{
    SPI.begin();
    Serial.begin(115200);
    
    while (!Serial)
        delay(10);
    
    setLeds();
    initializeSensor(bme);
    // setParallelModeHP354(bme);
    setForcedModeParameters(bme, BME68X_OS_1X, BME68X_OS_1X, BME68X_OS_1X);
    
    Serial.println("id,index,millis,gas_index,mes_index,temperature,pressure,humidity,gas_resistance,status");
    
    // int counter = 0;
    // while(counter < STABILIZATION_COUNTER){
        
    //     setColorRGB(0,0,255);//azul
    //     delayMicroseconds(bme.getMeasDur());
    //     setColorRGB(0,0,0);

    //     if (bme.fetchData()) {
    //         bme.getData(data);
    //         logSerial(data, data.gas_index, bme.getUniqueId(), 100);
    //     }
        
    //     setForcedModeCalib(bme);
    //     counter++;
    // }
    
    // if(SLEEP_MAX_READING >= 10){
    //     dataReading.baseline = data.gas_resistance;
    // } 

    
    
    
}


float calculateSeaLevelPressure(float stationPressure_Pa,
                                float temperature_C,
                                float altitude_m)
{
    float P_hPa = stationPressure_Pa / 100.0f;
    float T_k   = temperature_C + 273.15f;

    const float exponent = 5.255877f;          // g/(L·R) ≈ 5.255877

    // Correct formula: temperature decreases with altitude
    // float correction = powf( T_k / (T_k - 0.0065f * altitude_m), exponent );
    // or equivalently:
     float correction = powf(1.0f - 0.0065f * altitude_m / T_k, -exponent);

    return P_hPa * correction;
}


void getSensorDataSleep(){
    setSleepMode(bme);
    setColorRGB(0,0,0);//off
    delay(30000);
    

    for (int i=0; i < SLEEP_MAX_READING; i++) {
        setForcedModeParameters(bme, BME68X_OS_1X, BME68X_OS_1X, BME68X_OS_1X);
        setColorRGB(0,255,0);//verde
        delayMicroseconds(bme.getMeasDur());
        setColorRGB(0,0,0);//off
        if (bme.fetchData()) bme.getData(data);
        // setForcedMode(bme); // trigger next forced measurement
        logSerial(data, data.gas_index, bme.getUniqueId(), 99);
        // setColorRGB(0,0,0);
        
        dataReading.temperature = data.temperature;
        dataReading.pressure = data.pressure;
        dataReading.humidity = data.humidity;
        Serial.printf("--Temperature %.2f\n", data.temperature);
        Serial.printf("--Pressure %.2f\n", data.pressure);
        Serial.printf("--Humidity %.2f\n", data.humidity);

        float seaLevel = calculateSeaLevelPressure(data.pressure, data.temperature, 760.0f);
        Serial.printf("--Sea Level %.2f\n", seaLevel);


    }

    

}



void loop() {
    getSensorDataSleep();
}
