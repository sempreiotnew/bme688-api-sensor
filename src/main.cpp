/**
 * Copyright (C) 2021 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 * 
 */

#include "Arduino.h"
#include "bme68xLibrary.h"
#include <led_controller.h>
#include <log_serial.h>
#include <sensor_manager.h>

#define STABILIZATION_COUNTER   50
#define STABILIZATION_DELAY_MS  1000
#define SLEEP_MAX_READING 5
#define NEW_GAS_MEAS (BME68X_GASM_VALID_MSK | BME68X_HEAT_STAB_MSK | BME68X_NEW_DATA_MSK)

struct DataReading {
    float baseline;
    float gas_resistance;
    float temperature;
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
    setForcedModeCalib(bme);
    
    Serial.println("id,index,millis,gas_index,mes_index,temperature,pressure,humidity,gas_resistance,status");
    int counter = 0;
    while(counter < STABILIZATION_COUNTER){
        
        setColorRGB(0,0,255);//azul
        delayMicroseconds(bme.getMeasDur());
        setColorRGB(0,0,0);

        if (bme.fetchData()) {
            bme.getData(data);
            logSerial(data, data.gas_index, bme.getUniqueId(), 100);
        }
        
        setForcedModeCalib(bme);
        counter++;
    }

    // dataReading.baseline = data.gas_resistance;
    
}

void getSensorDataSleep(){

    dataReading.baseline = dataReading.gas_resistance;

    for (int i=0; i < SLEEP_MAX_READING; i++) {
        setForcedMode(bme);
        setColorRGB(0,0,0);//off
        delayMicroseconds(bme.getMeasDur());
        setColorRGB(0,255,0);//verde
        if (bme.fetchData()) bme.getData(data);
        // setForcedMode(bme); // trigger next forced measurement
        logSerial(data, data.gas_index, bme.getUniqueId(), 99);
        // setColorRGB(0,0,0);
        
        if(data.status == NEW_GAS_MEAS){
            dataReading.gas_resistance = data.gas_resistance;
        }
        
    }

    setSleepMode(bme);
    setColorRGB(255,0,0);//vermelho
    // delay(STABILIZATION_DELAY_MS);
    delay(10000);
    setColorRGB(0,0,0);//off

}

void getSensorDataSenquential(){
 uint8_t counter = 0;
 bool keepReading = true;
 uint32_t thresholdCounter = 10;


  while (keepReading){
    uint8_t nFieldsLeft = 0;
    bme.setOpMode(BME68X_FORCED_MODE);

    if (bme.fetchData()) {
        setColorRGB(0,255,0);//verde
        do {
            nFieldsLeft = bme.getData(data);
            if (data.status == NEW_GAS_MEAS) {
                logSerial(data, data.gas_index,
                          bme.getUniqueId(), 1);
            }
        } while (nFieldsLeft);
    }

    float drop = getDropPercentage(data.gas_resistance, dataReading.baseline);

    Serial.printf("-Drop: %.2f%% R %.2f  - B %.2f \n" , drop, data.gas_resistance, dataReading.baseline);

    if(drop <= 5){
        thresholdCounter = thresholdCounter - 1;
    }

    if(thresholdCounter <= 0){
        keepReading = false;
        // dataReading.baseline = data.gas_resistance;
    }

    setColorRGB(0,0,0);//off
  }  
  
    
}

float getDropPercentage(float R, float baseline){
    if (R <= 0 || baseline <= 0) return 0.0f; // safety check

    float dropPercent = 0;
    dropPercent = (baseline - R) / baseline ;

    if(dropPercent > 0) {
        return dropPercent * 100;
    } else {
        return 0.0f;
    }
    
}


void loop() {
    getSensorDataSleep();

    float drop = getDropPercentage(dataReading.gas_resistance, dataReading.baseline);
    Serial.printf("-Drop: %.2f%% R %.2f  - B %.2f \n" , drop, dataReading.gas_resistance, dataReading.baseline);
    if(drop >= 10.f){
        getSensorDataSenquential();
    } else {
        dataReading.baseline = data.gas_resistance;
    }
}
