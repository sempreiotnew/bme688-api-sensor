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

#define STABILIZATION_COUNTER   500
#define STABILIZATION_DELAY_MS  1000
#define NEW_GAS_MEAS (BME68X_GASM_VALID_MSK | BME68X_HEAT_STAB_MSK | BME68X_NEW_DATA_MSK)

Bme68x bme;
bme68xData data;


void setup(void)
{
    SPI.begin();
    Serial.begin(115200);
    
    while (!Serial)
        delay(10);
    
    setLeds();
    initializeSensor(bme);
    // setParallelModeHP354(bme);
    setForcedMode(bme);
    
    Serial.println("id,index,millis,gas_index,mes_index,temperature,pressure,humidity,gas_resistance,status");
    int counter = 0;
    while(counter < STABILIZATION_COUNTER){
        delayMicroseconds(bme.getMeasDur());

        if (bme.fetchData()) {
            bme.getData(data);
            // Serial.print(".");
            logSerial(data, data.gas_index, bme.getUniqueId(), 0.0f);
        }
        setColorRGB(0,0,255);//azul
        setForcedMode(bme);
        counter++;
    }
    
}


void loop() {

        setSleepMode(bme);
        setColorRGB(255,0,0);//vermelho
        // delay(STABILIZATION_DELAY_MS);
        delay(30000);
        
        setForcedMode(bme);
        // setParallelModeHP354(bme);
        delayMicroseconds(bme.getMeasDur());

        // Wait for the full heater duration + measurement time
        setColorRGB(0,255,0);//verde

        // Warm up: do 5-10 forced measurements
        for (int i=0; i < 10; i++) {
            
            setColorRGB(0,255,0);//verde
            delayMicroseconds(bme.getMeasDur());
            if (bme.fetchData()) bme.getData(data);
            setForcedMode(bme); // trigger next forced measurement
            logSerial(data, data.gas_index, bme.getUniqueId(), 0.0f);
            setColorRGB(0,0,0);//verde
            
        }

}
