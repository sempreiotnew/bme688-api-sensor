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


#ifndef PIN_CS
#define PIN_CS SS
#endif

Bme68x bme;

void setup(void)
{
    SPI.begin();
    Serial.begin(115200);
    
    while (!Serial)
        delay(10);
        
    // Initialize sensor
    bme.begin(PIN_CS, SPI);
    setLeds();

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
    
    // Set default configuration for temperature, pressure, humidity
    bme.setTPH();
    
    
    bme.setHeaterProf(300, 100);

    // CSV header
    Serial.println("id,index,millis,gas_index,mes_index,temperature,pressure,humidity,gas_resistance,status");
    // Serial.println(bme.getMeasDur() * 1000);
    // Serial.println(bme.getMeasDur());
}

unsigned long lastMeasurement = 0;
const unsigned long interval = 1000; // 10 s

void loop() {
    // if (millis() - lastMeasurement >= interval) {
    //     lastMeasurement = millis();

        bme.setOpMode(BME68X_SLEEP_MODE);
        setColorRGB(255,0,0);//vermelho
        // delay(30000);
        
        delayMicroseconds(bme.getMeasDur());
        
        // delay(4000);
        // Trigger Forced Mode
        bme.setOpMode(BME68X_FORCED_MODE);

        // Wait for the full heater duration + measurement time
        setColorRGB(0,255,0);//verde
        

        // Read data
        bme68xData data;
        if (bme.fetchData()) {
            bme.getData(data);
            
            logSerial(data, data.gas_index, bme.getUniqueId(), 0.0f);
        }

        
        // delay(4000);
        
    // }    

        

        // bme.setOpMode(BME68X_SLEEP_MODE);
        // Serial.printf("Mode: %d\n", bme.getOpMode());
        // if(bme.getOpMode() == 0){
        //     setColorRGB(255,0,0);//vermelho
        // }
        
}
