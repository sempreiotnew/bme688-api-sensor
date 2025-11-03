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
#define SLEEP_MAX_READING 10
#define NEW_GAS_MEAS (BME68X_GASM_VALID_MSK | BME68X_HEAT_STAB_MSK | BME68X_NEW_DATA_MSK)

#define WARNING_THRESHOLD 15
#define WARNING_THRESHOLD_MIN 20
#define WARNING_THRESHOLD_MAX 50

#define ALARM_THRESHOLD 10
#define ALARM_THRESHOLD_MIN 20
#define ALARM_THRESHOLD_MAX 100

#define RECOVERY_THRESHOLD_MEASUREMENTS 20
#define RECOVERY_THESHOLD_DROP_PERCENTAGE 10.f

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
    
    if(SLEEP_MAX_READING >= 10){
        dataReading.baseline = data.gas_resistance;
    } 
    
    
}

void getSensorDataSleep(){
    setSleepMode(bme);
    setColorRGB(0,0,0);//off
    delay(30000);
    

    // dataReading.baseline = dataReading.gas_resistance;

    for (int i=0; i < SLEEP_MAX_READING; i++) {
        setForcedMode(bme);
        setColorRGB(0,255,0);//verde
        delayMicroseconds(bme.getMeasDur());
        setColorRGB(0,0,0);//off
        if (bme.fetchData()) bme.getData(data);
        // setForcedMode(bme); // trigger next forced measurement
        logSerial(data, data.gas_index, bme.getUniqueId(), 99);
        // setColorRGB(0,0,0);
        
        if(data.status == NEW_GAS_MEAS){
            dataReading.gas_resistance = data.gas_resistance;
        }

        if(i == 1){
            dataReading.temperature = data.temperature;
            dataReading.pressure = data.pressure;
            dataReading.humidity = data.humidity;
            Serial.printf("--Temperature %.2f\n", data.temperature);
            Serial.printf("--Pressure %.2f\n", data.pressure);
            Serial.printf("--Humidity %.2f\n", data.humidity);
        }
        
    }

    

}

void getSensorDataSenquential(){
 uint8_t counter = 0;
 bool keepReading = true;
 uint32_t thresholdCounter = 0;
 bool isAlarm = false;
 uint32_t warningCounter = 0;
 uint32_t alarmCounter = 0;

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
            setColorRGB(0,0,0);//off
        } while (nFieldsLeft);
    }

    float drop = getDropPercentage(data.gas_resistance, dataReading.baseline);

    if (isAlarm){
        setColorRGB(255,0,0);
    }

    
    if(drop <= RECOVERY_THESHOLD_DROP_PERCENTAGE){
        thresholdCounter = thresholdCounter + 1;
    } else {
        thresholdCounter = 0;
    }

    if(thresholdCounter >= RECOVERY_THRESHOLD_MEASUREMENTS){
        keepReading = false;
        dataReading.baseline = data.gas_resistance;
        thresholdCounter = 0;
        warningCounter = 0;
        alarmCounter= 0;
        isAlarm = false;
    } 

    Serial.printf("-Drop: %.2f%% R %.2f - B %.2f Thresh_Recovery %d/%d \n" , drop, data.gas_resistance, dataReading.baseline, thresholdCounter, RECOVERY_THRESHOLD_MEASUREMENTS);
    
    if (drop >= ALARM_THRESHOLD_MIN && drop <= ALARM_THRESHOLD_MAX){
        alarmCounter = alarmCounter + 1;    
        if(alarmCounter > ALARM_THRESHOLD){
            setColorRGB(255,0,0);//vermelho
            isAlarm = true;
        }
    }

    
    
    // if (drop >= WARNING_THRESHOLD_MIN && drop <= WARNING_THRESHOLD_MAX){
    //     warningCounter = warningCounter + 1;    
    //     if(warningCounter > WARNING_THRESHOLD){
    //         setColorRGB(255,255,0);//amarelo
    //     }
    // } else {
    //     if (drop >= ALARM_THRESHOLD_MIN && drop <= ALARM_THRESHOLD_MAX){
    //         alarmCounter = alarmCounter + 1;    
    //         if(alarmCounter > ALARM_THRESHOLD){
    //             setColorRGB(255,0,0);//vermelho
    //         }
    //     } 
    // }


  }  
  
    
}

float getDropPercentage(float R, float baseline) {
    if (R <= 0 || baseline <= 0) return 0.0f;

    // promote to double for accurate math
    double ratio = 1.0 - (double(R) / double(baseline));
    double drop = fmax(0.0, ratio * 100.0);

    return (float)drop; // return float if you must
}

// float getDropPercentage(float R, float baseline){
//     if (R <= 0 || baseline <= 0) return 0.0f; // safety check

//     float dropPercent = 0;
//     dropPercent = (baseline - R) / baseline ;

//     if(dropPercent > 0) {
//         return dropPercent * 100;
//     } else {
//         return 0.0f;
//     }
    
// }

void loop() {
    getSensorDataSleep();

    float drop = getDropPercentage(dataReading.gas_resistance, dataReading.baseline);
    Serial.printf("-Drop: %.2f%% R %.2f  - B %.2f \n" , drop, dataReading.gas_resistance, dataReading.baseline);
    if(drop >= 20.f){
        getSensorDataSenquential();
    } else {
        dataReading.baseline = data.gas_resistance;
    }
}
