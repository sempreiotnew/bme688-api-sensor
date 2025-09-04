#include "Arduino.h"
#include "bme68xLibrary.h"
#include <led_controller.h>
#include <log_serial.h>
#include <sensor_manager.h>

#define STABILIZATION_COUNTER   200
#define SLEEP_DELAY_MS  10000
#define MEASUREMENT_OVERSAMPLING 20 //Total measurements in forced mode 
#define NEW_GAS_MEAS (BME68X_GASM_VALID_MSK | BME68X_HEAT_STAB_MSK | BME68X_NEW_DATA_MSK)

Bme68x bme;
bme68xData data;
float baseline = 0.0f;
bool isKeepChecking = false;

void stabilizationLED() { setColorRGB(0,0,255); } 
void sleepLED() { setColorRGB(255,0,0); } 
void measurementLED() { setColorRGB(0,255,0); } 
void offLED() { setColorRGB(0,0,0); } 

void setup(void)
{
    SPI.begin();
    Serial.begin(115200);
    
    while (!Serial)
        delay(10);
    
    setLeds();
    initializeSensor(bme);
    setForcedMode(bme); // initial forced measurement
    
    Serial.println("id,index,millis,gas_index,mes_index,temperature,pressure,humidity,gas_resistance,status");

    // Stabilization loop
    for (int counter = 0; counter < STABILIZATION_COUNTER; counter++){
        stabilizationLED();
        delayMicroseconds(bme.getMeasDur()); // wait for measurement
        if (bme.fetchData()) {
            bme.getData(data);
            logSerial(data, data.gas_index, bme.getUniqueId(), 0.0f);
        }
        offLED();
        setForcedMode(bme); // trigger next forced measurement
    }
}

void parallelMode(){
    // Parallel measurements
    setParallelModeHP354(bme); // set parallel mode once
    uint16_t measDur = bme.getMeasDur(); // get full measurement duration

    delayMicroseconds(measDur);
    for (int i = 0; i < 100; i++) {
        delay(140); // wait for measurement to complete
        uint8_t nFieldsLeft = 0;

        if (bme.fetchData()) {
            do {
                nFieldsLeft = bme.getData(data);
                if (data.status == NEW_GAS_MEAS) {
                    Serial.print(".");
                    logSerial(data, data.gas_index, bme.getUniqueId(), 0.0f);
                }
            } while (nFieldsLeft);
            
        }
        measurementLED();
    }
    delay(SLEEP_DELAY_MS);
}
float getDropPercentage(float R){
    if (R <= 0 || baseline <= 0) return 0.0f; // safety check

    float dropPercent = 0;
    dropPercent = (baseline - R) / baseline ;

    if(dropPercent > 0) {
        return dropPercent * 100;
    } else {
        return 0.0f;
    }
    
}

void setLastGasResistance(float R){
    baseline = R;
}


void loop() {
    // Sleep period
    setSleepMode(bme);
    delay(SLEEP_DELAY_MS);
    
    float temperature = 0;
    // Forced measurements
    for (int i = 0; i < MEASUREMENT_OVERSAMPLING; i++) {
        
        offLED();
        setForcedMode(bme); // trigger measurement
        delayMicroseconds(bme.getMeasDur()); // wait for measurement to finish
        if (bme.fetchData()) bme.getData(data);
        measurementLED();
        if(i == 1){ 
            temperature = data.temperature;
        }
        logSerial(data, data.gas_index, bme.getUniqueId(), 0.0f);
    }

    offLED();
    float percentage = getDropPercentage(data.gas_resistance);
    

    if(percentage > 10.0f){
        float percent = percentage;
        bool isCigarette = false;

        do {
            for (int i = 0; i < 20; i++) {
                isCigarette ? sleepLED() : measurementLED();
                setForcedModeHeat(bme); // trigger measurement
                delayMicroseconds(bme.getMeasDur()); // wait for measurement to finish
                
                if (bme.fetchData()) bme.getData(data);
                offLED();
                
                if(i == 1){ 
                    temperature = data.temperature;
                }
                logSerial(data, data.gas_index, bme.getUniqueId(), 0.0f);
                delay(1000);
            }

            percent = getDropPercentage(data.gas_resistance);
            Serial.println(percent);
            Serial.println(baseline);

            if(percent > 15.0f){
                Serial.println("CIGARRO");
                isCigarette = true;
            } else {
                isCigarette = false;
                setLastGasResistance(data.gas_resistance);        
            }
        } while(percent > 10.0f);
        offLED();

    } else {
        setLastGasResistance(data.gas_resistance);
        Serial.printf("Drop: %.2f %%\t\n", percentage);
        Serial.printf("Rounded: %2.f %%\t\n", percentage);
        Serial.printf("Baseline: %.2f \n", baseline);
        Serial.printf("Temperature: %.2f C\n", temperature);
    }
    
}


