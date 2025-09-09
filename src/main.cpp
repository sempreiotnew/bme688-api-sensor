#include "Arduino.h"
#include "bme68xLibrary.h"
#include <led_controller.h>
#include <log_serial.h>
#include <sensor_manager.h>

#define STABILIZATION_COUNTER   100
#define SLEEP_DELAY_MS  10000
#define MEASUREMENT_OVERSAMPLING 20 //Total measurements in forced mode 
#define NEW_GAS_MEAS (BME68X_GASM_VALID_MSK | BME68X_HEAT_STAB_MSK | BME68X_NEW_DATA_MSK)

Bme68x bme;
bme68xData data;
float baseline = 0.0f;
void stabilizationLED() { setColorRGB(0,0,255); } 
void sleepLED() { setColorRGB(255,0,0); } 
void measurementLED() { setColorRGB(0,255,0); } 
void offLED() { setColorRGB(0,0,0); } 

void computeDropSpeed(float arr[], int len, float dt) {
    if (len < 2) return;

    float maxDrop = 0;
    int dropStart = 0;
    int dropEnd = 0;

    // --- Find max drop over buffer ---
    for (int i = 0; i < len - 1; i++) {
        for (int j = i + 1; j < len; j++) {
            float diff = arr[i] - arr[j];
            if (diff > maxDrop) {
                maxDrop = diff;
                dropStart = i;
                dropEnd = j;
            }
        }
    }

    float dropSpeed = maxDrop / ((dropEnd - dropStart) * dt);

    // --- Recovery after minimum ---
    float recovery = 0;
    float minVal = arr[dropEnd];
    int minIdx = dropEnd;
    for (int k = dropEnd + 1; k < len; k++) {
        float diff = arr[k] - minVal;
        if (diff > recovery) {
            recovery = diff;
        }
    }
    float recoverySpeed = recovery / ((len - 1 - minIdx) * dt);

    Serial.printf("-Drop=%.1f Ω/s, Recovery=%.1f Ω/s\n", dropSpeed, recoverySpeed);
}

float previousGas = 0;
unsigned long previousMillis = 0;

void processReading(unsigned long currentMillis, float currentGas) {
    if (previousMillis != 0) {
        float deltaTime = (currentMillis - previousMillis) / 1000.0f; // convert ms to s
        if (deltaTime <= 0) deltaTime = 0.001; // avoid division by zero
        float velocity = (currentGas - previousGas) / deltaTime; // Ω/s

        if (velocity < 0) {
            Serial.print("-Drop = ");
            Serial.print(-velocity);
            Serial.println(" Ω/s");
        } else {
            Serial.print("-Recovery = ");
            Serial.print(velocity);
            Serial.println(" Ω/s");
        }
    }

    previousGas = currentGas;
    previousMillis = currentMillis;
}

// void calculatePercentVelocity(unsigned long currentMillis, float currentGas, float &previousGas, float gasIndex) {
//     // static float previousGas = 0;
//     static unsigned long previousMillis = 0;

//     Serial.printf("-currentGas %.2f", currentGas);
//     Serial.printf("-previousGas %.2f", previousGas);

//     if (previousMillis != 0 && previousGas > 0) {
//         float deltaTime = (currentMillis - previousMillis) / 1000.0f; // convert ms to s
//         if (deltaTime <= 0) deltaTime = 0.001; // avoid division by zero

//         // percentage change per second
//         float velocityPercent = ((currentGas - previousGas) / previousGas) / deltaTime * 100.0f;

//         // Drop
//         Serial.printf("-Drop %d = ", int(gasIndex));
//         Serial.print((velocityPercent < 0) ? -velocityPercent : 0.0f);
//         Serial.println(" %/s");

//         // Recovery
//         Serial.printf("-Recovery %d = ", int(gasIndex));
//         Serial.print((velocityPercent > 0) ? velocityPercent : 0.0f);
//         Serial.println(" %/s");

//         Serial.println("-");
//     }

//     previousGas = currentGas;
//     previousMillis = currentMillis;
// }

void calculatePercentVelocity(unsigned long currentMillis, float currentGas, float previousGas, float gasIndex) {
    static unsigned long previousMillis = 0;
    Serial.printf("-currentGas %.2f", currentGas); Serial.printf("-previousGas %.2f", previousGas);

    if (previousMillis != 0 && previousGas > 0) {
        float deltaTime = (currentMillis - previousMillis) / 1000.0f;
        if (deltaTime <= 0) deltaTime = 0.001;

        float percentChange = ((currentGas - previousGas) / previousGas) * 100.0f;
        float velocityPercent = percentChange / deltaTime;

        Serial.printf("\n-GasIdx %d Δ%%=%.2f over %.2fs\n", int(gasIndex), percentChange, deltaTime);
        Serial.printf("-Drop %d = %.2f %%/s\n", int(gasIndex), (velocityPercent < 0) ? -velocityPercent : 0.0f);
        Serial.printf("-Recovery %d = %.2f %%/s\n", int(gasIndex), (velocityPercent > 0) ? velocityPercent : 0.0f);
    }

    previousMillis = currentMillis;  // only update time here
}


float getDrop(unsigned long currentMillis, float currentGas, float gasIndex){
    static float previousGas = 0;
    static unsigned long previousMillis = 0;

    Serial.printf("-currentGas %.2f", currentGas); Serial.printf("-previousGas %.2f \n", previousGas);
    if (previousMillis != 0 && previousGas > 0) {
        float deltaTime = (currentMillis - previousMillis) / 1000.0f; // convert ms to s
        if (deltaTime <= 0) deltaTime = 0.001; // avoid division by zero

        // percentage change per second
        float velocityPercent = ((currentGas - previousGas) / previousGas) / deltaTime * 100.0f;

        // Drop
        float result = (velocityPercent < 0) ? -velocityPercent : 0.0f;
        Serial.printf("-Drop %d = ", int(gasIndex));
        Serial.print(result);
        Serial.println(" %/s");

    }

    previousGas = currentGas;
    previousMillis = currentMillis;
}

float getRecover(unsigned long currentMillis, float currentGas, float gasIndex){
    static float previousGas = 0;
    static unsigned long previousMillis = 0;

    if (previousMillis != 0 && previousGas > 0) {
        float deltaTime = (currentMillis - previousMillis) / 1000.0f; // convert ms to s
        if (deltaTime <= 0) deltaTime = 0.001; // avoid division by zero

        // percentage change per second
        float velocityPercent = ((currentGas - previousGas) / previousGas) / deltaTime * 100.0f;

        float result = (velocityPercent > 0) ? velocityPercent : 0.0f;
        // Recovery
        Serial.printf("-Recovery %d = ", int(gasIndex));
        Serial.print(result);
        Serial.println(" %/s");

        Serial.println("-");
    }

    previousGas = currentGas;
    previousMillis = currentMillis;    
}
// --- New: simple data structure for collected readings ---
struct Reading {
    unsigned long t;       // millis timestamp
    float gas_resistance;  
    float temperature;
    uint8_t gas_index;
};

#define PARALLEL_MAX_READS 200

// forward declarations for helper functions we add
void setParallelModeHP354(Bme68x& bme);
void runParallelAnalysis(); 
// String analyzeFingerprint(Reading *arr, int nReads, float baselineValue);
float getDropPercentage(float R);
void setLastGasResistance(float R);


// ----------------- setup() (unchanged except minor) -----------------
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
    int8_t max = 10;
    float values[max];
    int8_t index = 0;

    // Stabilization loop
    for (int counter = 0; counter < STABILIZATION_COUNTER; counter++){
        stabilizationLED();
        delayMicroseconds(bme.getMeasDur()); // wait for measurement
        if (bme.fetchData()) {
            bme.getData(data);
            if (data.status == NEW_GAS_MEAS) {
                values[index] = data.gas_resistance;
                if(index >= 10){
                    index = 0;
                    // computeDropSpeed(values, max, 0.1f);
                    // processReadingPercent(millis(), data.gas_resistance);

                }

                index++;
            
                logSerial(data, 100, bme.getUniqueId(), 0.0f);
            }
            
        }
        offLED();
        setForcedModeHeat(bme); // trigger next forced measurement
    }
}

// ----------------- parallelMode helper you already had (kept) -----------------
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

// ----------------- getDropPercentage / baseline setter (unchanged) -----------------
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



// ----------------- Modified loop(): uses runParallelAnalysis() when >10% -----------------
void loop() {
    // Sleep period
    setSleepMode(bme);
    delay(SLEEP_DELAY_MS);
    
    float temperature = 0;
    float forcedResistance = 0;
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
        logSerial(data, 99, bme.getUniqueId(), 0.0f);

        
        if(i + 1 >= MEASUREMENT_OVERSAMPLING){
            forcedResistance = data.gas_resistance;
            // calculatePercentVelocity(millis(), data.gas_resistance, data.gas_index);
            // processReading(millis(), data.gas_resistance);
        }
    }

    offLED();
    Serial.printf("- CurrentBaseline: %.2f \n", baseline);
    float percentage = getDropPercentage(forcedResistance);
    

    if(percentage > 10.0f){
        // New behavior: enter parallel-mode analysis to get fingerprint
        Serial.printf("-Drop %.2f%% > 10%%, switching to parallel analysis...\n", percentage);
        
        
        // while(true){
        //     runParallelAnalysis();
        // }
        do{
            float previousGas = 0;
            unsigned long startMillis = millis();
            float firstGas = 0;
            float lastGas = 0;
            for (int i = 0; i < MEASUREMENT_OVERSAMPLING; i++) {
                
                offLED();
                setForcedMode(bme); // trigger measurement
                delayMicroseconds(bme.getMeasDur()); // wait for measurement to finish
                if (bme.fetchData()) bme.getData(data);
                measurementLED();
                // if(i == 0){
                //     previousGas = data.gas_resistance;
                // }
                if (i == 0) {
                    firstGas = data.gas_resistance;   // baseline of this batch
                }
                lastGas = data.gas_resistance;       // keep updating until the last
                if(i == 1){ 
                    temperature = data.temperature;
                }
                logSerial(data, 99, bme.getUniqueId(), 0.0f);
                
                // processReading(millis(), data.gas_resistance);
                // calculatePercentVelocity(millis(), data.gas_resistance, previousGas, data.gas_index);
                
                // if(i + 1 >= MEASUREMENT_OVERSAMPLING){
                //     forcedResistance = data.gas_resistance;
                //     percentage = getDropPercentage(forcedResistance);
                //     // getDrop(millis(), data.gas_resistance, data.gas_index);
                //     // getRecover(millis(), data.gas_resistance, data.gas_index);
                //     Serial.printf("Percentage: %.2f" , percentage);
                //     // processReading(millis(), data.gas_resistance);
                //     // calculatePercentVelocity(millis(), data.gas_resistance, previousGas, data.gas_index);
                // }
            }
                percentage = getDropPercentage(data.gas_resistance);
                unsigned long endMillis = millis();
                float deltaTime = (endMillis - startMillis) / 1000.0f;
                if (deltaTime <= 0) deltaTime = 0.001;

                float percentChange = ((lastGas - firstGas) / firstGas) * 100.0f;
                float velocityPercent = percentChange / deltaTime;

                Serial.printf("\n-Δ%%=%.2f over %.2fs => %.2f %%/s\n", 
                            percentChange, deltaTime, velocityPercent);

                if (velocityPercent < 0) {
                    Serial.printf("-Drop = %.2f %%/s\n", -velocityPercent);
                } else {
                    Serial.printf("-Recovery = %.2f %%/s\n", velocityPercent);
                }
                Serial.printf("-Percentage %.2f", percentage);
            offLED();
        } while(percentage > 10.0f);
        
        
        
        

    } else {
        setLastGasResistance(forcedResistance);    
        Serial.printf("-Drop: %.2f %%\t\n", percentage);
        Serial.printf("-Rounded: %2.f %%\t\n", percentage);
        Serial.printf("-Baseline: %.2f \n", baseline);
        Serial.printf("-Temperature: %.2f C\n", temperature);
    }
    
}



// ----------------- New function: runParallelAnalysis -----------------
// Sets parallel profile, collects readings, logs them, then analyzes fingerprint.
void runParallelAnalysis(){
    // Set parallel profile (your implementation)
    setParallelModeHP354(bme);

    // Prepare collection buffer
    Reading reads[PARALLEL_MAX_READS];
    int idx = 0;
    unsigned long startTime = millis();
    uint16_t measDur = bme.getMeasDur(BME68X_PARALLEL_MODE) / 1000; // approx ms
    if (measDur < 1) measDur = bme.getMeasDur() / 1000;

    // Collect for a limited time / limited samples, whichever hits first
    const unsigned long collectTimeout = 90000UL; // 90 seconds max collection
    const int maxSamples = PARALLEL_MAX_READS;
    int8_t max = 10;
    float values[max];
    int8_t index = 0;
    Serial.println("-Starting parallel-mode collection...");
    while ((millis() - startTime) < collectTimeout && idx < maxSamples) {



        // Wait a bit longer than meas duration to ensure new fields ready
        delay(measDur + 50);

        if (bme.fetchData()) {
            uint8_t nFieldsLeft = 0;
            do {
                nFieldsLeft = bme.getData(data);
                if (data.status == NEW_GAS_MEAS) {
                    // store reading (last gas_resistance for this field)
                    reads[idx].t = millis();
                    reads[idx].gas_resistance = data.gas_resistance;
                    reads[idx].temperature = data.temperature;
                    reads[idx].gas_index = data.gas_index;
                    // if(data.gas_index == 6.f){
                    //     values[index] = data.gas_resistance;
                    //     processReadingPercent(millis(), data.gas_resistance);
                    // }
                    

                    logSerial(data, data.gas_index, bme.getUniqueId(), 0.0f);
                    // calculatePercentVelocity(millis(), data.gas_resistance, data.gas_index);
                    idx++;
                    if (idx >= maxSamples) break;
                }
            } while (nFieldsLeft);
        }
        measurementLED();
    }

    Serial.printf("-Collected %d parallel readings\n", idx);
    
}



