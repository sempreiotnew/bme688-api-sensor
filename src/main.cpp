#include "Arduino.h"
#include "bme68xLibrary.h"
#include <led_controller.h>
#include <log_serial.h>
#include <sensor_manager.h>
#include <vector>

#define STABILIZATION_COUNTER   10
#define SLEEP_DELAY_MS  10000
#define MEASUREMENT_OVERSAMPLING 20 //Total measurements in forced mode 
#define NEW_GAS_MEAS (BME68X_GASM_VALID_MSK | BME68X_HEAT_STAB_MSK | BME68X_NEW_DATA_MSK)

#define STEP0 0
#define STEP1 1
#define STEP2 2
#define STEP3 3
#define STEP4 4

#define MAX_READING_PER_STEP_PARALLEL_MODE 1

Bme68x bme;
bme68xData data;
float baseline = 0.0f;
float baselineSleep = 0.0f;
bool isBottomDrop = false;
int8_t stabilizeAfterOcurrency = 5;

bool firstParallelReading = true;

void stabilizationLED() { setColorRGB(0,0,255); } 
void sleepLED() { setColorRGB(255,0,0); } 
void measurementLED() { setColorRGB(0,255,0); } 
void offLED() { setColorRGB(0,0,0); } 
float getDropPercentageFromAtoB(float A, float B);

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


void getDrop(unsigned long currentMillis, float currentGas, float gasIndex){
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

void getRecover(unsigned long currentMillis, float currentGas, float gasIndex){
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
void setParallelModeCigarette(Bme68x& bme);

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
    
    float temporaryGasData = 0;

    // Stabilization loop
    // for (int counter = 0; counter < STABILIZATION_COUNTER; counter++){
    //     stabilizationLED();
    //     delayMicroseconds(bme.getMeasDur()); // wait for measurement
    //     if (bme.fetchData()) {
    //         bme.getData(data);
    //         if (data.status == NEW_GAS_MEAS) {
    //             logSerial(data, 100, bme.getUniqueId(), 0.0f);
    //             temporaryGasData = data.gas_resistance;
    //         }
            
    //     }

    //     setLastGasResistance(temporaryGasData);
    //     offLED();
    //     setForcedMode(bme); // trigger next forced measurement
    // }
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
    // while(true){
    //         runParallelAnalysis();
    // }
    // Sleep period
    setSleepMode(bme);
    delay(SLEEP_DELAY_MS);
    
    float temperature = 0;
    float forcedResistance = 0;
    int i = 0;
    int j = 0;
    
    while(i < MEASUREMENT_OVERSAMPLING || isBottomDrop){
        
        offLED();
        // isBottomDrop ? setForcedModeHeat(bme) : setForcedMode(bme); // trigger measurement
        setForcedMode(bme); // trigger measurement
        delayMicroseconds(bme.getMeasDur()); // wait for measurement to finish
        if (bme.fetchData()) bme.getData(data);
        measurementLED();
        if(i == 1){ 
            temperature = data.temperature;
            float drop = getDropPercentageFromAtoB(data.gas_resistance, baselineSleep );
            Serial.printf("-Bottom drop %.2f %% \n", drop);
            if(drop >= 5.0f){
                isBottomDrop = true;
            } else {
                isBottomDrop = false;
            }
            baselineSleep = data.gas_resistance;
            
            
        }
        logSerial(data, 99, bme.getUniqueId(), 0.0f);

        
        if(i + 1 >= MEASUREMENT_OVERSAMPLING && !isBottomDrop){
            forcedResistance = data.gas_resistance;
        }
        i++;

        if(isBottomDrop){
            j++;
        }

        if( j >= 2000){
            isBottomDrop = false;
        }
        
    }
    // Forced measurements
    // for (int i = 0; i < MEASUREMENT_OVERSAMPLING && !isBottomDrop; i++) {
        
    //     offLED();
    //     isBottomDrop ? setForcedModeHeat(bme) : setForcedMode(bme); // trigger measurement
    //     delayMicroseconds(bme.getMeasDur()); // wait for measurement to finish
    //     if (bme.fetchData()) bme.getData(data);
    //     measurementLED();
    //     if(i == 1){ 
    //         temperature = data.temperature;
    //         float drop = getDropPercentageFromAtoB(data.gas_resistance, baselineSleep );
    //         Serial.printf("-Bottom drop %.2f %% \n", drop);
    //         if(drop >= 5.0f){
    //             isBottomDrop = true;
    //         } else {
    //             isBottomDrop = false;   
    //         }
    //         baselineSleep = data.gas_resistance;
            
            
    //     }
    //     logSerial(data, 99, bme.getUniqueId(), 0.0f);

        
    //     if(i + 1 >= MEASUREMENT_OVERSAMPLING){
    //         forcedResistance = data.gas_resistance;
    //     }
    // }

    offLED();
    // Serial.printf("- CurrentBaseline: %.2f \n", baseline);
    float percentage = getDropPercentage(forcedResistance);
    
    

    // if(percentage > 10.0f){
    //     // New behavior: enter parallel-mode analysis to get fingerprint
    //     Serial.printf("-Drop %.2f%% > 10%%, switching to HP (300, 400) analysis...\n", percentage);
        
        
    //     while(true){
    //         runParallelAnalysis();
    //     }

    //     firstParallelReading = true;
    //     // do{
    //     //     float previousGas = 0;
    //     //     unsigned long startMillis = millis();
    //     //     float firstGas = 0;
    //     //     float lastGas = 0;
    //     //     for (int i = 0; i < MEASUREMENT_OVERSAMPLING; i++) {
                
    //     //         offLED();
    //     //         setForcedModeHeat(bme); // trigger measurement
    //     //         delayMicroseconds(bme.getMeasDur()); // wait for measurement to finish
    //     //         if (bme.fetchData()) bme.getData(data);
    //     //         measurementLED();
    //     //         // if(i == 0){
    //     //         //     previousGas = data.gas_resistance;
    //     //         // }
    //     //         if (i == 0) {
    //     //             firstGas = data.gas_resistance;   // baseline of this batch
    //     //         }
    //     //         lastGas = data.gas_resistance;       // keep updating until the last
    //     //         if(i == 1){ 
    //     //             temperature = data.temperature;
    //     //         }
    //     //         logSerial(data, 99, bme.getUniqueId(), 0.0f);
                
    //     //         // processReading(millis(), data.gas_resistance);
    //     //         // calculatePercentVelocity(millis(), data.gas_resistance, previousGas, data.gas_index);
                
    //     //         // if(i + 1 >= MEASUREMENT_OVERSAMPLING){
    //     //         //     forcedResistance = data.gas_resistance;
    //     //         //     percentage = getDropPercentage(forcedResistance);
    //     //         //     // getDrop(millis(), data.gas_resistance, data.gas_index);
    //     //         //     // getRecover(millis(), data.gas_resistance, data.gas_index);
    //     //         //     Serial.printf("Percentage: %.2f" , percentage);
    //     //         //     // processReading(millis(), data.gas_resistance);
    //     //         //     // calculatePercentVelocity(millis(), data.gas_resistance, previousGas, data.gas_index);
    //     //         // }
    //     //     }
    //     //         percentage = getDropPercentage(data.gas_resistance);
    //     //         // unsigned long endMillis = millis();
    //     //         // float deltaTime = (endMillis - startMillis) / 1000.0f; //time of measurements
    //     //         // if (deltaTime <= 0) deltaTime = 0.001;

    //     //         // float percentChange = ((lastGas - firstGas) / firstGas) * 100.0f;
    //     //         // float velocityPercent = percentChange / deltaTime;

    //     //         // Serial.printf("\n-Δ%%=%.2f over %.2fs => %.2f %%/s\n", 
    //     //         //             percentChange, deltaTime, velocityPercent);

    //     //         // if (velocityPercent < 0) {
    //     //         //     Serial.printf("-DROP = %.2f %%/s\n", -velocityPercent);
    //     //         // } else {
    //     //         //     Serial.printf("-RECOVERY = %.2f %%/s\n", velocityPercent);
    //     //         // }
    //     //         Serial.printf("-Baseline Percentage %.2f\n", percentage);
                
    //     //     offLED();
    //     // } while(percentage > 10.0f);

    //     setLastGasResistance(forcedResistance);

    // } else {
    //     setLastGasResistance(forcedResistance);    
    //     Serial.printf("-Rounded: %2.f %%\t\n", percentage);
    //     Serial.printf("-Baseline: %.2f \n", baseline);
    //     Serial.printf("-Baseline Drop: %.2f %%\t\n", percentage);
    //     Serial.printf("-Temperature: %.2f C\n", temperature);
    // }
    
        setLastGasResistance(forcedResistance);    
        Serial.printf("-Rounded: %2.f %%\t\n", percentage);
        Serial.printf("-Baseline: %.2f \n", baseline);
        Serial.printf("-Baseline Drop: %.2f %%\t\n", percentage);
        Serial.printf("-Temperature: %.2f C\n", temperature);
}

float getDropPercentageFromAtoB(float A, float B){
    if (A <= 0 || B <= 0) return 0.0f; // safety check

    float dropPercent = 0;
    dropPercent = (B - A) / B ;

    if(dropPercent > 0) {
        return dropPercent * 100;
    } else {
        return 0.0f;
    }
}

float getDropPercentageFromResistances(float A, float B){
    if (A <= 0 || B <= 0) return 0.0f; // safety check

    float dropPercent = 0;
    dropPercent = (logf(B) - logf(A)) / logf(B) ;

    if(dropPercent > 0) {
        return dropPercent * 100;
    } else {
        return 0.0f;
    }
}

float getDropPercentageLogarithm(float R, float baseline){
    if (R <= 0 || baseline <= 0) return 0.0f; // safety check

    float dropPercent = 0;
    dropPercent = (logf(baseline) - logf(R)) / logf(baseline) ;

    if(dropPercent > 0) {
        return dropPercent * 100;
    } else {
        return 0.0f;
    }
}

float getRecoveryPercentageLogarithm(float R, float baseline){
    if (R <= 0 || baseline <= 0) return 0.0f; // safety check

    float recoveryPercent = 0;
    recoveryPercent = (logf(R) - logf(baseline)) / logf(baseline);

    if(recoveryPercent > 0) {
        return recoveryPercent * 100;
    } else {
        return 0.0f;
    }
}

// Calculates % drop given max, min, and current baseline (log normalized)
float getDropPercentLogRange(float Rbaseline, float Rmax, float Rmin) {
    if (Rbaseline <= 0.0f || Rmax <= 0.0f || Rmin <= 0.0f) return 0.0f;
    if (Rmax <= Rmin) return 0.0f; // invalid range

    float numerator   = logf(Rmax) - logf(Rbaseline);
    float denominator = logf(Rmax) - logf(Rmin);

    float percent = (numerator / denominator) * 100.0f;

    if (percent < 0.0f) return 0.0f;
    if (percent > 100.0f) return 100.0f;

    return percent;
}

float getRecoveryPercentLogRange(float Rbaseline, float Rmax, float Rmin) {
    if (Rbaseline <= 0.0f || Rmax <= 0.0f || Rmin <= 0.0f) return 0.0f;
    if (Rmax <= Rmin) return 0.0f; // invalid range

    float numerator   = logf(Rbaseline) - logf(Rmin);
    float denominator = logf(Rmax) - logf(Rmin);

    float percent = (numerator / denominator) * 100.0f;

    if (percent < 0.0f) return 0.0f;
    if (percent > 100.0f) return 100.0f;

    return percent;
}

enum class Mode { DROP, RECOVERY };

float getPercentLogRange(float R, float Rmax, float Rmin, Mode mode) {
    if (R <= 0.0f || Rmax <= 0.0f || Rmin <= 0.0f) return 0.0f;
    float denom = logf(Rmax) - logf(Rmin);
    if (denom <= 0.0f) return 0.0f;

    float pct = 0.0f;
    if (mode == Mode::DROP) {
        pct = (logf(Rmax) - logf(R)) / denom * 100.0f;
    } else {
        pct = (logf(R) - logf(Rmin)) / denom * 100.0f;
    }
    if (pct < 0.0f) pct = 0.0f;
    if (pct > 100.0f) pct = 100.0f;
    return pct;
}

void showStatistics(float lastGas, int gas_index, float values[]){
    float temp = values[0];
    for(int i=0; i < MAX_READING_PER_STEP_PARALLEL_MODE; i++){
        Serial.printf("-------------------STEP %d------------------", gas_index);
        Serial.printf("\n-Previous: %.2f", lastGas );
        Serial.printf("-Current: %.2f\n", values[i] );
        if (values[i] < temp) {
            temp = values[i];   // update if a smaller value is found
        }

        // float drop = getDropPercentageFromResistances(gas0Aux[i], gas4Aux[i]);
        float drop = getDropPercentageLogarithm(values[i], lastGas);
        float recovery = getRecoveryPercentageLogarithm(values[i], lastGas);

        if(drop <= 0){
            Serial.printf("-Recovery %.2f %%\n\n", recovery);
            // float recoveryRange = getRecoveryPercentLogRange(temp, 102400000.f, 5684.85f);
            float recoveryRange = getRecoveryPercentLogRange(temp, 102400000.f, 5684.85f);
            
            Serial.printf("-Recovery Range %.2f %%\n", recoveryRange);
        } else {
            Serial.printf("-Drop %.2f %%\n\n", drop);    
            float dropRange = getDropPercentLogRange(temp, 102400000.f, 5684.85f);
            Serial.printf("-Drop Range %.2f %%\n", dropRange );
        }
        
    }

    // if(gas_index == 2){
    //     float total = getDropPercentageLogarithm(temp, 102400000.f);
    //     Serial.printf("-DropTotal 1e8 %.2f%% \n", total);
        
    // }
    Serial.println("------------------------------------------");
    Serial.println("\n");

    
}







