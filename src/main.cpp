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

bool firstParallelReading = true;
float lastGas0 = 0.0f;
float lastGas2 = 0.0f;
float lastGas4 = 0.0f;

uint8_t gas0Idx = 0;
uint8_t gas1Idx = 0;
uint8_t gas2Idx = 0;
uint8_t gas3Idx = 0;
uint8_t gas4Idx = 0;
float gas0[10];
float gas1[10];
float gas2[10];
float gas3[10];
float gas4[10];

uint8_t gas0AuxIdx = 0;
uint8_t gas1AuxIdx = 0;
uint8_t gas2AuxIdx = 0;
uint8_t gas3AuxIdx = 0;
uint8_t gas4AuxIdx = 0;
float gas0Aux[10];
float gas1Aux[10];
float gas2Aux[10];
float gas3Aux[10];
float gas4Aux[10];

float gas0Baseline = 0.f;
float gas1Baseline = 0.f;
float gas2Baseline = 0.f;
float gas3Baseline = 0.f;
float gas4Baseline = 0.f;

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
    while(true){
            runParallelAnalysis();
    }
    //Sleep period
    // setSleepMode(bme);
    // delay(SLEEP_DELAY_MS);
    
    // float temperature = 0;
    // float forcedResistance = 0;
    // // Forced measurements
    // for (int i = 0; i < MEASUREMENT_OVERSAMPLING; i++) {
        
    //     offLED();
    //     setForcedMode(bme); // trigger measurement
    //     delayMicroseconds(bme.getMeasDur()); // wait for measurement to finish
    //     if (bme.fetchData()) bme.getData(data);
    //     measurementLED();
    //     if(i == 1){ 
    //         temperature = data.temperature;
    //     }
    //     logSerial(data, 99, bme.getUniqueId(), 0.0f);

        
    //     if(i + 1 >= MEASUREMENT_OVERSAMPLING){
    //         forcedResistance = data.gas_resistance;
    //     }
    // }

    // offLED();
    // // Serial.printf("- CurrentBaseline: %.2f \n", baseline);
    // float percentage = getDropPercentage(forcedResistance);
    

    // if(percentage > 10.0f){
    //     // New behavior: enter parallel-mode analysis to get fingerprint
    //     // Serial.printf("-Drop %.2f%% > 10%%, switching to HP (300, 400) analysis...\n", percentage);
        
        
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
    //     // Serial.printf("-Rounded: %2.f %%\t\n", percentage);
    //     // Serial.printf("-Baseline: %.2f \n", baseline);
    //     // Serial.printf("-Baseline Drop: %.2f %%\t\n", percentage);
    //     // Serial.printf("-Temperature: %.2f C\n", temperature);
    // }
    
}

void getStepsResistanceData(float R, int step){
    
    switch (step)
    {
    case STEP0:
        if(gas0Idx > MAX_READING_PER_STEP_PARALLEL_MODE) break;
        lastGas0  = gas0Baseline;
        gas0Baseline  = R;
        gas0[gas0Idx] = R;
        gas0Idx++;
        break;

    case STEP1:
        if(gas1Idx > MAX_READING_PER_STEP_PARALLEL_MODE) break;
        if(gas1Idx == 0) gas1Baseline  = R;
        gas1[gas1Idx] = R;
        gas1Idx++;
        break;
    
    case STEP2:        
        if(gas2Idx > MAX_READING_PER_STEP_PARALLEL_MODE) break;
        lastGas2  = gas2Baseline;
        gas2Baseline  = R;
        gas2[gas2Idx] = R;
        gas2Idx++;
        break;

    case STEP3:
        if(gas3Idx > MAX_READING_PER_STEP_PARALLEL_MODE) break;
        if(gas3Idx == 0) gas3Baseline  = R;
        gas3[gas3Idx] = R;
        gas3Idx++;
        break;

    case STEP4:
        if(gas4Idx > MAX_READING_PER_STEP_PARALLEL_MODE) break;
        lastGas4  = gas4Baseline;
        gas4Baseline  = R;
        gas4[gas4Idx] = R;
        gas4Idx++;
        break;
                        
    default:
        break;
    }  
}

bool isArrayStepsFullFilled(){
    // Serial.printf("-%d", gas0Idx);
    // Serial.printf("-%d", gas1Idx);
    // Serial.printf("-%d", gas2Idx);
    // Serial.printf("-%d", gas3Idx);
    // Serial.printf("-%d", gas4Idx);

    if( gas0Idx >= MAX_READING_PER_STEP_PARALLEL_MODE && 
        gas1Idx >= MAX_READING_PER_STEP_PARALLEL_MODE &&
        gas2Idx >= MAX_READING_PER_STEP_PARALLEL_MODE && 
        gas3Idx >= MAX_READING_PER_STEP_PARALLEL_MODE &&
        gas4Idx >= MAX_READING_PER_STEP_PARALLEL_MODE){
            // Serial.println("-Fullfiled!");
            return true;
        }

     return false; 
}

void setAuxArrays(){
    memcpy(gas0Aux, gas0, sizeof(gas0));
    memcpy(gas1Aux, gas1, sizeof(gas1));
    memcpy(gas2Aux, gas2, sizeof(gas2));
    memcpy(gas3Aux, gas3, sizeof(gas3));
    memcpy(gas4Aux, gas4, sizeof(gas4));

}

void setAuxArraysEmpty(){
    memset(gas0, 0, sizeof(gas0));
    memset(gas1, 0, sizeof(gas1));
    memset(gas2, 0, sizeof(gas2));
    memset(gas3, 0, sizeof(gas3));
    memset(gas3, 0, sizeof(gas4));
    gas0Idx = 0;
    gas1Idx = 0;
    gas2Idx = 0;
    gas3Idx = 0;
    gas4Idx = 0;
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

#define MAX_STEPS 5
float dropFingerprint[MAX_STEPS];

void buildFingerprint(float valuesPerStep[MAX_STEPS], float baseline) {
  for (int i = 0; i < MAX_STEPS; i++) {
    dropFingerprint[i] = getDropPercentageLogarithm(valuesPerStep[i], baseline);
  }
}

// String classifyFingerprint() {
//   // Normalize
//   float maxVal = 0;
//   for (int i = 0; i < MAX_STEPS; i++) if (dropFingerprint[i] > maxVal) maxVal = dropFingerprint[i];
//   float norm[MAX_STEPS];
//   for (int i = 0; i < MAX_STEPS; i++) norm[i] = (maxVal > 0) ? dropFingerprint[i] / maxVal : 0;

//   // Count how many "strong dips" above 0.6
//   int strongCount = 0;
//   for (int i = 0; i < MAX_STEPS; i++) {
//     if (norm[i] > 0.6) strongCount++;
//   }

//   // Heuristic classification
//   if (strongCount >= 2) {
//     return "Alcohol";
//   } else if (strongCount == 1) {
//     return "Cigarette";
//   } else {
//     return "Unknown";
//   }
// }

// // Assumes dropFingerprint[MAX_STEPS] is already filled with drop % per step (0..100)
// String classifyFingerprint() {
//   // --- normalize
//   float maxVal = 0.0f;
//   for (int i = 0; i < MAX_STEPS; i++) if (dropFingerprint[i] > maxVal) maxVal = dropFingerprint[i];

//   float norm[MAX_STEPS];
//   for (int i = 0; i < MAX_STEPS; i++) norm[i] = (maxVal > 0.0f) ? dropFingerprint[i] / maxVal : 0.0f;

//   // --- print fingerprint initialized with '-'
//   String fp = "";
//   for (int i = 0; i < MAX_STEPS; i++) {
//     if (i) fp += ",";
//     fp += String(norm[i], 2); // two decimals
//   }
//   Serial.print("-Fingerprint: ");
//   Serial.println(fp);

//   // --- parameters (tune these if needed)
//   const float peakThreshold = 0.45f;   // value (0..1) to consider a "strong" peak
//   const float smallThreshold = 0.18f;  // value (0..1) to consider part of the event width
//   const float asymThreshold = 0.22f;   // asymmetry threshold to trigger "right drift"

//   // --- find peaks (local maxima above peakThreshold)
//   int peakIdxs[MAX_STEPS];
//   int peakCount = 0;
//   int maxIndex = 0;
//   float maxValNorm = 0.0f;
//   for (int i = 0; i < MAX_STEPS; i++) {
//     if (norm[i] > maxValNorm) {
//       maxValNorm = norm[i];
//       maxIndex = i;
//     }
//     float left = (i > 0) ? norm[i - 1] : -1.0f;
//     float right = (i < MAX_STEPS - 1) ? norm[i + 1] : -1.0f;
//     bool localMax = (norm[i] >= left) && (norm[i] >= right);
//     if (localMax && norm[i] >= peakThreshold) {
//       peakIdxs[peakCount++] = i;
//     }
//   }

//   // --- compute width around dominant peak (count contiguous steps >= smallThreshold)
//   int start = maxIndex, end = maxIndex;
//   while (start > 0 && norm[start - 1] >= smallThreshold) start--;
//   while (end < MAX_STEPS - 1 && norm[end + 1] >= smallThreshold) end++;
//   int widthSteps = end - start + 1;

//   // --- compute asymmetry around dominant peak
//   float sumLeft = 0.0f, sumRight = 0.0f, sumTotal = 0.0f;
//   for (int i = 0; i < MAX_STEPS; i++) sumTotal += norm[i];
//   for (int i = 0; i < maxIndex; i++) sumLeft += norm[i];
//   for (int i = maxIndex + 1; i < MAX_STEPS; i++) sumRight += norm[i];
//   float asym = 0.0f;
//   if (sumTotal > 0.0f) asym = (sumRight - sumLeft) / sumTotal; // positive -> more on right

//   // --- debug print
//   Serial.printf("-peaks=%d maxIdx=%d width=%d asym=%.2f\n", peakCount, maxIndex, widthSteps, asym);

//   // --- decision logic (heuristic)
//   // 1) multiple strong peaks close together -> Alcohol (W-shape)
//   if (peakCount >= 2) {
//     // if first two peaks are close (adjacent or one step apart) it's classic W
//     if (peakCount >= 2) {
//       int d = abs(peakIdxs[1] - peakIdxs[0]);
//       if (d <= 2) {
//         Serial.println("-Decision: Alcohol (multi-peak close)");
//         return "Alcohol";
//       }
//       // if peaks are separated but both strong, still likely alcohol
//       Serial.println("-Decision: Alcohol (multi-peak)");
//       return "Alcohol";
//     }
//   }

//   // 2) single or no strong peak -> use width and asymmetry
//   if (peakCount == 1) {
//     // narrow spike (small width) => quick event (likely alcohol)
//     if (widthSteps <= 2) {
//       Serial.println("-Decision: Alcohol (narrow spike)");
//       return "Alcohol";
//     }

//     // wide event or right-heavy energy => cigarette drift
//     if (widthSteps >= 3 || asym > asymThreshold) {
//       Serial.println("-Decision: Cigarette (wide/asymmetric right-drift)");
//       return "Cigarette";
//     }

//     // default fallback for single peak
//     Serial.println("-Decision: Cigarette (single peak fallback)");
//     return "Cigarette";
//   }

//   // 3) no strong peaks — maybe low concentration or noisy: use coarse rules
//   // if there is a mild right-drift, say cigarette; if two moderate adjacent bumps -> alcohol
//   if (asym > asymThreshold) {
//     Serial.println("-Decision: Cigarette (no strong peaks but right-drift)");
//     return "Cigarette";
//   }

//   Serial.println("-Decision: Unknown");
//   return "Unknown";
// }

// Detect only "Cigarette", otherwise return "Thinking" (and print diagnostics)
String classifyFingerprint() {
  // --- normalize
  float maxVal = 0.0f;
  for (int i = 0; i < MAX_STEPS; i++) if (dropFingerprint[i] > maxVal) maxVal = dropFingerprint[i];

  float norm[MAX_STEPS];
  for (int i = 0; i < MAX_STEPS; i++) norm[i] = (maxVal > 0.0f) ? dropFingerprint[i] / maxVal : 0.0f;

  // --- print fingerprint
  String fp = "";
  for (int i = 0; i < MAX_STEPS; i++) {
    if (i) fp += ",";
    fp += String(norm[i], 2);
  }
  Serial.print("-Fingerprint: ");
  Serial.println(fp);

  // --- thresholds (tune if needed)
  const float peakThreshold   = 0.50f;  // for optional peak count (not primary here)
  const float smallThreshold  = 0.18f;  // used to compute width around dominant peak
  const float asymThreshold   = 0.12f;  // right-drift threshold
  const float step2Shallow    = 0.45f;  // if norm[2] <= step2Shallow => shallow step2 (evidence for cigarette)
  const int   widthForCig     = 2;      // widthSteps >= this tends to indicate "slow drop"

  // --- find dominant index and optional peak count
  int peakCount = 0;
  int maxIndex = 0;
  float maxValNorm = 0.0f;
  for (int i = 0; i < MAX_STEPS; i++) {
    if (norm[i] > maxValNorm) {
      maxValNorm = norm[i];
      maxIndex = i;
    }
    float left = (i > 0) ? norm[i - 1] : -1.0f;
    float right = (i < MAX_STEPS - 1) ? norm[i + 1] : -1.0f;
    bool localMax = (norm[i] >= left) && (norm[i] >= right);
    if (localMax && norm[i] >= peakThreshold) peakCount++;
  }

  // --- compute width around dominant peak (how many contiguous steps >= smallThreshold)
  int start = maxIndex, end = maxIndex;
  while (start > 0 && norm[start - 1] >= smallThreshold) start--;
  while (end < MAX_STEPS - 1 && norm[end + 1] >= smallThreshold) end++;
  int widthSteps = end - start + 1;

  // --- compute asymmetry (right side energy minus left side)
  float sumLeft = 0.0f, sumRight = 0.0f, sumTotal = 0.0f;
  for (int i = 0; i < MAX_STEPS; i++) sumTotal += norm[i];
  for (int i = 0; i < maxIndex; i++) sumLeft += norm[i];
  for (int i = maxIndex + 1; i < MAX_STEPS; i++) sumRight += norm[i];
  float asym = (sumTotal > 0.0f) ? (sumRight - sumLeft) / sumTotal : 0.0f; // positive -> more on right

  // --- step2 safety
  float step2 = 0.0f;
  if (MAX_STEPS > 2) step2 = norm[2];

  // --- print diagnostics
  Serial.printf("-diag: peaks=%d maxIdx=%d width=%d asym=%.2f step2=%.2f\n", peakCount, maxIndex, widthSteps, asym, step2);

  // --- cigarette detection rules (OR-combination of indicators)
  bool isCigarette = false;

  // 1) clear right drift + some width -> cigarette
  if (asym > asymThreshold && widthSteps >= widthForCig) {
    isCigarette = true;
    Serial.println("-Reason: right-drift + width");
  }

  // 2) shallow step2 combined with width or drift -> cigarette (outdoor/indoor light smoke)
  if (!isCigarette && MAX_STEPS > 2) {
    if (step2 <= step2Shallow && (widthSteps >= widthForCig || asym > (asymThreshold/2.0f))) {
      isCigarette = true;
      Serial.println("-Reason: shallow step2 + width/drift");
    }
  }

  // 3) edge peak behavior (peak at step 0 or last + drift/width) -> cigarette
  if (!isCigarette) {
    if ((maxIndex == 0 || maxIndex == (MAX_STEPS - 1)) && (widthSteps >= widthForCig || asym > asymThreshold)) {
      isCigarette = true;
      Serial.println("-Reason: edge peak + width/drift");
    }
  }

  // 4) saturated but still shows right drift or wide -> still cigarette
  if (!isCigarette) {
    if (maxValNorm >= 0.9f && (asym > (asymThreshold/2.0f) || widthSteps >= (widthForCig + 1))) {
      isCigarette = true;
      Serial.println("-Reason: saturated + drift/width");
    }
  }

  if (isCigarette) {
    Serial.println("-Decision: Cigarette");
    return "Cigarette";
  }

  // --- Not confident: print thinking output and return Thinking
  Serial.println("-Thinking: not enough cigarette evidence");
  Serial.printf("-Thinking values -> maxValNorm=%.2f width=%d asym=%.2f step2=%.2f peaks=%d\n",
                maxValNorm, widthSteps, asym, step2, peakCount);
  return "Thinking";
}



void showStatistics(float lastGas, int gas_index, float values[]){
    float temp = values[0];
    for(int i=0; i < MAX_READING_PER_STEP_PARALLEL_MODE; i++){
        Serial.printf("-------------------STEP %d------------------", gas_index);
        Serial.printf("\n-Previous: %.2f", lastGas );
        Serial.printf("-Current: %.2f\n\n", values[i] );
        if (values[i] < temp) {
            temp = values[i];   // update if a smaller value is found
        }

        // float drop = getDropPercentageFromResistances(gas0Aux[i], gas4Aux[i]);
        float drop = getDropPercentageLogarithm(values[i], lastGas);
        float recovery = getRecoveryPercentageLogarithm(values[i], lastGas);


        dropFingerprint[gas_index] = drop;
        
        Serial.printf("-Recovery %.2f %%\n", recovery);
        // float recoveryRange = getRecoveryPercentLogRange(temp, 102400000.f, 5684.85f);
        float recoveryRange = getRecoveryPercentLogRange(temp, 102400000.f, 5684.85f);
        
        Serial.printf("-Recovery Range %.2f %%\n\n", recoveryRange);
    
        Serial.printf("-Drop %.2f %%\n", drop);    
        float dropRange = getDropPercentLogRange(temp, 102400000.f, 5684.85f);
        Serial.printf("-Drop Range %.2f %%\n\n", dropRange );
        
        
    }

    // if(gas_index == 2){
    //     float total = getDropPercentageLogarithm(temp, 102400000.f);
    //     Serial.printf("-DropTotal 1e8 %.2f%% \n", total);
        
    // }
    Serial.println("------------------------------------------");
    Serial.println("\n");

    
    classifyFingerprint();
}

void isCigarette(){
    
    showStatistics(lastGas0, 0, gas0Aux);
    showStatistics(lastGas2, 2, gas2Aux);
    showStatistics(lastGas4, 4, gas4Aux);

    setAuxArraysEmpty();
}

// ----------------- New function: runParallelAnalysis -----------------
// Sets parallel profile, collects readings, logs them, then analyzes fingerprint.
void runParallelAnalysis(){
    // Set parallel profile (your implementation)
    // setParallelModeHP354(bme);
    setParallelModeCigarette(bme);

    // Prepare collection buffer
    Reading reads[PARALLEL_MAX_READS];
    int idx = 0;
    unsigned long startTime = millis();
    uint16_t measDur = bme.getMeasDur(BME68X_PARALLEL_MODE) / 1000; // approx ms
    if (measDur < 1) measDur = bme.getMeasDur() / 1000;

    // Collect for a limited time / limited samples, whichever hits first
    const unsigned long collectTimeout = 90000UL; // 90 seconds max collection
    const int maxSamples = PARALLEL_MAX_READS;
    
    // Serial.println("-Starting parallel-mode collection...");
    // while ((millis() - startTime) < collectTimeout && idx < maxSamples) {
    int counter = 5;
    while (idx < counter) {
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

                    if(firstParallelReading){
                        firstParallelReading = false;
                        idx--;
                    } else {
                        getStepsResistanceData(data.gas_resistance, data.gas_index);
                    }

                    logSerial(data, data.gas_index, bme.getUniqueId(), 0.0f);
                    idx++;
                    
                    if (idx >= maxSamples) break;
                    
                }
            } while (nFieldsLeft);
        }
        measurementLED();
        
    }
    
    // Serial.printf("-Collected %d parallel readings\n", idx);
    // if(isArrayStepsFullFilled()){
    //     setAuxArrays();
    //     isCigarette();    
    //     String result = classifyFingerprint();
    //     Serial.printf("-Detected: %s\n", result.c_str());
    // } else {
    //     Serial.println("-Not full filled");
    //     setAuxArraysEmpty();
    // }
    
}



