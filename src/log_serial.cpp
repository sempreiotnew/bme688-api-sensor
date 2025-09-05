#include "bme68xLibrary.h"

void logSerial(bme68xData data, uint16_t step, uint32_t id, float drop_percent){
    Serial.print(id);
    Serial.print(",");
    Serial.print(0);
    Serial.print(",");
    Serial.print(millis());
    Serial.print(",");
    Serial.print(step > 10 ? step : data.gas_index);
    Serial.print(",");
    Serial.print(data.meas_index);
    Serial.print(",");
    Serial.print(data.temperature);
    Serial.print(",");
    Serial.print(data.pressure);
    Serial.print(",");
    Serial.print(data.humidity);
    Serial.print(",");
    Serial.print(data.gas_resistance);
    Serial.print(",");
    Serial.print(String(data.status, HEX));
    // Serial.print(",");
    // Serial.print(drop_percent, 2); 
    Serial.println("");
}
