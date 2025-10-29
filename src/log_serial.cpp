#include "bme68xLibrary.h"

void logSerial(bme68xData data, uint16_t step, uint32_t id, uint8_t index){
    Serial.print(id);
    Serial.print(",");
    Serial.print(index);
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
    Serial.println("");
}
