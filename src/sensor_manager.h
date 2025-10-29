#include "bme68xLibrary.h"

void initializeSensor(Bme68x& bme);
void setForcedMode(Bme68x& bme);
void setForcedModeCalib(Bme68x& bme);
void setParallelMode(Bme68x& bme);
void setSleepMode(Bme68x& bme);
void setParallelModeHP354(Bme68x& bme);
void setForcedModeParameters(Bme68x& bme, uint8_t osTemp, uint8_t osPres, uint8_t osHum);