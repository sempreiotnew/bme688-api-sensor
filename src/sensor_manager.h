#include "bme68xLibrary.h"

void initializeSensor(Bme68x& bme);
void setForcedMode(Bme68x& bme);
void setForcedModeHeat(Bme68x& bme);
void setForcedModeTemp(Bme68x& bme, uint16_t heaterTemp);
void setParallelMode(Bme68x& bme);
void setSleepMode(Bme68x& bme);
void setParallelModeHP354(Bme68x& bme);
void setParallelModeHP501(Bme68x& bme);