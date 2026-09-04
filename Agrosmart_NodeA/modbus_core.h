#pragma once

#include "structs.h"

void initModbus();
bool readSoilSensor(MasterSensorRecord &r);
