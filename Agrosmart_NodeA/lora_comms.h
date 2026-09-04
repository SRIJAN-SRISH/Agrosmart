#pragma once

#include "structs.h"

bool initLora();
bool transmitPacket(const MasterSensorRecord &record);
bool loraIsReady();
