#ifndef CONFIG_MANAGER_H
#define CONFIG_MANAGER_H

#include "structs.h"
#include "enums.h"

// Active farm configuration — populated at boot from SD or defaults.
extern FarmProfile activeProfile;

// Attempt to read /CONFIG/farm_profile.json from SD card.
// Returns true and populates *p on success.
// Returns false if the file is missing, malformed, or the SD is offline —
// caller should fall back to hardcoded defaults.
// Caller MUST hold spiMutex before calling (SD is on the shared SPI bus).
bool loadProfileFromSD(FarmProfile* p);

// Write the current activeProfile back to /CONFIG/farm_profile.json.
// Useful after an OTA config update or a crop/stage change.
// Caller MUST hold spiMutex.
bool saveProfileToSD(const FarmProfile* p);

#endif // CONFIG_MANAGER_H