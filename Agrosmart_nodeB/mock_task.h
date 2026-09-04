#pragma once

// mock_task.h is a no-op in production. The entire simulation is gated
// behind DEVELOPMENT_MODE so the linker produces no mock symbols in a
// production build.
#ifdef DEVELOPMENT_MODE
void vMockTask(void* pvParameters);
#endif
